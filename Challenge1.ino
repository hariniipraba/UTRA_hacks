#include <Servo.h>

// LED
#define LED A1

// Ultrasonic Sensor connections
#define echo A0
#define trig 2

//Servo connections
#define servo_pin 4

// Left Motor connections
#define ENA 9
#define IN1 8
#define IN2 7

// Right Motor connections
#define IN3 6
#define IN4 5
#define ENB 3 

// Colour Sensor connections
#define S0 0
#define S1 1
#define S2 10
#define S3 11
#define OUT 12
#define OE_PIN 13
int red = 0;
int green = 0;
int blue = 0;
int colour = 1111;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
int rcounter = 0;
int gcounter = 0;
int bcounter = 0;
#define OTHER 1111
#define RED 3333
#define BLUE 4444
#define GREEN 5555

Servo myServo;

int claw_state; 
#define OPEN 1
#define CLOSED 0

// Motor states
#define OFF 0
#define FORWARD 1
#define BACKWARD 2

long distance;
int STATE = 1;
int ROB_SPEED = 0.2462; // robot speed in m/s at L = 140, R = 150
bool motorsRunning = false;
long travelled = 0; 
int t0 = 0;
int t1;
int colour1; int colour2; int colour3; int colour4; int colour5; int prevcolour; int currcolour;
int position = 0;

void setup() {
    Serial.begin(9600);
    
    // Setup USS
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    
    // Setup CS
    pinMode(S0, OUTPUT);
    pinMode(S1, OUTPUT);
    pinMode(S2, OUTPUT);
    pinMode(S3, OUTPUT);
    pinMode(OUT, INPUT);
    pinMode(OE_PIN, OUTPUT);
    digitalWrite(S0, HIGH);
    digitalWrite(S1, LOW);
    digitalWrite(OE_PIN, LOW);

    // Setup servo
    myServo.attach(servo_pin);

    // Setup motors
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    previousTime = millis();
}

int count1 = 0; int count2 = 0;

void loop() {
  colour_sensor();
  
  // STATE = 1
  if (STATE ==1) {
    if (colour == OTHER) {
      mv_forward();
      delay(200);
      STOP();
      delay(100);
      count1++;
      if (count1 >= 16) {
        mv_backward();
        delay(count1*200);
        STOP();
        delay(100);
        rotate_left(30);
        count1 = 0;
      }
    }
    else {
      colour1 = colour;
      STOP();
      delay(5000);
      STATE = 2;
    }
  }

  //STATE = 2
  if (STATE == 2) {
    if (colour == colour1 && position == 0) {
      mv_forward();
      delay(600);
      STOP();
      delay(100);
      position = 1;
    }
    else if ((colour == colour1 && position == 1) || (colour == OTHER)) {
      mv_backward();
      delay(600);
      STOP();
      delay(100);
      position = 0;
      rotate_left(30);
    }
    else if (colour != colour1 && colour != OTHER) {
      STOP();
      colour2 = colour;
      prevcolour = colour;
      delay(5000);
      STATE = 3;
    }
  }

  //STATE 3
  if (STATE == 3) {
    mv_forward();
    delay(200);
    STOP();
    delay(500);
    currcolour = colour;

    if (currcolour != prevcolour) {
      count2++;
    }

    prevcolour = colour;

    if (count2 == 3) {
      STOP();
      delay(1000);
      open_claw();
      delay(100000);
    }
  }
}

void pivot_left(int degrees) {
  int time = 1050 * (degrees/360);
  right_motor(FORWARD, 150);
  delay(time);
  STOP();
}

void pivot_right(int degrees) {
  int time = 1050 * (degrees/360);
  left_motor(FORWARD, 140);
  delay(time);
  STOP();
}

void rotate_left(int degrees) {
  int time = 500 * (degrees/40);
  left_motor(BACKWARD, 140);
  right_motor(FORWARD, 150);
  delay(time);
  STOP();
}

void rotate_right(int degrees) {
  int time = 500 * (degrees/40);
  left_motor(FORWARD, 140);
  right_motor(BACKWARD, 150);
  delay(time);
  STOP();
}

void mv_forward() {
  left_motor(FORWARD, 140);
  right_motor(FORWARD, 150);
  motorsRunning = true;
}

void mv_backward() {
  left_motor(BACKWARD, 140);
  right_motor(BACKWARD, 150);
  motorsRunning = true;
}

void forward_dist(int dist) {
  int time = 1000 * dist / ROB_SPEED; // duration in milliseconds
  mv_forward();
  delay(time);
  STOP();
}

void backward_dist(int dist) {
  int time = 1000 * dist / ROB_SPEED; // duration in milliseconds
  mv_backward();
  delay(time);
  STOP();
}

void STOP() {
  left_motor(OFF, 0);
  right_motor(OFF, 0);
  motorsRunning = false;
}

void open_claw() {
  if (claw_state) {
    return;
  }
  myServo.write(0);
  claw_state = OPEN;
}

void close_claw() {
  if (!claw_state) {
    return;
  }
  myServo.write(90);
  claw_state = CLOSED;
}

void left_motor(int direction, int speed) {
  if (speed >= 0 && speed <= 255){
   analogWrite(ENA, speed); 
  }
  else {
    analogWrite(ENA, 0);
    Serial.print("Error: Incorrect use of motor speed");
  }

  if (direction == FORWARD){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (direction == BACKWARD){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
}

void right_motor(int direction, int speed) {
  if (speed >= 0 && speed <= 255){
   analogWrite(ENB, speed); 
  }
  else {
    analogWrite(ENB, 0);
    Serial.print("Error: Incorrect use of motor speed");
  }
  
  if (direction == FORWARD){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction == BACKWARD){
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
}

void read_distance() {
  long duration;
  // Send trigger pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Measure echo duration
  duration = pulseIn(echo, HIGH);
  distance = duration / 58.2;

  //Serial.print("duration: "); Serial.print(duration); Serial.println(" ");
  Serial.print("distance: "); Serial.print(distance); Serial.println(" ");
  duration = pulseIn(echo, LOW);
}

void colour_sensor() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  red = pulseIn(OUT, LOW, 100000);
  delay(50);
    
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  green = pulseIn(OUT, LOW, 100000);
  delay(50);
    
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blue = pulseIn(OUT, LOW, 100000);
  delay(50);

  Serial.print("R: "); Serial.print(red, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(green, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(blue, DEC); Serial.println(" ");

  colour = getColour(red, green, blue);
  Serial.println(colour);
}

int getColour(int red, int green, int blue) {
  if (red >= 190 && red <= 230) { 
    rcounter++;
    if (rcounter == 3) {
      colour = RED;
      rcounter = 0;
    }
  } 
  else if (blue >= 310 && blue <= 340) {
    bcounter++;
    if (bcounter == 3) {
      colour = BLUE;
      bcounter = 0;
    }
  } 
  else if (green >= 410 && green <= 470 && blue >= 580) {
    gcounter++;
    if (gcounter == 3) {
      colour = GREEN;
      gcounter = 0;
    }
  } 
  else {
    colour = OTHER;
  }
  return colour;
}
