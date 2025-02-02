#include <Servo.h>

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

// Color sequence
int COUNTER = 0;
unsigned long t1 = 0;
unsigned long t0 = 0;

long double distance;

void setup() {
  // Setup USS
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  // Setup CS
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(OE_PIN, OUTPUT);

  // Set frequency scaling for TCS3200
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  digitalWrite(OE_PIN, LOW);

  // Setup servo
  myServo.attach(servo_pin);

  // Setup gearmotors
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Turn off both motors
  digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);

  // LED
  pinMode(A1, OUTPUT);

}

void loop() {

  right_motor(FORWARD, 150);
  left_motor(FORWARD, 140);  
  
  // distance measurement
  t1 = millis();

  // obstacle detection
  read_distance();
  if (distance < 15) {
    right_motor(OFF, 150);
    left_motor(OFF, 140);
    delay(1000);
    rotate_right(90); 
  }


  // colour detection
  colour_sensor();
  if (colour == 3333) {
    if (COUNTER == 0) {
      COUNTER++;
      flash_LED();
      t0 = 0;
    }
  } else if (colour == 4444) {
    if (COUNTER == 2) {
      COUNTER++;
      flash_LED();
      t0 = 0;
    } else if (COUNTER == 4) {
      COUNTER++;
      flash_LED();
      t0 = 0;
    }
  } else if (colour == 5555) {
    if (COUNTER == 1) {
      COUNTER++;
      flash_LED();
      t0 = 0;
    } else if (COUNTER == 3) {
      COUNTER++;
      flash_LED();
      t0 = 0;
    }
  } else if (COUNTER == 5) {
    STOP();
  } else if (colour == 1111) {
    if (t0 == 0) {
      t0 = millis();
    } else if ((t1-t0) > 1125) {
      right_motor(OFF, 150);
      left_motor(OFF, 140);
      delay(1000);
      right_motor(BACKWARD, 150);
      left_motor(BACKWARD, 140);
      delay(1125);
      right_motor(OFF, 150);
      left_motor(OFF, 140);
      delay(1000);

      rotate_right(90);
      t0 = 0;

    }
  }

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
  long double duration;
  // Send trigger pulse
  duration = pulseIn(echo, LOW);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // Measure echo duration
  duration = pulseIn(echo, HIGH);
  distance = duration / 58.2;

  return distance;
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
    if (red >= 250 && red <= 300) {
        rcounter++;
        if (rcounter == 3) {
            colour = 3333;
            rcounter = 0;
        }
    }

    if (blue >= 140 && blue <= 170) {
        bcounter++;
        if (bcounter == 3) {
            colour = 4444;
            bcounter = 0;
        }
    } else if (green >= 430 && green <= 465 && blue >= 500) {
        gcounter++;
        if (gcounter == 3) {
            colour = 5555;
            gcounter = 0;
        }
    } else {
        colour = 1111;
    }
    return colour;
}

void flash_LED() {
  digitalWrite(A1, HIGH);
  delay(1000);
  digitalWrite(A1, LOW);
  delay(1000);
}

void rotate_right(int degrees) {
  int time = 500 * (degrees/45);
  left_motor(FORWARD, 140);
  right_motor(BACKWARD, 150);
  delay(time);
}

void STOP() {
  left_motor(OFF, 0);
  right_motor(OFF, 0);
}
