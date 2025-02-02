// Pin definitions for TCS3200
#define S0 0
#define S1 1
#define S2 10
#define S3 11
#define OUT 12
#define OE_PIN 13

// Ultrasonic Sensor connections
#define echo A0
#define trig 2

// Left Motor connections
#define ENA 9
#define IN1 8
#define IN2 7

// Right Motor connections
#define IN3 6
#define IN4 5
#define ENB 3 

#include <Servo.h>

int red = 0;
int green = 0;
int blue = 0;
int colour = 1111;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
int rcounter = 0;
int gcounter = 0;
int bcounter = 0;
long distance;
bool motorsRunning = false;
#define OFF 0
#define FORWARD 1
#define BACKWARD 2

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

void loop() {
    colourSensorTask();
    read_distance();

    mv_forward();
if (distance<30){
  STOP();
    if (colour == 4444) {
        turn_left(90);
    } else if (colour == 5555) {
        turn_right(90);
    }
    else if (colour == 3333) {
      turn_around();
      // turn_left(180);
    }
  }
}

void colourSensorTask() {
    digitalWrite(S2, LOW);
    digitalWrite(S3, LOW);
    red = pulseIn(OUT, LOW, 100000);
    delay(100);
    
    
    digitalWrite(S2, HIGH);
    digitalWrite(S3, HIGH);
    green = pulseIn(OUT, LOW, 100000);
    delay(100);
    
    digitalWrite(S2, LOW);
    digitalWrite(S3, HIGH);
    blue = pulseIn(OUT, LOW, 100000);
    delay(100);

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

void read_distance() {
    long duration;
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    duration = pulseIn(echo, HIGH);
    distance = duration / 58.2;

    Serial.print("distance: "); Serial.print(distance); Serial.println(" ");
}

void turn_left(int degrees) {
  int time = 500 * (degrees/50);
    left_motor(BACKWARD, 140);
    right_motor(FORWARD, 150);
    delay(time);
}

void turn_right(int degrees) {
  int time = 500 * (degrees/50);
    left_motor(FORWARD, 140);
    right_motor(BACKWARD, 150);
    delay(time);
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

void left_motor(int direction, int speed) {
    if (speed >= 0 && speed <= 255){
        analogWrite(ENA, speed); 
    } else {
        analogWrite(ENA, 0);
        Serial.print("Error: Incorrect use of motor speed");
    }

    if (direction == FORWARD){
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    } else if (direction == BACKWARD){
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
    }
}

void right_motor(int direction, int speed) {
    if (speed >= 0 && speed <= 255){
        analogWrite(ENB, speed); 
    } else {
        analogWrite(ENB, 0);
        Serial.print("Error: Incorrect use of motor speed");
    }
    
    if (direction == FORWARD){
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    } else if (direction == BACKWARD){
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
    }
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

void STOP() {
  left_motor(OFF, 0);
  right_motor(OFF, 0);
  motorsRunning = false;
}

void turn_around() {
  left_motor(FORWARD, 140);
  right_motor(BACKWARD, 150);
  delay(500);
  STOP();
  delay(2000);
}
 

