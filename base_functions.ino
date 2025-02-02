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

Servo myServo;

int claw_state; 
#define OPEN 1
#define CLOSED 0

// Motor statese
#define OFF 0
#define FORWARD 1
#define BACKWARD 2

long distance;

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

}

void loop() {
  right_motor(FORWARD, 150);
  left_motor(FORWARD, 150);
  open_claw();
  delay(1000);
  close_claw();
  delay(10000);
  // right_motor(BACKWARD, 150);
  // left_motor(BACKWARD, 125);
  // delay(1000);
  right_motor(OFF, 100);
  left_motor(OFF, 100);
  delay(1000);

  // close_claw();
  // delay(1000);
  // open_claw();
  // delay(1000);
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
