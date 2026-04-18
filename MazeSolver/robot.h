#ifndef ROBOT_H 
#define ROBOT_H
#include <Arduino.h>

// PINS
#define LeftF  6
#define LeftR  5
#define RightF 9
#define RightR 10

// ENCODERS
#define ENC_LEFT_A  2
#define ENC_LEFT_B  A4
#define ENC_RIGHT_A 3
#define ENC_RIGHT_B A3

// ULTRASONIC (HC-SR04)
#define US_FRONT_TRIG 7
#define US_FRONT_ECHO 8

#define US_LEFT_TRIG  A6
#define US_LEFT_ECHO  A5

#define US_RIGHT_TRIG A1
#define US_RIGHT_ECHO A2

#define MAX_SPEED 255
#define MIN_SPEED 50

// CALIBRATION
extern int TICKS_PER_CELL;
extern int BASE_SPEED;
extern float KP_STRAIGHT;
extern int TICKS_PER_90DEG_TURN;  // Ticks for 90 degree turn
extern int WALL_THRESHOLD_CM;

// FUNCTIONS
void robotInit();
void resetEncoders();

void motorLeft(int speed);
void motorRight(int speed);
void motorStop();

void driveStraight(int baseSpeed);
void turnLeft(int speed);
void turnRight(int speed);
void moveToDirection(int nextDir);

void leftEncoderISR();
void rightEncoderISR();

long getLeftTicks();
long getRightTicks();
void printEncoderStatus();

long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin);
bool wallFront();
bool wallLeft();
bool wallRight();
#endif // ROBOT_H