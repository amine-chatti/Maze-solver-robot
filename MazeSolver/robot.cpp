#include "Arduino.h"
#include "robot.h"
#include "MazeSolver.h"

// ============================================================================
// CALIBRATION VARIABLES
// ============================================================================
int TICKS_PER_CELL = 100;
int BASE_SPEED = 80;
float KP_STRAIGHT = 3.0;
int TICKS_PER_90DEG_TURN = 100;  // Calibrate this!
int WALL_THRESHOLD_CM = 15;

static float KP_TURN = 1.4;
static float KD_TURN = 0.3;

// ============================================================================
// PRIVATE VARIABLES
// ============================================================================

// Encoder tick counts (volatile for ISR access)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Target ticks for movement
long leftTarget = 0;
long rightTarget = 0;

static void turnLeft90PID();
static void turnRight90PID();
static void moveOneCell();
static long readUltrasonicMedianCM(uint8_t trigPin, uint8_t echoPin);

// ============================================================================
// INITIALIZATION
// ============================================================================

void robotInit() {
    // Motor pins (2-pin control per motor)
    pinMode(LeftF, OUTPUT);
    pinMode(LeftR, OUTPUT);
    pinMode(RightF, OUTPUT);
    pinMode(RightR, OUTPUT);

    // Encoder pins
    pinMode(ENC_LEFT_A, INPUT);
    pinMode(ENC_LEFT_B, INPUT);
    pinMode(ENC_RIGHT_A, INPUT);
    pinMode(ENC_RIGHT_B, INPUT);

    // Encoder interrupts
    attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);

    // Ultrasonic pins
    pinMode(US_FRONT_TRIG, OUTPUT);
    pinMode(US_FRONT_ECHO, INPUT);

    pinMode(US_LEFT_TRIG, OUTPUT);
    pinMode(US_LEFT_ECHO, INPUT);

    pinMode(US_RIGHT_TRIG, OUTPUT);
    pinMode(US_RIGHT_ECHO, INPUT);



    // Ensure motors are stopped
    motorStop();
}

void resetEncoders() {
    noInterrupts();
    leftTicks = 0;
    rightTicks = 0;
    interrupts();
}


// ============================================================================
// MOTOR CONTROL (2-PIN H-BRIDGE)
// ============================================================================

void motorLeft(int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed > 0) {
        // Forward
        analogWrite(LeftF, speed);
        analogWrite(LeftR, 0);
    } 
    else if (speed < 0) {
        // Reverse
        analogWrite(LeftF, 0);
        analogWrite(LeftR, -speed);
    } 
    else {
        // Stop (coast)
        analogWrite(LeftF, 0);
        analogWrite(LeftR, 0);
    }
}

void motorRight(int speed) {
    speed = constrain(speed, -MAX_SPEED, MAX_SPEED);

    if (speed > 0) {
        // Forward
        analogWrite(RightF, speed);
        analogWrite(RightR, 0);
    } 
    else if (speed < 0) {
        // Reverse
        analogWrite(RightF, 0);
        analogWrite(RightR, -speed);
    } 
    else {
        // Stop (coast)
        analogWrite(RightF, 0);
        analogWrite(RightR, 0);
    }
}

void motorStop() {
    analogWrite(LeftF, 0);
    analogWrite(LeftR, 0);
    analogWrite(RightF, 0);
    analogWrite(RightR, 0);
}


void driveStraight(int baseSpeed) {
    long error = getLeftTicks() - getRightTicks();

    int correction = error * KP_STRAIGHT;  // gain (tune this)

    int leftSpeed  = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;

    motorLeft(leftSpeed);
    motorRight(rightSpeed);
}

// ============================================================================
// ENCODER INTERRUPTS
// ============================================================================

void leftEncoderISR() {
    if (digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) {
        leftTicks--;
    } else {
        leftTicks++;
    }
}

void rightEncoderISR() {
    if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) {
        rightTicks++;
    } else {
        rightTicks--;
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

long getLeftTicks() {
    noInterrupts();
    long ticks = leftTicks;
    interrupts();
    return ticks;
}

long getRightTicks() {
    noInterrupts();
    long ticks = rightTicks;
    interrupts();
    return ticks;
}

void printEncoderStatus() {
    int error = getLeftTicks()-getRightTicks();
    Serial.print(getLeftTicks());
    Serial.print(" ");
    Serial.print(getRightTicks());
    Serial.print(" ");
    Serial.println(error);
}

// ============================================================================
// HIGH-LEVEL MOVEMENT (MAZE COMMANDS)
// ============================================================================

void moveToDirection(int nextDir) {
    int diff = (nextDir - dir + 4) % 4;

    if (diff == 1) {
        turnRight90PID();
    }
    else if (diff == 3) {
        turnLeft90PID();
    }
    else if (diff == 2) {
        turnRight90PID();
        turnRight90PID();
    }

    dir = nextDir;
    moveOneCell();
}

static void moveOneCell() {
    resetEncoders();

    while ((getLeftTicks() + getRightTicks()) / 2 < TICKS_PER_CELL) {
        driveStraight(BASE_SPEED);
    }

    motorStop();

    if (dir == NORTH) x--;
    else if (dir == EAST) y++;
    else if (dir == SOUTH) x++;
    else if (dir == WEST) y--;
}

static void turnLeft90PID() {
    resetEncoders();
    int lastError = 0;

    while (true) {
        int currentDiff = getRightTicks() - getLeftTicks();
        int error = TICKS_PER_90DEG_TURN - currentDiff;

        if (error <= 0) break;

        int derivative = error - lastError;
        float pidOutput = (KP_TURN * error) + (KD_TURN * derivative);
        lastError = error;

        int speed = constrain((int)pidOutput, 60, 100);

        motorLeft(-speed);
        motorRight(speed);

        delay(5);
    }

    motorStop();
}

static void turnRight90PID() {
    resetEncoders();
    int lastError = 0;

    while (true) {
        int currentDiff = getLeftTicks() - getRightTicks();
        int error = TICKS_PER_90DEG_TURN - currentDiff;

        if (error <= 0) break;

        int derivative = error - lastError;
        float pidOutput = (KP_TURN * error) + (KD_TURN * derivative);
        lastError = error;

        int speed = constrain((int)pidOutput, 60, 100);

        motorLeft(speed);
        motorRight(-speed);

        delay(5);

    }

    motorStop();
}

// ============================================================================
// ULTRASONIC HELPERS
// ============================================================================

long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 30 ms timeout prevents blocking loop when no echo is received
    unsigned long duration = pulseIn(echoPin, HIGH, 30000);
    if (duration == 0) return 999;

    return (long)(duration * 0.0343f / 2.0f);
}

static long readUltrasonicMedianCM(uint8_t trigPin, uint8_t echoPin) {
    long a = readUltrasonicCM(trigPin, echoPin);
    delay(2);
    long b = readUltrasonicCM(trigPin, echoPin);
    delay(2);
    long c = readUltrasonicCM(trigPin, echoPin);

    if (a > b) { long t = a; a = b; b = t; }
    if (b > c) { long t = b; b = c; c = t; }
    if (a > b) { long t = a; a = b; b = t; }

    return b;
}

bool wallFront() {
    return readUltrasonicMedianCM(US_FRONT_TRIG, US_FRONT_ECHO) <= WALL_THRESHOLD_CM;
}

bool wallLeft() {
    return readUltrasonicMedianCM(US_LEFT_TRIG, US_LEFT_ECHO) <= WALL_THRESHOLD_CM;
}

bool wallRight() {
    return readUltrasonicMedianCM(US_RIGHT_TRIG, US_RIGHT_ECHO) <= WALL_THRESHOLD_CM;
}

// ============================================================================
// TURN FUNCTIONS
// ============================================================================

void turnLeft(int speed) {
    // Left motor backward, right motor forward = turn left
    motorLeft(-speed);
    motorRight(speed);
}

void turnRight(int speed) {
    // Left motor forward, right motor backward = turn right
    motorLeft(speed);
    motorRight(-speed);
}