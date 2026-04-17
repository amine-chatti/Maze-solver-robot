
#include "maze.h"
#include "robot.h"

// =========================
// GOAL (EXEMPLE HATHA TAW NBADLOUH HASB L MAZE)
// =========================
int GOAL_X = 7;
int GOAL_Y = 7;

// =========================
// TURN PID 
// =========================
int TICKS_FOR_90DEG = 160;

float KP_TURN = 0.5;
float KD_TURN = 0.3;
float KI_TURN = 2.0;

// =========================
// FUNCTION DECLARATIONS
// =========================
void turnLeft90PID();
void turnRight90PID();
void moveOneCell();
void moveToDirection(int nextDir);

// =========================
// SETUP
// =========================
void setup() {
    Serial.begin(115200);

    robotInit();
    initMaze();

    x = 0;
    y = 0;
    dir = NORTH;
}

// =========================
// MAIN LOOP 
// =========================
void loop() {

    // 1. UPDATE MAP FROM SENSORS
    updateWalls();

    // 2. RUN FLOOD FILL
    floodFill(GOAL_X, GOAL_Y);

    // 3. CHOOSE BEST DIRECTION
    int nextDir = chooseNextDirection();

    // 4. MOVE ROBOT
    moveToDirection(nextDir);
}

// =========================
// MOVE LOGIC (CORE)
// =========================
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

// =========================
// MOVE 1 CELL (ENCODERS)
// =========================
void moveOneCell() {

    resetEncoders();

    while ((getLeftTicks() + getRightTicks()) / 2 < TICKS_PER_CELL) {
        driveStraight(BASE_SPEED);
    }

    motorStop();

    // update position
    if (dir == NORTH) x--;
    else if (dir == EAST) y++;
    else if (dir == SOUTH) x++;
    else if (dir == WEST) y--;
}

// =========================
// TURN LEFT 90 (PID)
// =========================
void turnLeft90PID() {

    resetEncoders();
    int lastError = 0;

    while (true) {

        int currentDiff = getRightTicks() - getLeftTicks();
        int error = TICKS_FOR_90DEG - currentDiff;

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

// =========================
// TURN RIGHT 90 (PID)
// =========================
void turnRight90PID() {

    resetEncoders();
    int lastError = 0;

    while (true) {

        int currentDiff = getLeftTicks() - getRightTicks();
        int error = TICKS_FOR_90DEG - currentDiff;

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
