
#include "MazeSolver.h"
#include "robot.h"

bool done = false;

int chooseReactiveDirection(bool frontBlocked, bool leftBlocked, bool rightBlocked) {
    if (!frontBlocked) return dir;
    if (!rightBlocked) return (dir + 1) % 4;
    if (!leftBlocked) return (dir + 3) % 4;
    return (dir + 2) % 4;
}

void runSensorNavigationTest() {
    const int STEPS = 4 ;

    for (int step = 1; step <= STEPS; step++) {

        bool frontBlocked = wallFront();
        bool leftBlocked = wallLeft();
        bool rightBlocked = wallRight();

        int nextDir = chooseReactiveDirection(frontBlocked, leftBlocked, rightBlocked);

        moveToDirection(nextDir);
    }

    motorStop();
}

// =========================
// SETUP
// =========================
void setup() {
    Serial.begin(115200);

    robotInit();

    x = 0;
    y = 0;
    dir = NORTH;
}

// =========================
// MAIN LOOP 
// =========================
void loop() {

    Serial.print(wallFront());
    Serial.print(" ");
    Serial.print(wallLeft());
    Serial.print(" ");
    Serial.print(wallRight());
    Serial.println(" ");

}
