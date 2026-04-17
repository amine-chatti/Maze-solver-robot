#include "maze.h"
#include "robot.h"

// ----------------------------
// GLOBAL STATE
// ----------------------------
int dist[N][N];
bool walls[N][N][4];

int x = 0, y = 0;
int dir = NORTH;

// ----------------------------
// INIT
// ----------------------------
void initMaze() {
    for (int i = 0; i < N; i++) {
        for (int j = 0; j < N; j++) {
            dist[i][j] = 9999;
            for (int k = 0; k < 4; k++) {
                walls[i][j][k] = false;
            }
        }
    }
}

// ----------------------------
// FLOOD FILL (BFS)
// ----------------------------
void floodFill(int gx, int gy) {

    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dist[i][j] = 9999;

    int qx[N*N], qy[N*N];
    int front = 0, back = 0;

    dist[gx][gy] = 0;
    qx[back] = gx;
    qy[back] = gy;
    back++;

    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, 1, 0, -1};

    while (front < back) {

        int cx = qx[front];
        int cy = qy[front];
        front++;

        for (int d = 0; d < 4; d++) {

            if (walls[cx][cy][d]) continue;

            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;

            if (dist[nx][ny] > dist[cx][cy] + 1) {
                dist[nx][ny] = dist[cx][cy] + 1;
                qx[back] = nx;
                qy[back] = ny;
                back++;
            }
        }
    }
}

// ----------------------------
// SENSOR WALL UPDATE
// (YOU MUST CONNECT ULTRASONIC HERE)
// ----------------------------
void updateWalls() {

    // FRONT WALL
    if (wallFront()) {
        walls[x][y][dir] = true;
    }

    // LEFT WALL
    int leftDir = (dir + 3) % 4;
    if (wallLeft()) {
        walls[x][y][leftDir] = true;
    }

    // RIGHT WALL
    int rightDir = (dir + 1) % 4;
    if (wallRight()) {
        walls[x][y][rightDir] = true;
    }
}

// ----------------------------
// CHOOSE BEST MOVE
// ----------------------------
int chooseNextDirection() {

    int bestDir = -1;
    int bestDist = 9999;

    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, 1, 0, -1};

    for (int d = 0; d < 4; d++) {

        if (walls[x][y][d]) continue;

        int nx = x + dx[d];
        int ny = y + dy[d];

        if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;

        if (dist[nx][ny] < bestDist) {
            bestDist = dist[nx][ny];
            bestDir = d;
        }
    }

    return bestDir;
}