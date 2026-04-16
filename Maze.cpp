#include "maze.h"
#include "robot.h"

#include <queue>

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
            for (int k = 0; k < 4; k++)
                walls[i][j][k] = false;
        }
    }
}

// ----------------------------
// FLOOD FILL BFS
// ----------------------------
void floodFill(int gx, int gy) {
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dist[i][j] = 9999;

    std::queue<int> qx, qy;

    dist[gx][gy] = 0;
    qx.push(gx);
    qy.push(gy);

    int dx[4] = {-1, 0, 1, 0};
    int dy[4] = {0, 1, 0, -1};

    while (!qx.empty()) {
        int cx = qx.front(); qx.pop();
        int cy = qy.front(); qy.pop();

        for (int d = 0; d < 4; d++) {

            // wall check
            if (walls[cx][cy][d]) continue;

            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;

            if (dist[nx][ny] > dist[cx][cy] + 1) {
                dist[nx][ny] = dist[cx][cy] + 1;
                qx.push(nx);
                qy.push(ny);
            }
        }
    }
}