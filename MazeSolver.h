#ifndef MAZE_H
#define MAZE_H

#define N 16

// Directions
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

extern int dist[N][N];
extern bool walls[N][N][4];

extern int x, y;
extern int dir;

// Core functions
void initMaze();
void floodFill(int goalX, int goalY);
void updateWalls();
int chooseNextDirection();

#endif