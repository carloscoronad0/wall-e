#ifndef AStar_h
#define AStar_h

#include "Arduino.h"
#include <ArduinoQueue.h>
#include <LinkedList.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#define GRID_DIV_SCALE 16
#define ROW 10
#define COL 32
#define GRID_DIV 4

/* 2 bytes */
struct point {
  uint8_t pos_x;
  uint8_t pos_y;
};

/* 3 bytes */
struct gridnode {
  point pos;
  uint8_t f_score;
  point father;
};

// class AStarGrid
// {
//   public:
//     AStarGrid();
//     // void registerObstacleOnGrid(point* p);
//     // bool isObstacleOnGrid(point* obstacle);
//     unsigned char calculateFScore(point* p1, point* p2);
//   private:
//     uint16_t grid [ROW][GRID_DIV];
//     //unsigned char limitLightObs;    // 60 obs -> 120 bytes
//     //LinkedList<point> lightObs;
//     //LinkedList<gridnode> unvisitedNeighbors; // En promedio el total de los GridNode es de 300 -> 900 bytes
//     //ArduinoQueue<point> visitedNeighbors; // 80 points -> 160 bytes
// };

void initGrid();
bool isObstacleCell(uint8_t i, uint8_t j);
void addObstacleCell(uint8_t i, uint8_t j);
void printGridOnSerial();
LinkedList<gridnode> createUnvisitedElements();
gridnode getLowestFScore();
LinkedList<gridnode> unvisitedToVisited();
int compare(gridnode *a, gridnode*b);

#endif