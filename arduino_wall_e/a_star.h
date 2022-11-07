#ifndef AStar_h
#define AStar_h

#include "Arduino.h"
#include <ArduinoQueue.h>
#include <LinkedList.h>

/* 2 bytes */
struct point {
  unsigned char pos_x;
  unsigned char pos_y;
};

/* 3 bytes */
struct gridnode {
  point pos;
  unsigned char f_score;
};

class AStarGrid
{
  public:
    AStarGrid(char* size_x, char* size_y);
    // void registerObstacleOnGrid(point* p);
    // bool isObstacleOnGrid(point* obstacle);
    unsigned char calculateFScore(point* p1, point* p2);
  private:
    unsigned char _size_x;
    unsigned char _size_y;
    //unsigned char limitLightObs;    // 60 obs -> 120 bytes
    //LinkedList<point> lightObs;
    //LinkedList<gridnode> unvisitedNeighbors; // En promedio el total de los GridNode es de 300 -> 900 bytes
    //ArduinoQueue<point> visitedNeighbors; // 80 points -> 160 bytes
};

#endif