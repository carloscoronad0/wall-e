#ifndef AStar_h
#define AStar_h

#include "Arduino.h"
#include <ArduinoQueue.h>
#include <LinkedList.h>

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

class AStar
{
  public:
    AStar();
    void initGrid();
    bool isObstacleCell(uint8_t i, uint8_t j);
    void addObstacleCell(uint8_t i, uint8_t j);
    void printGridOnSerial();
    LinkedList<gridnode> createUnvisitedElements();
    gridnode getLowestFScore();
    LinkedList<gridnode> unvisitedToVisited();
    void getNodeNeighbors(gridnode* node, point (*pArray)[4]);
    static int compare(gridnode *a, gridnode*b);
    uint8_t euclideanDistance(point* p1, point* p2);
    void registerNodeNeighbors(point* start, point* finish, gridnode* node);
    bool isPresentOnUnvisited(point* nodePos);
    bool isNeighborInsideGridMargins(gridnode* node, point* neighbor);
  private:
    uint16_t grid [ROW][GRID_DIV];
    LinkedList<gridnode> unvisited = LinkedList<gridnode>();
    LinkedList<gridnode> visited = LinkedList<gridnode>();
};

#endif