#ifndef AStar_h
#define AStar_h

#include "Arduino.h"
#include <ArduinoQueue.h>
#include <LinkedList.h>

// Grid Limit (20, 64) -> (19,63)
#define GRID_DIV_SCALE 8
#define ROW 20
#define GRID_DIV 8

enum searchState { Searching, Failed, Completed };

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
    // General management functions
    uint8_t euclideanDistance(point* p1, point* p2);
    gridnode getLowestFScore();
    void unvisitedToVisited();
    // Bool functions
    bool isPresentOnUnvisited(point* nodePos);
    bool isPresentOnVisited(point* nodePos);
    bool isNeighborInsideGridMargins(gridnode* node, point* neighbor);
    // Neighbor management
    void getNodeNeighbors(gridnode* node, point (*pArray)[4]);
    searchState registerNodeNeighbors(point* start, point* finish, gridnode* node);
    searchState visitNeighbors(gridnode* nodePosition, point* start, point* goal);
    // Wrapper function
    // ArduinoQueue<point> searchForOptimalPath(point start, point goal);
    bool getNodeFather(point *fatherPos, gridnode *fatherNode);
    bool createOptimalPathQueue(point *start, point *goal);
    void searchForOptimalPath(point start, point goal);

    // Aux functions
    static int compare(gridnode *a, gridnode*b);
    void printGridOnSerial();
    LinkedList<gridnode> createUnvisitedElements();
  private:
    uint16_t grid [ROW][GRID_DIV];
    LinkedList<gridnode> unvisited = LinkedList<gridnode>();
    LinkedList<gridnode> visited = LinkedList<gridnode>();
    LinkedList<point> optimalPath = LinkedList<point>();
};

#endif
