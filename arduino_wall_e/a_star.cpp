#include <ArduinoQueue.h>
#include <LinkedList.h>
#include "Arduino.h"
#include "a_star.h"

AStarGrid::AStarGrid(char* size_x, char* size_y)
{
  _size_x = size_x;
  _size_y = size_x;
}

unsigned char AStarGrid::calculateFScore(point* p1, point* p2)
{
  int a = pow((p1->pos_x) - (p2->pos_x), 2);
  int b = pow((p1->pos_y) - (p2->pos_y), 2);

  unsigned char res = round(sqrt(a + b));

  return res;
}