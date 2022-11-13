#include "HardwareSerial.h"
#include <ArduinoQueue.h>
#include <LinkedList.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "Arduino.h"
#include "a_star.h"

// AStarGrid::AStarGrid(uint8_t size_x, uint8_t size_y)
// {
// }

// unsigned char AStarGrid::calculateFScore(point* p1, point* p2)
// {
//   int a = pow((p1->pos_x) - (p2->pos_x), 2);
//   int b = pow((p1->pos_y) - (p2->pos_y), 2);

//   unsigned char res = round(sqrt(a + b));

//   return res;
// }

// GRID SECTION --------------------------------------------
// ---------------------------------------------------------

// Each bit represents a cell
// Number of possible cells => ROW * GRID_DIV * GRID_DIV_SCALE
// Memory usage: ROW * GRID_DIV * 2B (16 bits)
uint16_t grid [ROW][GRID_DIV];

void initGrid()
{
  for (uint8_t i=0; i<ROW; i++)
  {
    for (uint8_t j=0; j<GRID_DIV; j++)
    {
      grid[i][j]=0;
    }
  }
}

bool isObstacleCell(uint8_t i, uint8_t j)
{
  uint8_t inGridDiv = floor(j / GRID_DIV_SCALE);
  uint8_t posInDiv = j % GRID_DIV_SCALE;
  return grid[i][inGridDiv] & (1<<(posInDiv));
}

void addObstacleCell(uint8_t i, uint8_t j)
{
  uint8_t inGridDiv = floor(j / GRID_DIV_SCALE);
  uint8_t posInDiv = j % GRID_DIV_SCALE;
  grid[i][inGridDiv] |= (1<<posInDiv);
}

void printGridOnSerial()
{
  for (int8_t i=0; i<ROW; i++)
  {
    for (int8_t j=(GRID_DIV-1); j>=0; j--){
      Serial.print(j);
      Serial.print(":");
      Serial.print(grid[i][j], BIN);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// LinkedList<gridnode> *unvisited = LinkedList<gridnode>();

// No introducir nodos (posiciones registradas en UNVISITED y VISITED) repetidas a UNVISITED
point* getNodeNeighbors(gridnode* node)
{
  point neighborPositions[4];
  neighborPositions[0] = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y + 1};
  neighborPositions[1] = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y - 1};
  neighborPositions[2] = {.pos_x = node->pos.pos_x + 1, .pos_y = node->pos.pos_y};
  neighborPositions[3] = {.pos_x = node->pos.pos_x - 1, .pos_y = node->pos.pos_y};

  return neighborPositions;
}

// Una funcion para iterar sobre una lista