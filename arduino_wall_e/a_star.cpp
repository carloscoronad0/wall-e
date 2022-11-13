#include "a_star.h"

AStar::AStar(){}

void AStar::initGrid(){
  for (uint8_t i=0; i<ROW; i++)
  {
    for (uint8_t j=0; j<GRID_DIV; j++)
    {
      grid[i][j]=0;
    }
  }  
}

bool AStar::isObstacleCell(uint8_t i, uint8_t j){
  uint8_t inGridDiv = floor(j / GRID_DIV_SCALE);
  uint8_t posInDiv = j % GRID_DIV_SCALE;
  return grid[i][inGridDiv] & (1<<(posInDiv));
}

void AStar::addObstacleCell(uint8_t i, uint8_t j){
  uint8_t inGridDiv = floor(j / GRID_DIV_SCALE);
  uint8_t posInDiv = j % GRID_DIV_SCALE;
  grid[i][inGridDiv] |= (1<<posInDiv);
}

void AStar::printGridOnSerial(){
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

// No introducir nodos (posiciones registradas en UNVISITED y VISITED) repetidas a UNVISITED
point* AStar::getNodeNeighbors(gridnode *node){
  point neighborPositions[4];
  neighborPositions[0] = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y + 1};
  neighborPositions[1] = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y - 1};
  neighborPositions[2] = {.pos_x = node->pos.pos_x + 1, .pos_y = node->pos.pos_y};
  neighborPositions[3] = {.pos_x = node->pos.pos_x - 1, .pos_y = node->pos.pos_y};

  return neighborPositions;
}

//Create elements for unvisited list
LinkedList<gridnode> AStar::createUnvisitedElements(){
  point p1;
  p1.pos_x = 2;
  p1.pos_y = 57;

  point p2;
  p2.pos_x = 10;
  p2.pos_y = 20;

  point p3;
  p3.pos_x = 4;
  p3.pos_y = 5;

  point p4;
  p4.pos_x = 40;
  p4.pos_y = 50;
  
  gridnode node01;
  node01.f_score = 10;
  node01.pos = p1;
  node01.father = p2;

  gridnode node02;
  node02.f_score = 23;
  node02.pos = p3;
  node02.father = p4;

  gridnode node03;
  node03.f_score = 5;
  node03.pos = p3;
  node03.father = p4;

  unvisited.add(node01);
  unvisited.add(node02);
  unvisited.add(node03);

  return unvisited;
}

gridnode AStar::getLowestFScore(){
  unvisited.sort(compare);
  return unvisited.get(0);  
}

LinkedList<gridnode> AStar::unvisitedToVisited(){
  // Always return the first element of unvisited node
  gridnode node = unvisited.shift();
  visited.add(node);
  return visited;  
}

static int AStar::compare(gridnode *a, gridnode *b){
  if (a->f_score >= b->f_score){
    return 1;
  }else{
    return 0;
  }
}