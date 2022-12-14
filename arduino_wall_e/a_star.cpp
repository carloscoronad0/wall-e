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

// Aux functions ------------------------------------------------------

static int AStar::compare(gridnode *a, gridnode *b){
  if (a->f_score >= b->f_score){
    return 1;
  }else{
    return 0;
  }
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

// General management functions ---------------------------------------

uint8_t AStar::euclideanDistance(point *p1, point *p2)
{
  uint8_t a = pow((p1->pos_x) - (p2->pos_x), 2);
  uint8_t b = pow((p1->pos_y) - (p2->pos_y), 2);

  uint8_t res = round(sqrt(a + b));

  return res;
}

void AStar::unvisitedToVisited(){
  // Always return the first element of unvisited node
  gridnode node = this->unvisited.shift();
  this->visited.add(node);
}

gridnode AStar::getLowestFScore(){
  this->unvisited.sort(compare);
  return this->unvisited.get(0);
}

// Bool functions -----------------------------------------------------

bool AStar::isPresentOnUnvisited(point* nodePos)
{
  for (uint16_t i=0; i<this->unvisited.size(); i++)
  {
    gridnode auxNode = unvisited.get(i);
    if ((auxNode.pos.pos_x == nodePos->pos_x) && (auxNode.pos.pos_y == nodePos->pos_y))
    {
      return true;
    }
  }

  return false;
}

bool AStar::isPresentOnVisited(point* nodePos)
{
  gridnode auxNode;
  for (uint16_t i=0; i<this->visited.size(); i++)
  {
    auxNode = visited.get(i);
    if ((auxNode.pos.pos_x == nodePos->pos_x) && (auxNode.pos.pos_y == nodePos->pos_y))
    {
      return true;
    }
  }

  return false;
}

bool AStar::isNeighborInsideGridMargins(gridnode* node, point* neighbor)
{
  uint8_t x_difference = abs(node->pos.pos_x - neighbor->pos_x);
  uint8_t y_difference = abs(node->pos.pos_y - neighbor->pos_y);

  return (x_difference == 1 && y_difference == 0) || (x_difference == 0 && y_difference == 1);
}

// Neighbor management ------------------------------------------------

void AStar::getNodeNeighbors(gridnode *node, point (*pArray)[4]){
  point p0 = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y + 1};
  point p1 = {.pos_x = node->pos.pos_x, .pos_y = node->pos.pos_y - 1};
  point p2 = {.pos_x = node->pos.pos_x + 1, .pos_y = node->pos.pos_y};
  point p3 = {.pos_x = node->pos.pos_x - 1, .pos_y = node->pos.pos_y};

  (*pArray)[0] = p0;
  (*pArray)[1] = p1;
  (*pArray)[2] = p2;
  (*pArray)[3] = p3;
}

searchState AStar::registerNodeNeighbors(point* start, point* finish, gridnode* node)
{
  searchState reachedGoal;
  point neighborPositions[4] = {};
  this->getNodeNeighbors(node, &neighborPositions);

  for (uint8_t i=0; i<4; i++)
  {
    if (this->isNeighborInsideGridMargins(node, &neighborPositions[i]))
    {
      if (!this->isObstacleCell(neighborPositions[i].pos_x, neighborPositions[i].pos_y)
        && !this->isPresentOnUnvisited(&neighborPositions[i])
        && !this->isPresentOnVisited(&neighborPositions[i]))
      {
        uint8_t g_score = this->euclideanDistance(start, &neighborPositions[i]);
        uint8_t h_score = this->euclideanDistance(&neighborPositions[i], finish);

        gridnode neighbor;
        neighbor.pos = neighborPositions[i];
        neighbor.father = node->pos;
        neighbor.f_score = g_score + h_score;

        this->unvisited.add(neighbor);
        if (finish->pos_x == neighbor.pos.pos_x && finish->pos_y == neighbor.pos.pos_y){
          reachedGoal = Completed;
          return reachedGoal;
        }
      }
    }
  }
  reachedGoal = Searching;
  return reachedGoal;
}

searchState AStar::visitNeighbors(gridnode *nodePosition, point *start, point *goal){
  searchState state = this->registerNodeNeighbors(start, goal, nodePosition);
  this->unvisitedToVisited();
  if(state == Searching && this->unvisited.size() == 0){
    state = Failed;
  }
  return state;
}

// Wrapper function ---------------------------------------------------

bool AStar::getNodeFather(point *fatherPos, gridnode *fatherNode){
    gridnode aux;
    for (uint16_t i=0; i<this->visited.size(); i++){
        aux = this->visited.get(i);
        if (aux.pos.pos_x == fatherPos->pos_x && aux.pos.pos_y == fatherPos->pos_y){
            fatherNode = &aux;
            return true;
        }
    }
    return false;
}

bool AStar::createOptimalPathQueue(point *start, point *goal){
    bool success;
    point inPathPoint;
    point fatherPoint;
    gridnode inPathNode;

    // The last inserted element is the goalnode
    inPathNode = this->unvisited.pop();
    inPathPoint = inPathNode.pos;
    this->optimalPath.add(inPathPoint);

    while (inPathPoint.pos_x != start->pos_x && inPathPoint.pos_y != start->pos_y){
        fatherPoint = inPathNode.father;
        success = this->getNodeFather(&fatherPoint, &inPathNode);
        if (!success){
            return false;
        }

        inPathPoint = inPathNode.pos;
        this->optimalPath.add(inPathPoint);
    }

    return true;
}

void AStar::searchForOptimalPath(point start, point goal){
  //Insert start node to unvisited
  gridnode startNode;
  startNode.f_score = this->euclideanDistance(&start, &goal);
  startNode.pos = start;
  // startNode.father = NULL;
  this->unvisited.add(startNode);

  gridnode step;

  //Visit the neighbors
  searchState state = this->visitNeighbors(&startNode, &start, &goal);
  while (state == Searching) {
    step = this->getLowestFScore();
    state = this->visitNeighbors(&step, &start, &goal);
  }

  if (state == Completed) {
    // Path to goal completed
    bool success = this->createOptimalPathQueue(&start, &goal);
  }
}
