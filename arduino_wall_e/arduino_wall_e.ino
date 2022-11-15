#include "a_star.h"

// uint8_t random_i;
// uint8_t random_j;
// LinkedList<gridnode> u;
// LinkedList<gridnode> v;
// gridnode gn;
AStar a_star;
point start = {.pos_x=4, .pos_y=0};
point finish = {.pos_x=8, .pos_y=30};
gridnode testNode;

void setup() {
  // a_star.initGrid();
  testNode.f_score = 40;
  testNode.pos = {.pos_x=0, .pos_y=10};
  testNode.father = {.pos_x=7, .pos_y=9};

  Serial.begin(9600);
  Serial.println("------------------------------------------");

  // u = a_star.createUnvisitedElements();
  // Serial.println(u.get(0).f_score);
  // gn = a_star.getLowestFScore();
  // Serial.println(gn.f_score);
  // v = a_star.unvisitedToVisited();
  // Serial.println(v.get(0).f_score);

  a_star.registerNodeNeighbors(&start, &finish, &testNode);
  a_star.registerNodeNeighbors(&start, &finish, &testNode);
}

void loop()
{
  // random_i = random(0, ROW);
  // random_j = random(0, COL);

  // Serial.print("Setting random on: ");
  // Serial.print(random_i, DEC);
  // Serial.print(" , ");
  // Serial.print(random_j, DEC);
  // Serial.println();
  
  // a_star.addObstacleCell(random_i, random_j);
  // Serial.print("Is Obstacle: ");
  // Serial.println(a_star.isObstacleCell(random_i, random_j));
  // a_star.printGridOnSerial();

  // delay(1000); 
}