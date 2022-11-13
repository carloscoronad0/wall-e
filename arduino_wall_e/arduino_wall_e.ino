#include "a_star.h"

// uint8_t random_i;
// uint8_t random_j;
// LinkedList<gridnode> u;
// LinkedList<gridnode> v;
// gridnode gn;
AStar a_star;

void setup() {
  // a_star.initGrid();
  Serial.begin(9600);
  Serial.println("------------------------------------------");

  // u = a_star.createUnvisitedElements();
  // Serial.println(u.get(0).f_score);
  // gn = a_star.getLowestFScore();
  // Serial.println(gn.f_score);
  // v = a_star.unvisitedToVisited();
  // Serial.println(v.get(0).f_score);
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