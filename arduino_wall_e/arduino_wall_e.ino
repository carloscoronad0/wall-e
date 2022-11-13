#include "a_star.h"

// AStarGrid test(x, y);
uint8_t random_i;
uint8_t random_j;

void setup() {
  initGrid();
  Serial.begin(9600);
  Serial.println("------------------------------------------");
}

void loop()
{
  // point p_test_1;
  // p_test_1.pos_x = 10;
  // p_test_1.pos_y = 0;

  // point p_test_2;
  // p_test_2.pos_x = 2;
  // p_test_2.pos_y = 57;

  // res = test.calculateFScore(&p_test_1, &p_test_2);
  // Serial.println(res, BIN);

  random_i = random(0, ROW);
  random_j = random(0, COL);

  Serial.print("Setting random on: ");
  Serial.print(random_i, DEC);
  Serial.print(" , ");
  Serial.print(random_j, DEC);
  Serial.println();

  addObstacleCell(random_i, random_j);
  Serial.print("Is Obstacle: ");
  Serial.println(isObstacleCell(random_i, random_j));
  printGridOnSerial();

  delay(1000); 
}