#include "a_star.h"

char res;
int res_as_int;

char x = 19;
char y = 58;
AStarGrid test(&x, &y);

void setup() {
  Serial.begin(9600);
}

void loop()
{
  point p_test_1;
  p_test_1.pos_x = 10;
  p_test_1.pos_y = 0;

  point p_test_2;
  p_test_2.pos_x = 2;
  p_test_2.pos_y = 57;

  res = test.calculateFScore(&p_test_1, &p_test_2);
  Serial.println(res, BIN);
  delay(1000); 
}