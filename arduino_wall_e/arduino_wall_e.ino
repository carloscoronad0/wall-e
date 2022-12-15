#include "a_star.h"

// Motor A
const int ENA = 5;
const int IN1 = 3;
const int IN2 = 4;
// Motor B
const int ENB = 6;
const int IN3 = 7;
const int IN4 = 8;

///// Sensors pins
// Distance Sensors
const int dsFront = A0;
const int dsLeft = A1;
const int dsRight = A2;

// Light Sensors;
const int lsFront = A4;
const int lsLeft = A5;
const int lsRight = A6;

// Variables to capture sensors values
int dsFrontValue;
int dsLeftValue;
int dsRightValue;

int lsFrontValue;
int lsLeftValue;
int lsRightValue;

point initial = {.pos_x = 10, .pos_y = 0};
point final = {.pos_x = 10, .pos_y = 57};
bool obstacleFound = false;

AStar a_star;

void setup() {
  // Motors pings
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  // Initialize grid
  a_star.initGrid();

  Serial.begin(9600);
  Serial.println("------------------------------------------");
}

void loop()
{
  // Read Values of Sensors
  dsFrontValue = analogRead(dsFront);
  dsLeftValue = analogRead(dsLeft);
  dsRightValue = analogRead(dsRight);

  lsFrontValue = analogRead(lsFront);
  lsLeftValue = analogRead(lsLeft);
  lsRightValue = analogRead(lsRight);

  // Detect Distance Sensors Obstacles
  if(dsFrontValue > 600){
    Serial.begin("Wall in front");
    if(initial.pos_x < ROW && initial.pos_y + 1 < GRID_DIV_SCALE * GRID_DIV){
      if(!(a_star.isObstacleCell(initial.pos_x, initial.pos_y + 1))){
        a_star.addObstacleCell(initial.pos_x, initial.pos_y + 1);
        obstacleFound = true;
      } 
    }
  }

  if(dsLeftValue > 600){
    Serial.begin("Wall in left");
    if(initial.pos_x - 1 >= 0){
      if(!(a_star.isObstacleCell(initial.pos_x - 1, initial.pos_y))){
        a_star.addObstacleCell(initial.pos_x - 1, initial.pos_y);
        obstacleFound = true;
      }
    }
  }
  if(dsRightValue > 600){
    Serial.begin("Wall in right");
    if(initial.pos_x + 1 < ROW){
      if(!(a_star.isObstacleCell(initial.pos_x + 1, initial.pos_y))){
        a_star.addObstacleCell(initial.pos_x + 1, initial.pos_y);
        obstacleFound = true;
      }
    }
  }

  // Detect Light Sensors Obstacles
  if(lsFrontValue > 600){
    Serial.begin("Light in front");
    if(initial.pos_y + 1 < GRID_DIV_SCALE * GRID_DIV){
      if(!(a_star.isObstacleCell(initial.pos_x, initial.pos_y + 1))){
        a_star.addObstacleCell(initial.pos_x, initial.pos_y + 1);
        obstacleFound = true;
      }
    }
  }
  if(lsLeftValue > 600){
    Serial.begin("Light in left");
    if(initial.pos_x - 1 >= 0 && initial.pos_y + 1 < GRID_DIV_SCALE * GRID_DIV){
      if(!(a_star.isObstacleCell(initial.pos_x - 1, initial.pos_y + 1))){
        a_star.addObstacleCell(initial.pos_x - 1, initial.pos_y + 1);
        obstacleFound = true;
      }
    }
  }
  if(lsRightValue > 600){
    Serial.begin("Light in right");
    if(initial.pos_x + 1 < ROW && initial.pos_y + 1 < GRID_DIV_SCALE * GRID_DIV){
      if(!(a_star.isObstacleCell(initial.pos_x + 1, initial.pos_y + 1))){
        a_star.addObstacleCell(initial.pos_x + 1, initial.pos_y + 1);
        obstacleFound = true;
      }
    }
  } 

  // Move Robot
}


// Movement Functions
void goForward (){
  // Direction (Motor A)
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
   // Velocity (Motor A)
  analogWrite (ENA, 100);

  // Direction (Motor B)
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  // Velocity (Motor B)
  analogWrite (ENB, 100);
}
void goBack (){
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, 128);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, 128);
}
void goRight (){
  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 200);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, HIGH);
  analogWrite (ENB, 100);
}
void goLeft (){
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, HIGH);
  analogWrite (ENA, 50);

  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 150);
}
void stop (){
  digitalWrite (IN1, LOW);
  digitalWrite (IN2, LOW);
  analogWrite (ENA, 0);

  digitalWrite (IN3, LOW);
  digitalWrite (IN4, LOW);
  analogWrite (ENB, 0);
}
