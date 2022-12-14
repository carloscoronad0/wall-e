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
const int dsFront;
const int dsLeft;
const int dsRight;

// Light Sensors;
const int lsFront;
const int lsLeft;
const int lsRight;

// Variables to capture sensors values
int dsFrontValue;
int dsLeftValue;
int dsRightValue;

int lsFrontValue;
int lsLeftValue;
int lsRightValue;

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

  // Detect Obstacles


  goForward();
  delay(2000);
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
