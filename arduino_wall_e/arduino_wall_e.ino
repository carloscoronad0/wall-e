char grid[19][58];

void setup() {
  Serial.begin(9600);  
}

void loop()
{
  int x_pos = random(0, 42);
  int y_pos = random(0,115);

  grid[x_pos][y_pos] = 'A';

  for (int i = 0; i < 19; i++){
    for (int j = 0; j < 58; j++){
      Serial.print(grid[i][j]);
    }
    Serial.print("\n");
  }  
  
  delay(1000); 
}