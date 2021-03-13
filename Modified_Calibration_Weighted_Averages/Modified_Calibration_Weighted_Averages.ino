#include "defines.h"

SoftwareSerial BTSerial(BT_RX, BT_TX); //A6, A5 

MPU6050 mpu6050(Wire);

void setup() {
  BTSerial.begin(9600);
  //Serial.begin(9600);
  while(!Serial);
  InitializeGPIO();
  
  for(int x = 0; x < N_SENS; x++){
    sens_max[x] = 0;
    sens_min[x] = 1023;
  } // Values have to be initialised first
  
  delay(5);  // Wait for MPU to Startup
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  initZAngle = mpu6050.getAngleZ();
  BTSerial.print(initZAngle);
  
  delay(100);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  delay(100);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR );
  delay(100);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR );
  delay(100);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  delay(10000);
  mode = STOPPED;
  status = 0;
}

void InitializeGPIO(){
  pinMode(LEDG, OUTPUT);
  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);
  pinMode(BTN , INPUT );
  for(int x = 0; x < N_SENS; x++){
    pinMode(SENSOR[x], INPUT);
    digitalWrite(SENSOR[x], LOW);
  }
}

void loop() {
  while (digitalRead(BTN) && !mode){
    //button_prog only waits
  }
  mpu6050.update();
  zAngle = (unsigned int)(mpu6050.getAngleZ() - initZAngle);
  BTSerial.println(zAngle);
  //IR();  //  DO FOR CODE WHERE INITIAL ORIENTATION MATTERS
  //correct_bot_pos();
  // DO NOT UPDATE ms = millis();
  mazeSolve(); // First pass to solve the maze
  BTSerial.println("Done First Pass");
  while (digitalRead(BTN) && !mode){
    //button_prog only waits
  }
  BTSerial.println("Starting 2nd Pass");
  pathIndex = 0;
  status = 0;
  mazeOptimization(); //run the maze as fast as possible
  pathIndex = 0;
  status = 0;
  BTSerial.println("End 2nd Pass");
  while (digitalRead(BTN) && !mode){
    //button_prog only waits
  }
  mode = STOPPED;
  status = 0; // 1st pass
  pathIndex = 0;
  pathLength = 0;
}
