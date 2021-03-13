#define dW digitalWrite
#define dR digitalRead

#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

/* PIN DEFINITION */

// Software Serial
#define BT_TX A5
#define BT_RX A6

// LED
#define LEDG  13

// Button
#define BTN   12

// Motor
#define MOT1  5
#define MOT2  6   // LF
#define MOT3  9
#define MOT4  11  // RF

//IR PID SENSORS
#define sensorPIDL A0
#define sensorPIDC A1     
#define sensorPIDR A2

//IR DIRECTION SENSORS
#define sensorC   2
#define sensorLA  3
#define sensorRA  4
#define sensorL   7
#define sensorR   8

/* PID CONTROLER GLOBAL CONSTANTS */

#define WHITE  1
#define BLACK  0

#define COLOR   WHITE         //Line color

#define N_SENS        3       //Number of sensors
#define R_SENS        1000    //Sensor readings are mapped to this range
#define WEIGHT_UNIT   1000    //Unit for weighted average
#define RIGHT_DIR     1
#define LEFT_DIR      0
#define CAL_SPEED     100     //Sensor calibration speed
#define CAL_TIME      4000    //Calibration time
#define P_LINE_MIN    0.5     //Minimum brightness percentage to consider part of the line

#define STOPPED           0
#define FOLLOWING_LINE    1
#define NO_LINE           2
#define RIGHT_TURN        3
#define LEFT_TURN         4
#define RIGHT_ANGLED_TURN 5   // <- AND BELOW ARE MINE
#define LEFT_ANGLED_TURN  6 
#define T_INTER           7
#define CROSS_INTER       8
#define LA_R_INTER        9
#define RA_L_INTER        10
#define Y_INTER           11
#define S_L_INTER         12
#define S_R_INTER         13
#define ONCOMING          14

#define RIGHT 1
#define LEFT -1

//OVERSHOOT FUNCTION AND RELATED VALUES
#define LINE_WIDTH 3 // 3cm width
#define CONVERSION_FACTOR 10  //<-RANDOM VALUE TESTING REQUIRED  // Scaling factor to convert speed to cm/ms from a number between 0-1000
#define OVERSHOOT(distance)(((CONVERSION_FACTOR*(distance)) / SPEED))
/*
inline overshoot(int distance){
  return();
}
*/ // Alternative way to define OVERSHOOT
/*  PID PARAMS */
uint16_t SPEED = 300;  // PID controlled bot speed
const float KP = .3;
const float KD = .6;
const float KI = .0;  //.0001;
const uint16_t PID_sample_time = 5; // 5ms sample time

/* GLOBAL VARIABLES */
float sens_scaled[N_SENS];
unsigned long ms = 0;
const int SENSOR[N_SENS] = { sensorPIDL, 
                             sensorPIDC, 
                             sensorPIDR };  //Arduino pins
int sens_max[N_SENS];          //Maximum value each sensor measures (calibrated)
int sens_min[N_SENS];          //Minimum value each sensor measures (calibrated)
int start = 0;
float line_pos = 0;
float last_line_pos = 0;

//ir array values
bool ir[5];
uint8_t ir_val = 0;

/* MPU-6050 VARIABLES */
float initZAngle = 0;
unsigned int  zAngle = 0, 
              Gyro_sample_time = 1000; // Update every second
unsigned long Gyro_ms = 0;

/*  MAZE_SOLVE VARIABLES */
int mode = 0;
const int adj = 0;
float adjTurn = 8;
int extraTime = 200;
int adjGoAndTurn = 800;

//Specific Maze optimization definitions and variables 
unsigned char dir; 

// The path variable will store the path that the robot has taken:
//  'L' for left
//  'l' for left angled
//  'R' for right
//  'r' for right angled
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)

char path[70] = "";
unsigned int pathLength = 0; // the length of the path
uint16_t pathIndex = 0;
unsigned int status = 0; // solving = 0; reach end = 1
