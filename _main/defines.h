
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

#define N_SENS        5       //Number of sensors
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
#define LINE_WIDTH 3
#define CONVERSION_FACTOR 10  //<-RANDOM VALUE TESTING REQUIRED  // Scaling factor to convert speed to cm/ms from a number betwen 0-1000
#define OVERSHOOT(distance)(((CONVERSION_FACTOR*(distance)) / SPEED))
// SENSOR_THRESHOLD is a value to compare reflectance sensor
// readings to to decide if the sensor is over a black line
#define SENSOR_THRESHOLD 300

// ABOVE_LINE is a helper macro that takes returns
// 1 if the sensor is over the line and 0 if otherwise
#define ABOVE_LINE(sensor)(analogRead(sensor) > SENSOR_THRESHOLD)

// Motor speed when turning. TURN_SPEED should always
// have a positive value, otherwise the Zumo will turn
// in the wrong direction.
//#define TURN_SPEED 200

// Motor speed when driving straight. SPEED should always
// have a positive value, otherwise the Zumo will travel in the
// wrong direction.
//#define SPEED 200

// Thickness of your line in inches
//#define LINE_THICKNESS .75

// When the motor speed of the zumo is set by
// motors.setSpeeds(200,200), 200 is in ZUNITs/Second.
// A ZUNIT is a fictitious measurement of distance
// and only helps to approximate how far the Zumo has
// traveled. Experimentally it was observed that for
// every inch, there were approximately 17142 ZUNITs.
// This value will differ depending on setup/battery
// life and may be adjusted accordingly. This value
// was found using a 75:1 HP Motors with batteries
// partially discharged.
//#define INCHES_TO_ZUNITS 17142.0

// When the Zumo reaches the end of a segment it needs
// to find out three things: if it has reached the finish line,
// if there is a straight segment ahead of it, and which
// segment to take. OVERSHOOT tells the Zumo how far it needs
// to overshoot the segment to find out any of these things.
//#define OVERSHOOT(line_thickness)(((INCHES_TO_ZUNITS * (line_thickness)) / SPEED))

/*  PID PARAMS */
uint16_t SPEED = 300;  // PID controlled bot speed
const float KP = .3;
const float KD = .6;
const float KI = .0;  //.0001;
const uint16_t PID_sample_time = 5; // 5ms sample time

/* GLOBAL VARIABLES */
float sens_scaled[N_SENS];
unsigned long ms = 0;
const int SENSOR[N_SENS] = { A0, A1, A2, A3, A4 };  //Arduino pins
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
