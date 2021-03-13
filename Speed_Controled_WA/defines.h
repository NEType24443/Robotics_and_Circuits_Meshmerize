#define dW digitalWrite
#define dR digitalRead
#define ENCODER_USE_INTERRUPTS

#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Encoder.h>
/* PIN DEFINITION */

// Software Serial
#define BT_TX  8
#define BT_RX  7

// LED
#define LEDG  13

// Button
#define BTN   12

// Motor
#define MOT1  5
#define MOT2  3   // LF
#define MOT3  6
#define MOT4  4   // RF

//IR PID SENSORS
#define sensorPIDL 14
#define sensorPIDC 15     
#define sensorPIDR 16

//IR DIRECTION SENSORS
#define sensorC   9
#define sensorLA  10
#define sensorRA  17
#define sensorL   11
#define sensorR   16

// ENCODER CONNECTIONS
#define LF 20
#define LB 21
#define RF 22
#define RB 23

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

/* OVERSHOOT FUNCTION AND RELATED VALUES*/
#define LINE_WIDTH 3
#define CONVERSION_FACTOR 10  //<-RANDOM VALUE TESTING REQUIRED  // Scaling factor to convert speed to cm/ms from a number betwen 0-1000
#define OVERSHOOT(distance)(((CONVERSION_FACTOR*(distance)) / SPEED))

/* SPEED CONTROL AND RELATED VALUES */

// PATH LENGTHS
#define END       230.0F  // One segment is 24cm in length.
#define END_R2    340.0F  // One diagonal segment is 34cm in length. 
// SPEED CONTROL
#define MAX_SPEED 1000 
#define SPEED 300
#define A_ (MAX_SPEED-1.5*SPEED)  // A_ is the max value of the curve.
                                  // B_ is the location of the peak.
                                  // C_ is the width of the bell.
                                
#define BELL_CURVE(x, c)   A_*exp(-float(sq(x-c/2))/float(2*sq(c/6)))   // Returns a value corresponding to the bell curve

#define SPEED_CONVERSION_FACTOR     0.2F    // Converts 0->1000 value to Euclidean speed in cm/s.
                                            // Can be found experimenatally only.
                                            
#define ENCODER_CONVERSION_FACTOR   1.25F   // Converts encoder value to Euclidean distance in mm.
                                            // Can be calculated using 2*PI*Radius/(Pulses/Rev).
                                            // Note the value can also be interpreted as the resolution of the encoder.
                                            // Currently we are using the N20 motor with 3 Pulses/Revolution(PPR) of motorshaft.
                                            // This can be Quadrupled to 12 PPR by checking multiple state changes.
// Read https://www.motioncontroltips.com/faq-what-do-x1-x2-and-x4-position-encoding-mean-for-incremental-encoders/ for more info.
                                            // Our wheel diameter is approx 60mm I believe, but may need to change.
                                            // PI*60.0/(3*4) = 15.7 mm/pulse <- Resolution of encoder

/* ENCODER CLASS DEFINITION */
Encoder LeftEncoder (LF, LB);
Encoder RightEncoder(RF, RB);

/*  PID PARAMS */
uint16_t  speed = 300,  // Bell Curve adjusted PID controlled bot speed
          readDistance = 0;
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

const int adj = 0;
float adjTurn = 8;
int mode = 0,
    extraTime = 200,
    adjGoAndTurn = 800;

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
unsigned int  pathLength = 0, // the length of the path
              status = 0;     // solving = 0; reach end = 1
uint16_t pathIndex = 0;
