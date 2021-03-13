#define dW digitalWrite
#define dR digitalRead

#include <SoftwareSerial.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


SoftwareSerial BTSerial(3, 10); //RX, TX //SoftwareSerial BTSerial(3, 10); //RX, TX

/* PIN DEFINITION */

#define LEDG  13

#define MOT1  5
#define MOT2  6   // LF
#define MOT3  9
#define MOT4  11  // RF

/* GLOBAL CONSTANTS */

#define WHITE  1
#define BLACK  0

#define COLOR   WHITE         //Line color

#define N_SENS        3       //Number of sensors
#define R_SENS        1000    //Sensor readings are mapped to this range
#define WEIGHT_UNIT   1000    //Unit for weighted average
#define RIGHT_DIR     1
#define LEFT_DIR      0
#define PRESS         0
#define RELEASE       1
#define CAL_SPEED     500   //Sensor calibration speed
#define CAL_TIME      1000     //Calibration time
#define P_LINE_MIN    0.5     //Minimum brightness percentage to consider part of the line

const float SPEED = 300;
const float KP = .3;  //0.3
const float KD = .6 ;
const float KI = .0001;

/* GLOBAL VARIABLES */

float sens_scaled[N_SENS];
unsigned long ms = 0;
const int SENSOR[N_SENS] = { A0, A1, A2};     //Arduino pins
int sens_max[N_SENS];          //Maximum value each sensor measures (calibrated)
int sens_min[N_SENS];          //Minimum value each sensor measures (calibrated)
int start = 0;
float line_pos = 0;
float last_line_pos = 0;


//IR ARRAY
#define sensor1 A0     // RIGHT most sensor
#define sensor2 A1     
#define sensor3 A2

#define sensorLA 
#define sensorRA 
#define sensorL 
#define sensorR

#define LED 12

MPU6050 mpu6050(Wire);

bool ir[5];  //ir array values

bool runBot = true;

float zAngle, initZAngle;

uint8_t ir_val;

void setup() {
  BTSerial.begin(9600);
  Serial.begin(9600);
  while(!Serial);
  InitializeGPIO();
  
  for(int x = 0; x < N_SENS; x++){
    sens_max[x] = 0;
    sens_min[x] = 1023;
  } 
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  initZAngle = mpu6050.getAngleZ();
  BTSerial.print(initZAngle);
  delay(500);
  digitalWrite(i1,0);
  digitalWrite(i2,1);
  digitalWrite(i3,1);
  digitalWrite(i4,0);
  delay(250);
}

void InitializeGPIO(){
  pinMode(LEDG, OUTPUT);
  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);

  for(int x = 0; x <= N_SENS; x++){
    pinMode(SENSOR[x], INPUT);
  }
}

void loop() {
  delay(1000);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  delay(10);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
  delay(10);
  calibrate(CAL_TIME, CAL_SPEED, LEFT_DIR);
  delay(10);
  calibrate(CAL_TIME, CAL_SPEED, RIGHT_DIR);
  delay(10000);
  while(1){ 
    if(runBot){
      mpu6050.update();
      zAngle = mpu6050.getAngleZ();//- initZAngle;
      error = zAngle;
      Serial.println(zAngle);
      find_position();
      line_val();
      //calculate_PID();
    }
    race();
    BTSerial.print(line_pos);
    BTSerial.print(sens_scaled[0] );
    BTSerial.print('\t');
    BTSerial.print(sens_scaled[1]);
    BTSerial.print('\t');
    BTSerial.print(sens_scaled[2]);
    BTSerial.print('\t');
    BTSerial.print(sens_scaled[3]);
    BTSerial.print('\t');
    BTSerial.print(sens_scaled[4]);
    BTSerial.println('\t');
    delay(10);
  }  
}


void race(void){
  last_line_pos = line_pos;
  line_pos = get_line_pos(COLOR, (last_line_pos>0));
  float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
  float max_correction = SPEED;    //Can be changed to a lower value in order to limit the correction, needs to be at most SPEED
  if(PID_correction > 0){
    PID_correction = (PID_correction > max_correction)? max_correction : PID_correction;
    motorSpeed(SPEED, SPEED - PID_correction);
  }
  else{
    PID_correction = (PID_correction < -max_correction)? -max_correction : PID_correction;
    motorSpeed(SPEED + PID_correction, SPEED);
  }
}


float get_line_pos(int color, int last_dir){
  float line = 0;
  int line_detected = 0;
  
  float avg_num = 0;          //Average numerator
  float avg_den = 0;          //Average denominator
  
  for(int x = 0; x < N_SENS; x++){
    //Scale conversion from 0->1023 to 0->R_SENS in this case 0->1000
    sens_scaled[x] = analogRead(SENSOR[x]) - sens_min[x];
    sens_scaled[x] *= R_SENS;
    sens_scaled[x] /= (sens_max[x] - sens_min[x]);
    if(color == BLACK){
      sens_scaled[x] = R_SENS - sens_scaled[x];     //Reverse scale to go from R_SENS to 0 essentially invert values
    }
    if(sens_scaled[x] >= (float) R_SENS * ((float)P_LINE_MIN / 100.0)){   //At least one sensor has to detect a line
      line_detected = 1;
    }
    avg_num += sens_scaled[x] * x * WEIGHT_UNIT;
    avg_den += sens_scaled[x];
  }
  if(line_detected == 1){
    line = avg_num / avg_den;                           //Weighted average
    line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale from 0 -> 4000 to -2000 -> 2000  <- incase of 5 sensors
                                                        //Change scale form 0 -> 2000 to -1000 -> 1000  <- incase of 3 sensors
    dW(LEDG, LOW);
  }else{
    line = WEIGHT_UNIT * (N_SENS - 1) * last_dir;       //Use last direction to calculate error as the maximum value
    line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale
    dW(LEDG, HIGH);
  } 
  return line;
}

float get_PID_correction(float line, float last_line, float kp, float kd, float ki){
  float proportional = line;
  float derivative = line - last_line;
  float integral = line + last_line;
  float correction = ( proportional*KP + derivative*KD + KI* integral);

  return correction;
}

void motorSpeed(int m1, int m2) {           //From -1000 to 1000
  int pwm1 = map(abs(m1), 0, 1000, 0, 255);
  int pwm2 = map(abs(m2), 0, 1000, 0, 255);
  pwm1 = (m1>0) ? 255-pwm1 : pwm1; // pwm1 = (m1>=0) ? pwm1 : 255-pwm1;  
  pwm2 = (m2>=0) ? pwm2 : 255-pwm2; 
  analogWrite(MOT2, pwm1);
  analogWrite(MOT4, pwm2);
  digitalWrite(MOT1, (m1 > 0) ? HIGH : LOW); // digitalWrite(MOT1, (m1 >= 0) ? LOW : HIGH);
  digitalWrite(MOT3, (m2 >= 0) ? LOW : HIGH);
}

void calibrate(int cal_time, int cal_speed, int cal_dir){
  ms = millis();
  dW(LEDG, LOW);
  while((ms + cal_time) > millis()){
    dW(LEDG, millis()%100 < 50);        //Blink led
    if(cal_dir == RIGHT_DIR)  motorSpeed(cal_speed, -cal_speed);
    if(cal_dir == LEFT_DIR)  motorSpeed(-cal_speed, cal_speed);
    int sens_value[N_SENS];
    for(int x = 0; x < N_SENS; x++){
      sens_value[x] = analogRead(SENSOR[x]);
      sens_min[x] = (sens_value[x] < sens_min[x]) ? sens_value[x] : sens_min[x];
      sens_max[x] = (sens_value[x] > sens_max[x]) ? sens_value[x] : sens_max[x];
    }
  }
  motorSpeed(0, 0);
  dW(LEDG, HIGH);
}

void IR(){
  ir[0] = digitalRead(sensor1); // Right Most
  ir[0]=(!ir[0]);
  //Serial.print("1 : ");
  //Serial.println(ir[1]);
  ir[1] = digitalRead(sensor2); 
  ir[1]=(!ir[1]);
  //Serial.print("2 : ");
  //Serial.println(ir[2]);
  ir[2] = digitalRead(sensor3);
  ir[2]=(!ir[2]);
  //Serial.print("3 : ");
  //Serial.println(ir[3]);
  ir[3] = digitalRead(sensor4);
  ir[3]=(!ir[3]);
  //Serial.print("4 : ");
  //Serial.println(ir[4]);
  ir[4] = digitalRead(sensor5); // Left Most
  ir[4]=(!ir[4]);
  ir_val = ir[4]<<4 | ir[3]<<3 | ir[2]<<2 | ir[1]<<1 | ir[0];
  Serial.print("ir_val: ");
  Serial.println(ir_val);
}


void error_val(void){
  if      (ir_val == 0b10000){
    error = 4;
  }
  else if (ir_val == 0b01000){
    error = 2;
  }
  else if (ir_val == 0b00100){
    error = 0;
  }
  else if (ir_val == 0b00010){
    error = -2;
  }
  else if (ir_val == 0b00001){
    error = -4;
  }
  else if (ir_val == 0b11000){
    error = 3;
  }
  else if (ir_val == 0b01100){
    error = 0.8;
  }
  else if (ir_val == 0b00110){
    error = -0.8;
  }
  else if (ir_val == 0b00011){
    error = -3;
  }
}

  // Every time you two are getting rough my mind just sounds like Shia Le Bouf it says "No No No No No "
  // Every time I see him with you my heart sounds like Shia Le Bouf to No No No,
  
  
  // Still, and silent calm before the storm,
  // gold and diamond jewels behind the throne,
  // into the night
  // out of the dark
  // take to the sky
  // chasing the stars
  // all that we say
  // all that we are
  // waiting to fly
  // this is the start.
  // Hide and seek,
  // reason and rhyme,
  // grand and glorious,
  // living the dream yours and mine, euphoria
  //  /Chorous/
  // Hide and seek,
  // reason and rhyme,
  // grand and glorious,
  // living the dream yours and mine, euphoria...
  // Stone and feather ,
  // move outside your head,
  // now or never,
  // strong in every step,
  // give me a sign
  // hitting the mark
  // take to the sky
  // chasing the stars  
  // open your eyes
  // watching afar
  // waiting to fly
  // this is the start.
  // Hide and seek,
  // reason and rhyme,
  // grand and glorious,
  // living the dream yours and mine, euphoria.
  //  /Chorous/
  // Hide and seek
  // reason and rhyme
  // grand and glorious
  // living the dream yours and mine, euphoria.


  // Come and fly away with me, x8
  // Dont you be afraid, 
