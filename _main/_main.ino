
#include "defines.h"

class SensorClass{
  private:
    byte num_sens = 5;
    int* sensor_val[], sensor[];
  public:
    SensorClass(byte num_sens, int* sensor_val, int* sensor){
      this->num_sens    = num_sens;
      this->sensor_val[num_sens]  = sensor_val[num_sens];
      this->sensor[num_sens]= sensor[num_sens];
      }
    void readLine(){
      for(int i=0; i<num_sens; i++){
        sensor_val[i] = analogRead(sensor[i]);  
      }
    }
};

class MotorClass{
  void setSpeeds(uint16_t left_speed, uint16_t right_speed){
    int pwm1 = map(abs(left_speed), 0, 1000, 0, 255)+ adj;
    int pwm2 = map(abs(right_speed), 0, 1000, 0, 255);
    //pwm1 = (m1>0) ? 255-pwm1 : pwm1;  //Original
    pwm1 = (m1>=0) ? pwm1 : 255-pwm1;  //Modded
    pwm2 = (m2>=0) ? pwm2 : 255-pwm2; 
    analogWrite(MOT2, pwm1);
    analogWrite(MOT4, pwm2);
    //digitalWrite(MOT1, (m1 > 0) ? HIGH : LOW);  //Original
    digitalWrite(MOT1, (m1 >= 0) ? LOW : HIGH);   //Modded
    digitalWrite(MOT3, (m2 >= 0) ? LOW : HIGH);
  }
  
  void calibrate(int cal_time, int cal_speed, int cal_dir){
    ms = millis();
    dW(LEDG, LOW);
    while((ms + cal_time) > millis()){
      dW(LEDG, millis()%100 < 50);        //Blink led
      if(cal_dir == RIGHT_DIR)  motors.setSpeeds(cal_speed, -cal_speed);
      if(cal_dir == LEFT_DIR )  motors.setSpeeds(-cal_speed, cal_speed);
      int sens_value[N_SENS];
      for(int x = 0; x < N_SENS; x++){
        sens_value[x] = analogRead(SENSOR[x]);
        sens_min[x] = (sens_value[x] < sens_min[x]) ? sens_value[x] : sens_min[x];
        sens_max[x] = (sens_value[x] > sens_max[x]) ? sens_value[x] : sens_max[x];
      }
    }
    motors.setSpeeds(0, 0);
    dW(LEDG, HIGH);
  }
  
};
