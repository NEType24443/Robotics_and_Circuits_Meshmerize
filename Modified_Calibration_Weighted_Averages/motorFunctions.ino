void Motor_Speed(int m1, int m2) {         //From -1000 to 1000
  int pwm1 = map(abs(m1), 0, 1000, 0, 255)+ adj;
  int pwm2 = map(abs(m2), 0, 1000, 0, 255);
  //pwm1 = (m1>0) ? 255-pwm1 : pwm1;  //Original
  pwm1 = (m1>=0) ? pwm1 : 255-pwm1;  //Modded
  pwm2 = (m2>=0) ? pwm2 : 255-pwm2; 
  analogWrite(MOT2, pwm1);
  analogWrite(MOT4, pwm2);
  //digitalWrite(MOT1, (m1 > 0) ? HIGH : LOW);  //Original
  digitalWrite(MOT1, (m1 >= 0) ? LOW : HIGH);   //Modded
  digitalWrite(MOT3, (m2 >= 0) ? LOW : HIGH);
}

void Motor_Turn(int direction, int degrees){  // Left degrees for if and when we need to take turn using the MPU 
  Motor_Speed(SPEED*direction, SPEED*(-direction));
  //delay (round(adjTurn*degrees+20));    // REPLACE WITH SENSOR BASED
  delay(1000);  // to avoid reading straight line and stopping before being able to turn
  while(!digitalRead(sensorC)){delay(2);}  // CHANGE IF NEEDED
  Motor_Speed(0, 0);
}

void Run_Extra(uint16_t distance){
  //follow(true);  // Original
  // OR
  Motor_Speed(SPEED,SPEED);   // Modded
  delay(OVERSHOOT(distance)); // Travel distance CM
  Motor_Speed(0,0);
}

void calibrate(int cal_time, int cal_speed, int cal_dir){
  ms = millis();
  dW(LEDG, LOW);
  while((ms + cal_time) > millis()){
    dW(LEDG, millis()%100 < 50);        //Blink led
    if(cal_dir == RIGHT_DIR)  Motor_Speed(cal_speed, -cal_speed);
    if(cal_dir == LEFT_DIR )  Motor_Speed(-cal_speed, cal_speed);
    int sens_value[N_SENS];
    for(int x = 0; x < N_SENS; x++){
      sens_value[x] = analogRead(SENSOR[x]);
      sens_min[x] = (sens_value[x] < sens_min[x]) ? sens_value[x] : sens_min[x];
      sens_max[x] = (sens_value[x] > sens_max[x]) ? sens_value[x] : sens_max[x];
    }
  }
  Motor_Speed(0, 0);
  dW(LEDG, HIGH);
}
