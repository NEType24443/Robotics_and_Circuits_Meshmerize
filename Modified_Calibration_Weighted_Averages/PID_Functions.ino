void follow(bool ignore){
  if((unsigned long)(millis()-ms <= PID_sample_time)) return;
  last_line_pos = line_pos;
  line_pos = get_line_pos(COLOR, (last_line_pos>0), ignore);
  float PID_correction = get_PID_correction(line_pos, last_line_pos, KP, KD, KI);
  float max_correction = SPEED;    //Can be changed to a lower value in order to limit the correction, needs to be at most SPEED
  if(PID_correction > 0){
    PID_correction = (PID_correction > max_correction)? max_correction : PID_correction;
    Motor_Speed(SPEED, SPEED - PID_correction);
  }
  else{
    PID_correction = (PID_correction < -max_correction)? -max_correction : PID_correction;
    Motor_Speed(SPEED + PID_correction, SPEED);
  }
  ms = millis();
}

float get_line_pos(int color, int last_dir, bool ignore){
  float line = 0;
  int line_detected = 0;
  
  float avg_num = 0;          //Average numerator
  float avg_den = 0;          //Average denominator
  if(!ignore){
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
  }
  if(line_detected == 1){
    line = avg_num / avg_den;                           //Weighted average
    line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale from 0 -> 4000 to -2000 -> 2000  <- in case of 5 sensors
                                                        //Change scale form 0 -> 2000 to -1000 -> 1000  <- in case of 3 sensors
    dW(LEDG, LOW);
  }else{
    line = last_line_pos;     // Keep the value of line the same as before
    //  OR
    //line = 0;               // Set as zero to avoid turning
    //  OR
    //line = WEIGHT_UNIT * (N_SENS - 1) * last_dir;       //Use last direction to calculate error as the maximum value
    //line = line - (WEIGHT_UNIT * (N_SENS - 1) / 2);     //Change scale
    dW(LEDG, HIGH);
  }
  return line;
}

float get_PID_correction(float line, float last_line, float kp, float kd, float ki){
  float proportional = line;    //-1000 -> 1000
  float derivative = line - last_line;
  float integral = line + last_line;
  //float integral = 0;
  float correction = ( proportional*kp + derivative*kd + ki* integral);

  return correction;
}
