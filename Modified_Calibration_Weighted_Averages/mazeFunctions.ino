void mazeSolve(void){
  while (!status)
  {   
    /*
    if((unsigned long)(millis()-Gyro_ms <= Gyro_sample_time)){
      mpu6050.update();
      zAngle = (unsigned int)(mpu6050.getAngleZ() - initZAngle);
    } */
    Read_Direction_Sensor();
    Cases();
  }
}

void Cases(void){
  switch (mode)
  {   
    case NO_LINE:
      Following(3);    // Distance between sensorC and PID sensors.
      Run_Extra(2);    // Distance between PID sensors and rotation axis. 
      delay(100);
      Motor_Turn(LEFT, 180);
      Rec_Inter('B');
      break;
    
    case T_INTER:
      Motor_Turn (LEFT, 90);
      Rec_Inter('L'); // it is a "T" or "Cross". In both cases, goes to LEFT
      Following(4);   // Follow Line for 2 CM
      break;
    
    case CROSS_INTER:
      Motor_Turn (LEFT, 90);
      Rec_Inter('L'); // it is a "T" or "Cross". In both cases, goes to LEFT
      Following(4);
      break;
    
    case S_L_INTER:
      Motor_Turn (LEFT, 90);
      Rec_Inter('L'); // it is a "T" or "Cross". In both cases, goes to LEFT
      Following(4);
      break;
    
    case S_R_INTER:
      Motor_Turn (RIGHT, 90);
      Rec_Inter('R');
      Following(4);
      break;
    
    case LA_R_INTER:
      Motor_Turn (LEFT, 45);
      Rec_Inter('l');
      Following(2);
      break;
      
    case RA_L_INTER:
      Motor_Turn (RIGHT, 45);
      Rec_Inter('r');
      Following(2);
      break;
      
    case Y_INTER:
      Motor_Turn (LEFT, 45);
      Rec_Inter('l');
      Following(4);
      break;
      
    case RIGHT_TURN: 
      Run_Extra(LINE_WIDTH/2);
      delay(100);
      Read_Direction_Sensor();
      if (mode == RIGHT_TURN) {Motor_Turn (RIGHT, 90); Rec_Inter('R'); Following(4);}
      //else Rec_Inter('S');
      break;   
      
    case LEFT_TURN: 
      Run_Extra(LINE_WIDTH/2);
      delay(100);
      Read_Direction_Sensor();
      if (mode == LEFT_TURN ) {Motor_Turn (LEFT, 90);  Rec_Inter('L'); Following(4);}
      //else Rec_Inter('S');
      break;
      
    case RIGHT_ANGLED_TURN:
      Run_Extra(LINE_WIDTH/2);
      delay(100);
      Read_Direction_Sensor();
      if (mode == RIGHT_ANGLED_TURN) {Motor_Turn (RIGHT, 45); Rec_Inter('r'); Following(4);}
      break;
      
    case LEFT_ANGLED_TURN:
      Run_Extra(LINE_WIDTH/2);
      delay(100);
      Read_Direction_Sensor();
      if (mode == LEFT_ANGLED_TURN ) {Motor_Turn (LEFT, 45);  Rec_Inter('l'); Following(4);}
      break;
      
    case FOLLOWING_LINE: 
      follow(false);
      break;
      
    case ONCOMING:
      Following(4);
      while(mode == ONCOMING){
        Motor_Speed(SPEED, SPEED);
        Read_Direction_Sensor();
      }
      Motor_Speed(0,0);
      break;
  }
}

void Following(uint8_t distance){
  unsigned long duration = OVERSHOOT(distance);
  unsigned long MS = millis();
  while(millis()-MS<duration){
    follow(false);
  }
}

void Rec_Inter(char direction){
  path[pathLength] = direction; // Store the intersection in the path variable.
  pathLength ++;
  simplifyPath(); // Simplify the learned path.
}

void mazeEnd(void){
  Motor_Speed(0, 0);
  BTSerial.print("The End  ==> Path: ");
  for(uint16_t i=0;i<pathLength;i++)
    BTSerial.print(path[i]);
    //Serial.print(path[i]);
  BTSerial.println("");
  Serial.print("  pathLength ==> ");
  Serial.println(pathLength);
  status = 1;
  mode = STOPPED;
}

// Path simplification.  The strategy is that whenever we encounter a
// sequence xBx, we can simplify it by cutting out the dead end.  For
// example, LBL -> S, because a single S bypasses the dead end
// represented by LBL.
void simplifyPath(){
  // only simplify the path if the second-to-last turn was a 'B'
  if(pathLength < 3 || path[pathLength-2] != 'B')
    return;
  bool is_angled = false;
  int totalAngle = 0;
  int i;
  for(i=1;i<=3;i++)
  {
    switch(path[pathLength-i])
    {
      case 'R':
        totalAngle += 90;
        break;
      case 'L':
        totalAngle += 270;
        break;
      case 'r':     // Added by me
        totalAngle += 90;
        is_angled = true;
        break; 
     case 'l':     // Added by me
        totalAngle += 270;
        is_angled = true;
        break; 
      case 'B':
        totalAngle += 180;
        break;
    }
  }
  // Get the angle as a number between 0 and 360 degrees.
  totalAngle = totalAngle % 360;
  // Replace all of those turns with a single one.
  switch(totalAngle)
  {
    case 0:
      path[pathLength - 3] = 'S';
      break;
    case 90:
      path[pathLength - 3] = (is_angled)? 'r' : 'R';
      break;
    case 180:
      path[pathLength - 3] = 'B';
      break;
    case 270:
      path[pathLength - 3] = (is_angled)? 'l' : 'L';
      break;
  }
  // The path is now two steps shorter.
  pathLength -= 2;
} 


void mazeOptimization (void){
  while (!status)
  {
    Read_Direction_Sensor();
    switch(mode)
    {
      case FOLLOWING_LINE:
        follow(false);
        break;
      case LEFT_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case LEFT_ANGLED_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case RIGHT_ANGLED_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;    
      case RIGHT_TURN:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;  
      case T_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case CROSS_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case S_L_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case S_R_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case LA_R_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case RA_L_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
      case Y_INTER:
        if (pathIndex >= pathLength) mazeEnd (); 
        else {mazeTurn (path[pathIndex]); pathIndex++;}
        break;
    }
  }
}


void mazeTurn (char dir) {
  //mpu6050.update();
  //zAngle = (unsigned int)(mpu6050.getAngleZ() - initZAngle);
  switch(dir){
    case 'L': // Turn Left
      Motor_Turn (LEFT, 90);
      break;   
    case 'R': // Turn Right
      Motor_Turn (RIGHT, 90);     
      break;
    case 'l': // Turn Left angled           ADDED
      Motor_Turn (LEFT, 45);      
      break; 
    case 'r': // Turn right angled          ADDED
      Motor_Turn (RIGHT, 45);      
      break;
    case 'B': // Turn Back
      Motor_Turn (LEFT, 180);     
      break;
    case 'S': // Go Straight
      Run_Extra(5); 
      break;
  }
}
