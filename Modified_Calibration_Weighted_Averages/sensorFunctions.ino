void Read_Direction_Sensor(void){
  Direction_IR();
  ir_to_mode();
}

void Direction_IR(){
  ir[0] = dR(sensorR); // Right Most
  //Serial.print("1 : ");
  //Serial.println(!ir[1]);
  ir[1] = dR(sensorRA); 
  //Serial.print("2 : ");
  //Serial.println(!ir[2]);
  ir[2] = dR(sensorC);
  //Serial.print("3 : ");
  //Serial.println(!ir[3]);
  ir[3] = dR(sensorLA);
  //Serial.print("4 : ");
  //Serial.println(!ir[4]);
  ir[4] = dR(sensorL); // Left Most
  ir_val = ir[4]<<4 | ir[3]<<3 | ir[2]<<2 | ir[1]<<1 | ir[0] ;
  ir_val = (COLOR)? ~ir_val : ir_val; //Because White is LOW and black is HIGH
  Serial.print("ir_val: ");
  Serial.println(ir_val);
}

void ir_to_mode(void){
  switch(ir_val)
  {
    case B00000:
      mode = NO_LINE;
      break;
    case B10000:
      mode = LEFT_TURN;
      break;
    case B01000:
      mode = LEFT_ANGLED_TURN;
      break;
    case B00100:
      mode = FOLLOWING_LINE;
      break;
    case B00010:
      mode = RIGHT_ANGLED_TURN;
      break;
    case B00001:
      mode = RIGHT_TURN;
      break;
    case B10001:
      mode = T_INTER;
      break;
    case B10100:
      mode = S_L_INTER;
      break;
    case B00101:
      mode = S_R_INTER;
      break;
    case B10101:
      mode = CROSS_INTER;
      break;
    case B01001:
      mode = LA_R_INTER;
      break;
    case B10010:
      mode = RA_L_INTER;
      break;
    case B01100:
      mode = ONCOMING; // LA Turn oncoming
      break;
    case B00110:
      mode = ONCOMING; // RA Turn oncoming
      break;
    case B01110:
      mode = ONCOMING; // Y Inter oncoming
      break;
  }
}
