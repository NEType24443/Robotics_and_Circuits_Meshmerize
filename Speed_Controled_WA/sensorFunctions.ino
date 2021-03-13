void Read_Direction_Sensor(void){
  Direction_IR();
  ir_to_mode();
}

void Direction_IR(){
  /*ir[0] = digitalRead(sensorR); // Right Most
  //Serial.print("1 : ");
  //Serial.println(!ir[1]);
  ir[1] = digitalRead(sensorRA); 
  //Serial.print("2 : ");
  //Serial.println(!ir[2]);
  ir[2] = digitalRead(sensorC);
  //Serial.print("3 : ");
  //Serial.println(!ir[3]);
  ir[3] = digitalRead(sensorLA);
  //Serial.print("4 : ");
  //Serial.println(!ir[4]);
  ir[4] = digitalRead(sensorL); // Left Most
  ir_val = ir[4]<<4 | ir[3]<<3 | ir[2]<<2 | ir[1]<<1 | ir[0];*/
  ir_val = digitalRead(sensorL)<<4 | digitalRead(sensorLA)<<3 | digitalRead(sensorC)<<2 | digitalRead(sensorRA)<<1 | digitalRead(sensorR);
  ir_val = (COLOR)? ~ir_val : ir_val; //Because White is LOW and Black is HIGH
  Serial.print("ir_val: ");
  Serial.println(ir_val);
}

void ir_to_mode(void){
  switch(ir_val)
  {
    case 0b00000:
      mode = NO_LINE;
      break;
    case 0b10000:
      mode = LEFT_TURN;
      break;
    case 0b01000:
      mode = LEFT_ANGLED_TURN;
      break;
    case 0b00100:
      mode = FOLLOWING_LINE;
      break;
    case 0b00010:
      mode = RIGHT_ANGLED_TURN;
      break;
    case 0b00001:
      mode = RIGHT_TURN;
      break;
    case 0b10001:
      mode = T_INTER;
      break;
    case 0b10100:
      mode = S_L_INTER;
      break;
    case 0b00101:
      mode = S_R_INTER;
      break;
    case 0b10101:
      mode = CROSS_INTER;
      break;
    case 0b01001:
      mode = LA_R_INTER;
      break;
    case 0b10010:
      mode = RA_L_INTER;
      break;
    case 0b01100:
      mode = ONCOMING;
      break;
    case 0b00110:
      mode = ONCOMING;
      break;
    case 0b01110:
      mode = ONCOMING;
      break;
  }
}
