#define UNIT_ERROR 0.1F
#define A_SIZE 8
uint16_t unit = 0, rootUnit = 0;
uint16_t encoder_val[A_SIZE] = {4, 5, 3, 4, 1414, 2820, 3010, 987};
float  unit_val[A_SIZE] = {};
void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
}

void loop() {
  if(Serial.available()>0){
    String data = Serial.readStringUntil('\n');
    Serial.println("\n"+String(data));
    if(data == "R"){
      unit = 0;
      rootUnit = 0;
    }
    else if(data == "N"){
      Serial.println("Waiting");
      Serial.flush();
      while(1){digitalWrite(13, millis()%500<100);if(Serial.available()>0){Serial.println("Broken");break;}}
      data = Serial.readStringUntil('\n');
      unit = data.toInt();
      rootUnit = sqrt(2)*unit;
      Serial.print("unit: " + String(unit)+ "\t");
      Serial.print("rootUnit: " + String(rootUnit));
      digitalWrite(13,LOW);
    }
    else if(data == "A"){
      Serial.println("Calculating Values");
      for (uint8_t x = 0; x<A_SIZE; x++){
         unit_val[x] = unit_converter(encoder_val[x]);
      }
    }
    else if(data == "P"){
      for (uint8_t x = 0; x<A_SIZE; x++){
         Serial.print(String(unit_val[x]) + "\t");
      }
    }
  }
}

float unit_converter(uint16_t val){
  if(val == 0)return(0);
  float quot = float(unit)/float(val);
  Serial.print(String(quot) + "\t");
  if     (abs(1-            quot)<UNIT_ERROR)return(1);
  else if(abs(1-  sqrt(2.0)*quot)<UNIT_ERROR)return(1.5);
  else if(abs(1-2*          quot)<UNIT_ERROR)return(2);
  else if(abs(1-2*sqrt(2.0)*quot)<UNIT_ERROR)return(2.5);
  else if(abs(1-3*          quot)<UNIT_ERROR)return(3);    //Not really needed
  else return(0);
}
