#define SPEED 300   //Y-axis base speed
#define MAX_SPEED 1000 
#define A_ (MAX_SPEED-1.5*SPEED)  // A_ is the max value of the curve.
                                  // B_ is the location of the peak.
                                  // C_ is the width of the bell.
                                
#define BELL_CURVE(x, c)   A_*exp(-float(sq(x-c/2))/float(2*sq(c/6)))   // Returns a value corresponding to the bell curve

#define SPEED_CONVERSION_FACTOR     0.2F    // Converts 0->1000 value to Euclidean speed in cm/s.
                                            // Can be found experimenatally only.
                                            
#define ENCODER_CONVERSION_FACTOR   1.25F   // Converts encoder value to Euclidean distance in mm.
                                            // Can be calculated using 2*PI*Radius/((Pulses/Rev)*(Gear Ratio)*(Counting Mode)).
                                            // Note the value can also be interpreted as the resolution of the encoder.
                                            // Counting Mode can be 1X, 2X and 4X for only rising/falling edge of one pin,
                                            //                                         rising and falling edge of one pin
                                            //                                   and rising and falling edge of both pins
                                            // Currently we are using the N20 motor with 1:50 Gr and 3 Pulses/Revolution(PPR) of encoder shaft.
                                            // This can be Quadrupled to 12 PPR by checking multiple state changes.
// Read https://www.motioncontroltips.com/faq-what-do-x1-x2-and-x4-position-encoding-mean-for-incremental-encoders/ for more info.
                                            // Our wheel diameter is approx 60mm I believe, but may need to change.
                                            // PI*60.0/(3*1*50) = 1.25 mm/pulse <- Resolution of encoder
                                            
/* TIME IS DESIGNED TO ADJUST THE DISTANCE UPDATE RATE TO THAT OF REALISTIC VALUES*/

// When rate of change of speed changes rate of change of distance will also change proportionally.
// The below function adjusts the rate of change of distance this is to emulate the behaviour of the bot better.

#define TIME(s, p)   float(abs(ENCODER_CONVERSION_FACTOR*p-END/2)*100.0F)/float(SPEED_CONVERSION_FACTOR*float(s))  // returns time in ms

#define END   230.0F  // One segment is 24cm in length.
#define END_2 340.0F  // One diagonal segment is 34cm in length.

uint16_t  lastDistance = 0,   // 
          readDistance = 0,   // Updated by ISR Routine from the Teensy LC  but manually updated here
          speed = 0,          //
          sample_time = 0,
          prev_sample = 0;
          
uint32_t  lastTime,
          time_taken = 0;

void setup() {
  Serial.begin(115200);         //establish serial connection for debugging
  pinMode(13, OUTPUT);
  delay(3000);
  lastTime = millis();
}

void loop() {
  //if(millis()-lastTime >= 100){
  //speed = bell_curve(readDistance, END);
  //print_curve(speed);
  speed = BELL_CURVE(int(float(readDistance)*ENCODER_CONVERSION_FACTOR), END);
  sample_time  = TIME(speed + SPEED, readDistance);
  update_distance();
  print_curve();
  delay(1);
  if(int(float(readDistance)*ENCODER_CONVERSION_FACTOR) > END){
    //Serial.print(" Total Time Taken : ");
    //Serial.println(time_taken);
    while(1) digitalWrite(13, millis()%1000<200);
  }
}

void update_distance(void){
  if(millis() - lastTime > sample_time){
    readDistance++;
    //prev_sample = sample_time;
    time_taken += sample_time;
    lastTime = millis();
  }
}

void print_curve(void) {
  //if(abs(sample_time-prev_sample)>0) return;
  //Serial.print("Speed: ");
  Serial.print(speed + SPEED);
  Serial.print(',');
  //Serial.print(" Distance_Covered: ");
  Serial.println(int(float(readDistance)*ENCODER_CONVERSION_FACTOR));
  //Serial.print(',');
  //Serial.print(" Time Period: ");
  //Serial.println(sample_time);
}

uint16_t bell_curve(int distance, uint32_t endPoint){
  uint32_t b = endPoint/2, c = endPoint/6 ;
  //lastDistance = distance;
  return(A_*exp(-float(sq(distance-b  ))/float(2*sq(c))));
//       A_*exp(-float(sq(x       -c/2))/float(2*sq(c)))
}
