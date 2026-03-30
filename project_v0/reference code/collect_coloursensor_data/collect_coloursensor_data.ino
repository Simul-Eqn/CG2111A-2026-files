/* this is for: WHITE BLACK METHOD 

Usage instructions: 
Run the program while the bot is on a surface similar to the floor of the field. 
During setup, it'll first ask for white, which is that surface. 
THEN, IT WILL ASK FOR BLACK --> quickly place the bot on top of black 

Then, it'll start to serial printline a lot of values like (red,green,blue,), 
These values should be roughly around 0-1 (but may vary outside) 

Place the bot on top of a certain colour to collect data for that colour. 
Toggle off autoscroll (the button with the two downward arrowheads symbol) to be able to copy paste the data out 

Then you can e.g. copy those values into a .txt file or something 

Collect data on multiple different runs (re-calibrate) 

Also, at the start of each run it'll print 
BLACK: something,something,something 
WHITE: something,something,something 
consider recording that down too :] 
*/


// COLOUR SENSOR -------------------------------------------------------------------------------------------------------------------------

// Define time delay before taking another LDR reading
#define LDRWait 10  //in milliseconds
#define RGBWait 100
#define LightSensor A2 

#define baselineDecay 0.1


//floats to hold colour arrays
float currArray[] = { 0, 0, 0 };
float whiteArray[] = { 992.00,1007.00,997.00 }; // not initialized to 0 but doesn't matter as it'll be rewritten anyways 
float blackArray[] = { 0, 0, 0 };
float greyDiff[] = { 0, 0, 0 };
float baselineArray[] = { 0 , 0 , 0 };
char colourStr[3][6] = { "Red", "Green", "Blue" };

void activateIR() {
  digitalWrite(A0, LOW); 
  digitalWrite(A1, LOW);
}

void shineRed() {// Code for turning on the red LED only
  digitalWrite(A0, HIGH); 
  digitalWrite(A1, HIGH);
}
void shineGreen() {// Code for turning on the green LED only
  // green 
  digitalWrite(A0, LOW); 
  digitalWrite(A1, HIGH);
}
void shineBlue() {// Code for turning on the blue LED only
  digitalWrite(A0, HIGH); 
  digitalWrite(A1, LOW);
}

int getColAvgReading(int times) {
  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(LightSensor);
    total = reading + total;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}

void getColVals(float* vals) {

  // red 
  int times= 8;
  shineRed(); 
  delay(RGBWait);
  vals[0] = getColAvgReading(times);

  // green
  shineGreen(); 
  delay(RGBWait);
  vals[1] = getColAvgReading(times);

  // blue 
  shineBlue(); 
  delay(RGBWait);
  vals[2] = getColAvgReading(times);

  activateIR(); 

  return vals; 
}

void calibrateWhiteBlack() {
  
  //set black values
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000); 
  getColVals(blackArray); 

  //set white values
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);             //delay for five seconds for getting sample ready
  getColVals(whiteArray); 
  
  for (int i = 0; i <= 2; i++) {
    //the differnce between the maximum and the minimum gives the range
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }

}

void updateBaseline() {
  if (baselineArray[0] == 0) { // means that it's just been initialized 
    getColVals(baselineArray); 
  } else {
    // update moving average 
    getColVals(currArray); 
    for (int i=0; i<2; i++) {
      baselineArray[i] = baselineArray[i]*(1-baselineDecay) + currArray[i]*baselineDecay; 
    }
  }
}

void detectColour() {
  // POSSIBLE COLOURS: red, green, orange, pink, light blue 

  // the code has been prepared for both white/black method and baseline method 
  // white/black will need to calibrateWhiteBlack() at the start 
  // baseline method will need to call updateBaseline() function every once in a while 

  // current code is: baseline method 


  getColVals(currArray); 

  for (int c = 0; c <= 2; c++) {  //one colour at a time

    
    // if black white method: 
    currArray[c] =  ((currArray[c] - blackArray[c]) / (greyDiff[c]))*255 ;
    
     
    /*
    // if baseline method: 
    currArray[c] = currArray[c] - baselineArray[c]; 
    */
  }
}

void setup() {
  //begin serial communication
  Serial.begin(9600);

  
  pinMode(LightSensor, INPUT); // colour sensor 
  pinMode(A0, OUTPUT); 
  pinMode(A1, OUTPUT); 
  // port 3 S1: A2; port 3 S2: A3 
  // 3 is the one receiving the inputs (S1 is colour sensor, S2 is IR)
  // port 4 S1: A0; port 4 S2: A1 
  // 4 is the one with 2-to-4 decoder, all the outputs 

  /*
  // TEST COLOURS 
  Serial.println(0); // 
  digitalWrite(A0, LOW); 
  digitalWrite(A1, LOW);
  delay(6000); 

  Serial.println(1); // green 
  digitalWrite(A0, LOW); 
  digitalWrite(A1, HIGH);
  delay(6000); 

  Serial.println(2); // blue 
  digitalWrite(A0, HIGH); 
  digitalWrite(A1, LOW);
  delay(6000); 

  Serial.println(3); // red 
  digitalWrite(A0, HIGH); 
  digitalWrite(A1, HIGH);
  delay(6000); 
  */ 


  calibrateWhiteBlack(); //calibration

  Serial.println("BLACK:"); 
  Serial.print(blackArray[0]); 
  Serial.print(","); 
  Serial.print(blackArray[1]); 
  Serial.print(","); 
  Serial.println(blackArray[2]); 

  Serial.println("WHITE:"); 
  Serial.print(whiteArray[0]); 
  Serial.print(","); 
  Serial.print(whiteArray[1]); 
  Serial.print(","); 
  Serial.println(whiteArray[2]); 


  Serial.println("Put colour sample for detection... ");
}

int run_number=8;


void loop() {
  
  for (int i=0; i<2; i++) { //for each colour
    if (i==0){
      Serial.println( "MY_ORANGE");
    } else if (i==1){
      Serial.println( "MY_RED");
    }
    delay(8000);

    for (int j=0; j<8; j++) { //check 10 times
      //delay another 5 seconds to allow us to prepare the colour objects
      delay(4000);  //get ready the colour paper

      detectColour();
      for (int c=0; c<3; c++) { //for each result print all 3 colours
        Serial.print( currArray[c] );
        Serial.print(",");
      }
      if (i==0){
        Serial.print( "MY_ORANGE");
      } else if (i==1){
        Serial.print( "MY_RED");
      }
      Serial.print(",");
      Serial.println(run_number);
    }
  }
  Serial.println("END OF COLOUR DATA ===============================================");
}




