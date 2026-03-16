#include <MeMCore.h>
// #define TEST_COLOUR
// #define TEST_PID
// #define TEST_IR
// #define TEST_MOTOR
// #define CALIBRATEWB
// #define START_END

#ifdef START_END
unsigned long timetaken=0;
#endif

// DEFINING PINS
// port 3 S1: A2; port 3 S2: A3
// 3 is the one receiving the inputs (S1 is colour sensor, S2 is IR)
// port 4 S1: A0; port 4 S2: A1
// 4 is the one with 2-to-4 decoder, all the outputs

// NOTE: 8mA limit for LEDs
// red tends to want << 8mA ; adjust voltages so that
// except motor driver chip allows 30mA because IR sensor needs such high current

//LINE FOLLOWER ============================================================================================
MeLineFollower lineFinder(PORT_2);
int lineSensorState = 3;  //init to 3, will be overwritten anyway.
//0 is "both sensors see black", 1 is "left black", 2 is "right black", 3 is "no black"

//BUZZER ==================================================================================================
MeBuzzer buzzer;
const int beat = 1800;
const int quarter = beat/4;
const int huh = quarter/3;
const int eighth = quarter/2; 

void celebrate (){
    if (millis()%2==0){ //soviet anthem
        buzzer.tone(392, 395);

        buzzer.tone(523, 810);
        
        buzzer.tone(392, 395);
        buzzer.tone(440, 300);

        buzzer.tone(494, 810);

        buzzer.tone(330, 395);
        buzzer.tone(330, 300);

        buzzer.tone(440, 810);

        buzzer.tone(392, 395);
        buzzer.tone(349, 300);

        buzzer.tone(392, 810);

        buzzer.tone(262, 395);
        buzzer.tone(262, 300);

        buzzer.tone(294, 810);

        buzzer.tone(294, 395);
        buzzer.tone(330, 300);

        buzzer.tone(349, 810);

        buzzer.tone(349, 395);
        buzzer.tone(392, 375);

        buzzer.tone(440, 810);

        buzzer.tone(494, 395);
        buzzer.tone(523, 375);

        buzzer.tone(587, 810);
        delay(810);
        buzzer.tone(392, 395);

        buzzer.tone(659, 810);

        buzzer.tone(587, 395);
        buzzer.tone(523, 375);

        buzzer.tone(587, 810);

        buzzer.tone(494, 395);
        buzzer.tone(392, 375);

        buzzer.tone(523, 810);

        buzzer.tone(494, 395);
        buzzer.tone(440, 375);

        buzzer.tone(494, 810);

        buzzer.tone(330, 395);
        buzzer.tone(330, 375);

        buzzer.tone(440, 810);

        buzzer.tone(392, 395);
        buzzer.tone(349, 375);

        buzzer.tone(392, 810);

        buzzer.tone(262, 395);
        buzzer.tone(262, 375);

        buzzer.tone(523, 810);

        buzzer.tone(494, 395);
        buzzer.tone(440, 375);

        buzzer.tone(392, 810);
        delay(810);
    } else { //mario level complete
        //rest C4 E4 G4 C5 E5 G5 E5 rest
        delay(huh);
        buzzer.tone(262,huh);
        buzzer.tone(330,huh);
        buzzer.tone(392,huh);
        buzzer.tone(523,huh);
        buzzer.tone(659,huh);
        buzzer.tone(784,quarter);
        buzzer.tone(659,eighth);
        delay(eighth);
        
        //rest C#4 F4 G#4 C#5 F5 G#5 F5 REST
        delay(huh);
        buzzer.tone(262+15,huh);
        buzzer.tone(349,huh);
        buzzer.tone(392+20,huh);
        buzzer.tone(523+30,huh);
        buzzer.tone(698,huh);
        buzzer.tone(784+50,quarter);
        buzzer.tone(698,eighth);
        delay(eighth);
        
        //REST D#4 G4 A#4 D#5 G5 A#5 A#5 A#5 A#5 B#5
        delay(huh);
        buzzer.tone(294+15,huh);
        buzzer.tone(392,huh);
        buzzer.tone(440+20,huh);
        buzzer.tone(587+30,huh);
        buzzer.tone(794,huh);
        buzzer.tone(880+50,quarter);
        buzzer.tone(880+50,huh);
        buzzer.tone(880+50,huh);
        buzzer.tone(880+50,huh);

        buzzer.tone(1030,beat);
    }
}

//MOTOR_BASIC =============================================================================================
MeDCMotor leftMotor(M1);   // assigning leftMotor to port M1
MeDCMotor rightMotor(M2);  // assigning RightMotor to port M2

#define motorSpeed 175
#define turnSpeed 200
#define TURNING_TIME_MS 380
#define turnDelay 950

enum { LEFT_TURN,
       RIGHT_TURN };

void stopMotor() {    // Code for stopping motors
  leftMotor.stop();   // Stop left motor
  rightMotor.stop();  // Stop right motor
}

void turn(bool direction) {       //turn(0) for left, 1 for right
  if (direction == LEFT_TURN) {   //turn left
    leftMotor.run(turnSpeed);    // Positive: wheel turns anti-clockwise
    rightMotor.run(turnSpeed);   // Positive: wheel turns anti-clockwise
  } else {                        //turn right
    leftMotor.run(-turnSpeed);   // Negative: wheel turns clockwise
    rightMotor.run(-turnSpeed);  // Negative: wheel turns clockwise
  }
  delay(TURNING_TIME_MS);  // Keep turning left for this time duration
  stopMotor();             //can comment out for extra efficiency or smth
}

void moveForward() {           // Code for moving forward for some short interval
  leftMotor.run(-motorSpeed);  // Negative: wheel turns anti-clockwise
  rightMotor.run(motorSpeed);  // Positive: wheel turns clockwise
  delay(turnDelay);
}

void uTurn() {  // Code for u-turn (more delay time code for turnleft)
  turn(RIGHT_TURN);
  turn(RIGHT_TURN);
}

void doubleTurn(bool direction) {  // Code for double turn in given direction
  turn(direction);
  moveForward();
  turn(direction);
}

void speedCorrection(float correction) {  //-ve is go right(increase left speed, decrease right speed), +ve is go left (decrease left speed, increase right speed)
  float left_speed = constrain(motorSpeed + correction, 0, 255);
  float right_speed = constrain(motorSpeed - correction, 0, 255);

  leftMotor.run(-left_speed);
  rightMotor.run(right_speed);
}

//2 BY 4 DECODER ==========================================================================================
void activateIR() {  // Code for turning on the IR only
  digitalWrite(A0, LOW);
  digitalWrite(A1, LOW);
}

void shineRed() {  // Code for turning on the red LED only
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
}
void shineGreen() {  // Code for turning on the green LED only
  // green
  digitalWrite(A0, LOW);
  digitalWrite(A1, HIGH);
}
void shineBlue() {  // Code for turning on the blue LED only
  digitalWrite(A0, HIGH);
  digitalWrite(A1, LOW);
}

//IR SENSOR ===============================================================================================
#define IRSensor A3
#define IRwait 10  //in milliseconds

float starting_voltage = 0.0;

int getIRAvgReading(int times) {  //find the average reading for the requested number of times of scanning LDR
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++) {
    reading = analogRead(IRSensor);
    total = reading + total;
    delay(IRwait);
  }
  //calculate the average and return it
  return total / times;
}

bool IRResult(float *starting_voltage) {  //code for getting distance from wall
  activateIR();
  return getIRAvgReading(5) < *starting_voltage;
  //resulting IR signal will be sent thru pin A3
  //use this result to calculate distance
}

//ULTRASONIC SENSOR =======================================================================================
#define ultrasonicPin 12  // port 1 is pin 12; both input and ouptut

int ultrasonicPing() {  // returns in cm
  long duration;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(ultrasonicPin, OUTPUT);  //SET AS OUTPUT
  digitalWrite(ultrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasonicPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultrasonicPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(ultrasonicPin, INPUT);  //SET AS INPUT
  duration = pulseIn(ultrasonicPin, HIGH);

  // convert the time into a distance
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return duration / 29 / 2;
}

//COLOUR SENSOR CALIBRATION ===============================================================================
#define LightSensor A2

#define LDRWait 10  //in milliseconds
#define RGBWait 100
#define baselineDecay 0.1

float currArray[] = { 0, 0, 0 };
float whiteArray[] = { 731.00,576.00,426.00 };  // not initialized to 0 but doesn't matter as it'll be rewritten anyways
float blackArray[] = { 829.00,812.00,730.00 };
float greyDiff[] = { whiteArray[0]-blackArray[0], whiteArray[1]-blackArray[1], whiteArray[2]-blackArray[2] };
float baselineArray[] = { 0, 0, 0 };
char colourStr[3][6] = { "Red", "Green", "Blue" };

int getColAvgReading(int times) {  //get average LDR reading for the requested number of times of scanning LDR
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

void getColVals(float *vals) {  //shine each LED and get avg reading. modifies 'vals' arr
  // red
  shineRed();
  delay(RGBWait);
  vals[0] = getColAvgReading(5);

  // green
  shineGreen();
  delay(RGBWait);
  vals[1] = getColAvgReading(5);

  // blue
  shineBlue();
  delay(RGBWait);
  vals[2] = getColAvgReading(5);

  activateIR();
}

// currently i'm using calibrateWhiteBlack
void calibrateWhiteBlack() {
  //set white values
  Serial.println("Put White Sample For Calibration ...");
  delay(5000);  //delay for five seconds for getting sample ready
  getColVals(whiteArray);

  //done scanning white, time for the black sample.

  //set black values
  Serial.println("Put Black Sample For Calibration ...");
  delay(5000);
  getColVals(blackArray);

  for (int i = 0; i <= 2; i++) {
    //the differnce between the maximum and the minimum gives the range
    greyDiff[i] = whiteArray[i] - blackArray[i];
  }
}

// curently not using baseline method
void updateBaseline() {
  if (baselineArray[0] == 0) {  // means that it's just been initialized
    getColVals(baselineArray);
  } else {
    // update moving average
    getColVals(currArray);
    for (int i = 0; i < 2; i++) {
      baselineArray[i] = baselineArray[i] * (1 - baselineDecay) + currArray[i] * (baselineDecay);
    }
  }
}

void subtractBaseline() {
  for (int c = 0; c <= 2; c++) {  //Iterate thru each colour
    currArray[c] = currArray[c] - baselineArray[c];
  }
}

void normaliseBlackWhite() {
  for (int c = 0; c <= 2; c++) {  //Iterate thru each colour
    currArray[c] = ((currArray[c] - blackArray[c]) / (greyDiff[c]))*255;

  }
}

//COLOUR DETECTION ========================================================================================
enum { MY_BLUE,
   MY_GREEN,
   MY_ORANGE,
   MY_PINK,
   MY_RED,
   MY_WHITE };

void convert_to_colour(int result) {
  switch (result) {
    case MY_RED :
      Serial.println("Red"); break;
    case MY_GREEN :
      Serial.println("Green"); break;
    case MY_BLUE :
      Serial.println("Blue"); break;
    case MY_ORANGE:
      Serial.println("Orange"); break;
    case MY_PINK :
      Serial.println("Pink"); break;
    case MY_WHITE :
      Serial.println("White"); break;
    
  }
}

int col_nc = 6; 
float col_coef[6][3] = {{-0.01860725007411738, -0.0423876775262573, 0.04382935983498103}, {-0.021929079978621712, 0.029930121784642806, -0.014736629055868703}, {0.006396213470742044, 0.15883031141302487, -0.24320431412357305}, {0.004367613836646749, -0.008712323832029446, 0.008990814253786292}, {0.03770202752147757, -0.1736261780399539, 0.0535618818162405}, {-0.0008179248743882601, 0.009811948893584945, -0.0015295663264269465}};
float col_intc[6] = { -0.0006461633776652192, 0.6787724983990242, -0.10079448680457566, -1.6424735747175052, 0.0006955256789669058, -1.683075801006096 };
int col_predict(float r, float g, float b) {
    float allvs[6];
    for (int i=0; i<col_nc; i++) {
        allvs[i] = col_intc[i];
    }

    float c; 
    for (int cidx=0; cidx<3; cidx++) {
        if (cidx==0) c=r;
        else if (cidx==1) c=g;
        else c=b;
        for (int i=0; i<col_nc; i++){
            allvs[i] += col_coef[i][cidx]*c;
        }
    }

    // get index of max
    int maxidx = 0; int maxsofar = allvs[0];
    for (int i=1; i<col_nc; i++) {
        if (allvs[i] > maxsofar) {
            maxidx = i;
            maxsofar = allvs[i];
        }
    }
    return maxidx;
}

int detectColour() {
  // POSSIBLE COLOURS: red, green, blue, orange, pink -> 0 1 2 3 4
  // the code has been prepared for both white/black method and baseline method
  // white/black will need to calibrateWhiteBlack() at the start
  // baseline method will need to call updateBaseline() function every once in a while
  // current code is: blackwhite method

  //get current values
  getColVals(currArray);  //get curr colour values. currArray is declared near the start
  //process
  //subtractBaseline();
  normaliseBlackWhite();

  //predict
  int result = col_predict(currArray[0], currArray[1], currArray[2]);

  #ifdef TEST_COLOUR //print outputs
  Serial.print("Predicted colour: ");  
  convert_to_colour(result);
  Serial.println("VALS:"); 
  Serial.print(currArray[0]); 
  Serial.print(","); 
  Serial.print(currArray[1]); 
  Serial.print(","); 
  Serial.println(currArray[2]);
  #endif

  return result;
}

//SETUP+LOOP ==============================================================================================
// EXTRA VARIABLES
bool running = false;

//track time since last sensor check
unsigned long black_line_update = 0;
unsigned long PID_update = 0;  //used for PID

//PID variables
float Kp = 20.0;  //scale immediate response
float Kd = 16.0;  //predicts future error direction, reduces overshoot
float target_dist = 9.0;
//base speed is defined in motor.h as 255
float max_correction = 70;
float previous_error = 0;
bool right_too_close = false;
float PID_interval = 30;
bool has_wall=false;
float prev_correction = 0.0;

//copy paste from here onwards
void setup() {
  Serial.begin(9600);

  pinMode(LightSensor, INPUT);  //colour sensor
  pinMode(IRSensor, INPUT);     //IR sensor
  pinMode(A0, OUTPUT);          //for 2 to 4 decoder
  pinMode(A1, OUTPUT);
  pinMode(A7, INPUT);  // push button

  // IR init
  activateIR();

  #ifdef TEST_COLOUR
  // colour init
  #ifdef CALIBRATEWB
  calibrateWhiteBlack();
  #endif

  Serial.println("WHITE:"); 
  Serial.print(whiteArray[0]); 
  Serial.print(","); 
  Serial.print(whiteArray[1]); 
  Serial.print(","); 
  Serial.println(whiteArray[2]); 

  Serial.println("BLACK:"); 
  Serial.print(blackArray[0]); 
  Serial.print(","); 
  Serial.print(blackArray[1]); 
  Serial.print(","); 
  Serial.println(blackArray[2]); 

  int test;
  for (int j=0; j<20; j++) { //check 10 times
      //delay another 5 seconds to allow us to prepare the colour objects
      Serial.println("Put colour to detect...");
      delay(5000);  //get ready the colour paper

      test=detectColour();
    }
  #endif
  #ifdef TEST_MOTOR
  turn(LEFT_TURN);
  delay(7000);
  doubleTurn(LEFT_TURN);
  delay(7000);
  uTurn();
  #endif
}

void loop() {
  // if the analogRead value of the button is very low than start the robot (can do toggle switch)
  if (analogRead(A7) < 150) {
    #ifdef START_END
    timetaken=millis();
    #endif
    while (analogRead(A7) < 150) {
      delay(1); 
    }
    running = !running;
    moveForward();
    stopMotor();
    starting_voltage = getIRAvgReading(10);  //take the starting value to be the threshold 
    // this is calibrating IR
    #ifdef TEST_IR
    Serial.print("IR CALIBRATION "); 
    Serial.println(starting_voltage); 
    #endif
  }

  if (!running) continue;

  //PID =======================================================
  if ((millis() - PID_update) >= PID_interval) {  //check every 20ms
    float dt = (PID_update - millis()) / 1000.00;
    PID_update = millis(); //update time
    float dist = ultrasonicPing();
    float error = target_dist - dist;  //-ve if too far left, +ve if too far right

    has_wall = (error < 6 && error > -10 ) ; //error is relatively small. there is indeed a wall on the left side.

    if (has_wall) {
      float derivative = (error - previous_error) / PID_interval;
      float correction = (Kp * error) + (Kd * derivative);
      correction = constrain(correction, -max_correction, max_correction);

      #ifdef TEST_PID
      Serial.print(error);
      Serial.print(" ");
      Serial.print(derivative);
      Serial.print(" ");
      Serial.print(correction);
      Serial.print(" ");
      if (correction<0.00){
        Serial.println("Go left");
      } else {
        Serial.println("Go right");
      }
      #endif

      // if too close, adjust right. correction is negative
      // if too far, adjust left. correction is positive
      speedCorrection(correction);  //-ve is go right, +ve is go left
      prev_correction=correction;
      previous_error = error;

    } else {
      #ifdef TEST_PID
      Serial.println("Left side no wall !!!!!!!!!!!!!!!!!!!!");
      #endif
      speedCorrection(-prev_correction);
      speedCorrection(-prev_correction);
      prev_correction=0;
      right_too_close = IRResult(&starting_voltage);
      #ifdef TEST_IR
      Serial.print("IR RESULT ");
      Serial.println(right_too_close);
      #endif
      if (right_too_close) {  //if error is very large, assume that the left side is open. check if right wall exists.
        //only case when IR is needed is when there's a "right wall only" section.
        speedCorrection(-40);  //nudge left by 40 (arbitrary value)
      }
    }
  }

  //CHECK LINE + COLOUR (every 250ms)======================================
  if ((millis() - black_line_update) >= 15) {  //after this runs, the bot will be stationary after completing an action.
    // if detected black line, stop motor, detect colour, and take corresponding action
    lineSensorState = lineFinder.readSensors();
    black_line_update = millis();
    if (lineSensorState != S1_OUT_S2_OUT) {

      #ifdef TEST_PID
      Serial.println("DETECTED BLACK LINE");  // detected black line
      #endif

      stopMotor();
      switch (detectColour()) {
        case MY_RED:
          turn(LEFT_TURN); break;
        case MY_GREEN:
          turn(RIGHT_TURN); break;
        case MY_ORANGE:
          uTurn(); break;
        case MY_PINK:
          doubleTurn(LEFT_TURN); break;
        case MY_BLUE:
          doubleTurn(RIGHT_TURN); break;
        case MY_WHITE:
          #ifdef START_END
          timetaken=millis()-timetaken;
          Serial.println(timetaken/1000);
          #endif
          celebrate();         //play yippee sound
          running = !running;  //stop running
          break;
      }
    }
  }
  
}
