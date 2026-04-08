/*
 * sensor_miniproject_template.ino
 * Studio 13: Sensor Mini-Project
 *
 * This sketch is split across three files in this folder:
 *
 *   packets.h        - TPacket protocol: enums, struct, framing constants.
 *                      Must stay in sync with pi_sensor.py.
 *
 *   serial_driver.h  - Transport layer.  Set USE_BAREMETAL_SERIAL to 0
 *                      (default) for the Arduino Serial path that works
 *                      immediately, or to 1 to use the bare-metal USART
 *                      driver (Activity 1).  Also contains the
 *                      sendFrame / receiveFrame framing code.
 *
 *   sensor_miniproject_template.ino  (this file)
 *                    - Application logic: packet helpers, E-Stop state
 *                      machine, color sensor, setup(), and loop().
 */

#include "packets.h"
#include "serial_driver.h"
#include <AFMotor.h>
#include <Servo.h>
#include <util/delay.h>
typedef enum dir
{
  STOP,
  GO,
  BACK,
  CCW,
  CW
} dir;

#define FRONT_LEFT   4
#define FRONT_RIGHT  1
#define BACK_LEFT    3
#define BACK_RIGHT   2  

volatile uint16_t debounce_ms = 0; 
volatile uint16_t color_ms = 0; 

AF_DCMotor motorFL(FRONT_LEFT);
AF_DCMotor motorFR(FRONT_RIGHT);
AF_DCMotor motorBL(BACK_LEFT);
AF_DCMotor motorBR(BACK_RIGHT);

struct ColorRef {
  float R;
  float G;
  float B;
};

ColorRef RED_REF   = {5028/6000.0, 1702/5000.0, 8192/13000.0};
ColorRef GREEN_REF = {3312/6000.0, 3918/5000.0, 9656/13000.0};
ColorRef BLUE_REF  = {2714/6000.0, 4142/5000.0, 12100/13000.0};

int motorSpeed = 150;
//uint16_t lastDebounceTime = 0; //debounce variable
bool currentStatus = true;     //helper variable for INT5 ISR

// =============================================================
// Robot arm (Timer-safe integration)
// Uses Arduino Servo library to avoid direct register/timer clashes
// with existing Timer2 debounce/color timing and other modules.
// On Mega, use analog pins A9-A12 for servo signal wires.
// =============================================================

#define ARM_BASE_PIN      A9
#define ARM_SHOULDER_PIN  A10
#define ARM_ELBOW_PIN     A11
#define ARM_GRIPPER_PIN   A12

#define ARM_BASE_MIN      0
#define ARM_BASE_MAX      180
#define ARM_SHOULDER_MIN  70
#define ARM_SHOULDER_MAX  120
#define ARM_ELBOW_MIN     60
#define ARM_ELBOW_MAX     120
#define ARM_GRIPPER_MIN   70
#define ARM_GRIPPER_MAX   100
#define ARM_SPEED_MIN     1
#define ARM_SPEED_MAX     50

typedef enum {
  ARM_JOINT_BASE = 0,
  ARM_JOINT_SHOULDER = 1,
  ARM_JOINT_ELBOW = 2,
  ARM_JOINT_GRIPPER = 3,
  ARM_JOINT_COUNT = 4
} ArmJoint;

static Servo armServos[ARM_JOINT_COUNT];
static const uint8_t armPins[ARM_JOINT_COUNT] = {
  ARM_BASE_PIN, ARM_SHOULDER_PIN, ARM_ELBOW_PIN, ARM_GRIPPER_PIN
};
static uint8_t armPos[ARM_JOINT_COUNT] = {90, 90, 90, 90};
static uint8_t armTarget[ARM_JOINT_COUNT] = {90, 90, 90, 90};
static uint8_t armMsPerDeg = 10;
static uint32_t armLastUpdate[ARM_JOINT_COUNT] = {0, 0, 0, 0};
// =============================================================
// Packet helpers (pre-implemented for you)
// =============================================================

/*
 * Build a zero-initialised TPacket, set packetType = PACKET_TYPE_RESPONSE,
 * command = resp, and params[0] = param.  Then call sendFrame().
 */
static void sendResponse(TResponseType resp, uint32_t param) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command    = resp;
    pkt.params[0]  = param;
    sendFrame(&pkt);
}

/*
 * Send a RESP_STATUS packet with the current state in params[0].
 */
static void sendStatus(TState state) {
    sendResponse(RESP_STATUS, (uint32_t)state);
}
static void sendOk() {
    sendResponse(RESP_OK, 0);
}

static void sendMotorStatus(uint32_t speed) {
    sendResponse(RESP_MOTOR_STATUS, speed);
}

static void sendArmStatus() {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command = RESP_ARM_STATUS;
  // params[0..3] are actual servo positions, params[5..8] are targets.
  pkt.params[0] = armPos[ARM_JOINT_BASE];
  pkt.params[1] = armPos[ARM_JOINT_SHOULDER];
  pkt.params[2] = armPos[ARM_JOINT_ELBOW];
  pkt.params[3] = armPos[ARM_JOINT_GRIPPER];
    pkt.params[4] = armMsPerDeg;
  pkt.params[5] = armTarget[ARM_JOINT_BASE];
  pkt.params[6] = armTarget[ARM_JOINT_SHOULDER];
  pkt.params[7] = armTarget[ARM_JOINT_ELBOW];
  pkt.params[8] = armTarget[ARM_JOINT_GRIPPER];
    sendFrame(&pkt);
}

static void sendOkWithMsg(const char *msg) {
    TPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.packetType = PACKET_TYPE_RESPONSE;
    pkt.command = RESP_OK;
    if (msg != NULL) {
      strncpy(pkt.data, msg, sizeof(pkt.data) - 1);
      pkt.data[sizeof(pkt.data) - 1] = '\0';
    }
    sendFrame(&pkt);
}

static void armInit() {
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    armServos[i].attach(armPins[i]);
    armServos[i].write(armPos[i]);
  }
}

static bool armApplyTarget(uint8_t joint, uint32_t value, uint8_t minV, uint8_t maxV) {
  if (value < minV || value > maxV) {
    sendOkWithMsg("ERR_RANGE");
    return false;
  }
  armTarget[joint] = (uint8_t)value;
  // Debug mode: apply the command immediately to isolate motion-loop issues
  // from physical servo channel issues.
  armPos[joint] = armTarget[joint];
  armServos[joint].write(armPos[joint]);
  armLastUpdate[joint] = millis();
  return true;
}

static void armHome() {
  armTarget[ARM_JOINT_BASE] = 90;
  armTarget[ARM_JOINT_SHOULDER] = 90;
  armTarget[ARM_JOINT_ELBOW] = 90;
  armTarget[ARM_JOINT_GRIPPER] = 90;
}

static void armUpdateMotion() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    if (armPos[i] == armTarget[i]) continue;
    if ((now - armLastUpdate[i]) < armMsPerDeg) continue;
    armLastUpdate[i] = now;
    if (armPos[i] < armTarget[i]) armPos[i]++;
    else armPos[i]--;
    armServos[i].write(armPos[i]);
  }
}

static void handleArmCommand(const TPacket *cmd) {
  const uint32_t value = cmd->params[0];
  switch (cmd->command) {
    case COMMAND_ARM_BASE:
      if (armApplyTarget(ARM_JOINT_BASE, value, ARM_BASE_MIN, ARM_BASE_MAX)) {
        sendOkWithMsg("BASE_OK");
        sendArmStatus();
      }
      break;
    case COMMAND_ARM_SHOULDER:
      if (armApplyTarget(ARM_JOINT_SHOULDER, value, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX)) {
        sendOkWithMsg("SHLDR_OK");
        sendArmStatus();
      }
      break;
    case COMMAND_ARM_ELBOW:
      if (armApplyTarget(ARM_JOINT_ELBOW, value, ARM_ELBOW_MIN, ARM_ELBOW_MAX)) {
        sendOkWithMsg("ELBOW_OK");
        sendArmStatus();
      }
      break;
    case COMMAND_ARM_GRIPPER:
      if (armApplyTarget(ARM_JOINT_GRIPPER, value, ARM_GRIPPER_MIN, ARM_GRIPPER_MAX)) {
        sendOkWithMsg("GRIP_OK");
        sendArmStatus();
      }
      break;
    case COMMAND_ARM_HOME:
      armHome();
      sendOkWithMsg("HOME_OK");
      sendArmStatus();
      break;
    case COMMAND_ARM_SET_SPEED:
      if (value < ARM_SPEED_MIN || value > ARM_SPEED_MAX) {
        sendOkWithMsg("ERR_SPEED");
      } else {
        armMsPerDeg = (uint8_t)value;
        sendOkWithMsg("SPEED_OK");
        sendArmStatus();
      }
      break;
    default:
      break;
  }
}
// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;
volatile bool   motorShutdownRequested = false;

static TState readButtonState() {
    cli();
    TState state = buttonState;
    sei();
    return state;
}

static void requestMotorShutdown() {
    motorShutdownRequested = true;
}

/*
 * TODO (Activity 1): Implement the E-Stop ISR.
 *
 * Fire on any logical change on the button pin.
 * State machine (see handout diagram):
 *   RUNNING + press (pin HIGH)  ->  STOPPED, set stateChanged = true
 *   STOPPED + release (pin LOW) ->  RUNNING, set stateChanged = true
 *
 * Debounce the button.  You will also need to enable this interrupt
 * in setup() -- check the ATMega2560 datasheet for the correct
 * registers for your chosen pin.
 */
 ISR(INT3_vect) { // this resets timer 2 ticks 

    if (debounce_ms > 20) { // 20 ms 
      debounce_ms = 0; 

      int state = (PIND & (1 << 3));

      if(state && buttonState == STATE_RUNNING && currentStatus == true) {
        buttonState = STATE_STOPPED;
        stateChanged = true;
        requestMotorShutdown();
      }
      else if (!state && buttonState == STATE_STOPPED && currentStatus == true) {
        currentStatus = false;
      }
      else if(!state && buttonState == STATE_STOPPED && currentStatus == false) {
        buttonState = STATE_RUNNING;
        stateChanged = true;
        currentStatus = true;
      }
    }
  }

// =============================================================
// Color sensor (TCS3200)
// =============================================================

// =============================================================
// Color sensor (TCS3200)
// =============================================================

// Pin mapping using your current bare-metal ports:
// PK0 -> sensor OUT
// PJ0 -> S0
// PJ1 -> S1
// PH0 -> S2
// PH1 -> S3

static void INIT_COLOR_SENSOR() {
    // PK0 = OUT from TCS3200 -> input
    DDRK &= ~(1 << DDK0);

    // PJ0 = S0, PJ1 = S1 -> outputs
    DDRJ |= (1 << DDJ0) | (1 << DDJ1);

    // PH0 = S2, PH1 = S3 -> outputs
    DDRH |= (1 << DDH0) | (1 << DDH1);

    // 20% output scaling: S1 = HIGH, S0 = LOW
    PORTJ |=  (1 << PJ1);   // S1 high
    PORTJ &= ~(1 << PJ0);   // S0 low

    // init timer 2 
}

static void selectRed() {
    // S2 = LOW, S3 = LOW
    PORTH &= ~(1 << PH0);
    PORTH &= ~(1 << PH1);
}

static void selectBlue() {
    // S2 = LOW, S3 = HIGH
    PORTH &= ~(1 << PH0);
    PORTH |=  (1 << PH1);
}

static void selectGreen() {
    // S2 = HIGH, S3 = HIGH
    PORTH |=  (1 << PH0);
    PORTH |=  (1 << PH1);
}

static uint32_t measureChannel100ms() {
    uint32_t count = 0;

    // OUT is on PD0
    uint8_t lastState = (PINK & (1 << PINK0));

    // can set color_ms to a value>250 like 300 to avoid integer overflow 
    color_ms = 0;

    // 100 ms 
    while (color_ms < 100) {
        uint8_t nowState = (PINK & (1 << PINK0));

        // count rising edge
        if(!lastState && nowState) {
            count++;
        }

        lastState = nowState;
    }

    return count;
}

float colorDistance(float r1, float g1, float b1, ColorRef ref) {
  float dr = r1 - ref.R;
  float dg = g1 - ref.G;
  float db = b1 - ref.B;
  return dr * dr + dg * dg + db * db; // no sqrt needed
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b, uint32_t *c) {
    selectRed();
    _delay_ms(5);
    *r = measureChannel100ms() * 10;   // Hz

    selectGreen();
    _delay_ms(5);
    *g = measureChannel100ms() * 10;   // Hz

    selectBlue();
    _delay_ms(5);
    *b = measureChannel100ms() * 10;   // Hz

    float rN = *r / 6000.0;
    float gN = *g / 5000.0;
    float bN = *b / 13000.0;

    float dRed   = colorDistance(rN, gN, bN, RED_REF);
    float dGreen = colorDistance(rN, gN, bN, GREEN_REF);
    float dBlue  = colorDistance(rN, gN, bN, BLUE_REF);

    if (dRed < dGreen && dRed < dBlue) {
        *c = 0;
    } else if (dGreen < dRed && dGreen < dBlue) {
        *c = 1;
    } else {
        *c = 2;
    }
}
// =============================================================
// motor
// =============================================================
void move(int speed, int direction)
{
  motorFL.setSpeed(speed);
  motorFR.setSpeed(speed);
  motorBL.setSpeed(speed);
  motorBR.setSpeed(speed);

  switch(direction)
  {
    case BACK:
      motorFL.run(BACKWARD);
      motorFR.run(BACKWARD);
      motorBL.run(FORWARD);
      motorBR.run(FORWARD);
      break;
    case GO:
      motorFL.run(FORWARD);
      motorFR.run(FORWARD);
      motorBL.run(BACKWARD);
      motorBR.run(BACKWARD);
      break;
    case CW:
      motorFL.run(BACKWARD);
      motorFR.run(FORWARD);
      motorBL.run(FORWARD);
      motorBR.run(BACKWARD);
      break;
    case CCW:
      motorFL.run(FORWARD);
      motorFR.run(BACKWARD);
      motorBL.run(BACKWARD);
      motorBR.run(FORWARD);
      break;
    case STOP:
    default:
      motorFL.run(RELEASE);
      motorFR.run(RELEASE);
      motorBL.run(RELEASE);
      motorBR.run(RELEASE);
      break;
  }
}

void forward(int speed)  { move(speed, GO); }
void backward(int speed) { move(speed, BACK); }
void ccw(int speed)      { move(speed, CCW); }
void cw(int speed)       { move(speed, CW); }
void stop()              { move(0, STOP); }

static void haltArmMotion() {
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    armTarget[i] = armPos[i];
  }
}

static void stopAllMotionNow() {
  stop();
  haltArmMotion();
  motorShutdownRequested = false;
}

static bool commandIsBlockedWhenStopped(uint8_t command) {
  switch (command) {
    case COMMAND_FORWARD:
    case COMMAND_BACKWARD:
    case COMMAND_LEFT:
    case COMMAND_RIGHT:
    case COMMAND_ARM_BASE:
    case COMMAND_ARM_SHOULDER:
    case COMMAND_ARM_ELBOW:
    case COMMAND_ARM_GRIPPER:
    case COMMAND_ARM_HOME:
      return true;
    default:
      return false;
  }
}


// =============================================================
// Command handler
// =============================================================

/*
 * Dispatch incoming commands from the Pi.
 *
 * COMMAND_ESTOP is pre-implemented: it sets the Arduino to STATE_STOPPED
 * and sends back RESP_OK followed by a RESP_STATUS update.
 *
 * TODO (Activity 2): add a case for your color sensor command.
 *   Call your color-reading function, then send a response packet with
 *   the channel frequencies in Hz.
 */
static void handleCommand(const TPacket *cmd) {
    if (cmd->packetType != PACKET_TYPE_COMMAND) return;

    if (readButtonState() == STATE_STOPPED && commandIsBlockedWhenStopped(cmd->command)) {
        requestMotorShutdown();
        stopAllMotionNow();
        sendStatus(STATE_STOPPED);
        sendMotorStatus(motorSpeed);
        sendArmStatus();
        return;
    }

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            requestMotorShutdown();
            stopAllMotionNow();
            sendOk();
            sendStatus(STATE_STOPPED);
            sendMotorStatus(motorSpeed);
            sendArmStatus();
            break;

        case COMMAND_COLOR_SENSOR: {
            uint32_t r, g, b, c;
            readColorChannels(&r, &g, &b, &c);

            TPacket pkt;
            memset(&pkt, 0, sizeof(pkt));
            pkt.packetType = PACKET_TYPE_RESPONSE;
            pkt.command    = RESP_COLOR_SENSOR;
            pkt.params[0]  = r;
            pkt.params[1]  = g;
            pkt.params[2]  = b;
            pkt.params[3]  = c;
            sendFrame(&pkt);
            break;
        }

        case COMMAND_FORWARD:
            forward(motorSpeed);
            sendMotorStatus(motorSpeed);
            break;

        case COMMAND_BACKWARD:
            backward(motorSpeed);
            sendMotorStatus(motorSpeed);
            break;

        case COMMAND_LEFT:
            ccw(motorSpeed);
            sendMotorStatus(motorSpeed);
            break;

        case COMMAND_RIGHT:
            cw(motorSpeed);
            sendMotorStatus(motorSpeed);
            break;

        case COMMAND_STOP:
            stopAllMotionNow();
            sendMotorStatus(motorSpeed);
            sendArmStatus();
            break;

        case COMMAND_SET_SPEED: {
            uint32_t newSpeed = cmd->params[0];
            if (newSpeed > 255) newSpeed = 255;
            motorSpeed = (int)newSpeed;
            sendMotorStatus(motorSpeed);
            break;
        }

        case COMMAND_ARM_BASE:
        case COMMAND_ARM_SHOULDER:
        case COMMAND_ARM_ELBOW:
        case COMMAND_ARM_GRIPPER:
        case COMMAND_ARM_HOME:
        case COMMAND_ARM_SET_SPEED:
            handleArmCommand(cmd);
            break;

        default:
            sendOk();
            break;
    }
}
// =============================================================
// Arduino setup() and loop()
// =============================================================

void setup() {
    // Initialise the serial link at 9600 baud.
    // Serial.begin() is used by default; usartInit() takes over once
    // USE_BAREMETAL_SERIAL is set to 1 in serial_driver.h.
#if USE_BAREMETAL_SERIAL
    usartInit(103);   // 9600 baud at 16 MHz
#else
    Serial.begin(9600);
#endif

    // TODO (Activity 1): configure the button pin and its external interrupt,
    // then call sei() to enable global interrupts.
    cli();

    EICRA = 0b01000000;
    //EICRB = 0b00000100;
    //EIMSK = 0b00100000;
    EIMSK = 0b00001000; 
    //DDRE &= ~(1 << 5);
    DDRD &= ~(1<<3); 


    // init timer 2 counter used for all 
    // CTC WGM: 010, only WGM21 is set 
    TCCR2A = (1 << WGM21); 
    // prescaler 128, TOP 125, gives 1ms per tick 
    // prescaler 128 is 101 
    OCR2A = 125; 
    // enable interrupt timer2A 
    TIMSK2 = 0b10; 


    

    // TO DO
    // SET PD0 AS AN INTERRUPT WITH RISING EDGE
    // COMMAND_COLOUR_SENSOR RESP_COLOUR
    
    // Color Sensor
    INIT_COLOR_SENSOR();
    armInit();
    sendArmStatus();


    
  // TODO (Activity 3a): Enable the button to fire an interrupt on any
    // logical change (both rising and falling edges).
    sei();
    TCNT2 = 0; 
    TCCR2B = 0b101; // start timer 2 ticking 
}

ISR(TIMER2_COMPA_vect) { // this'll automatically reset timer2 
    debounce_ms ++; 
    color_ms ++; 
}

void loop() {
    if (motorShutdownRequested) {
        stopAllMotionNow();
    }

    armUpdateMotion();

    // --- 1. Report any E-Stop state change to the Pi ---
    if (stateChanged) {
        cli();
        TState state = buttonState;
        stateChanged = false;
        sei();
        sendStatus(state);
    }

    // --- 2. Process incoming commands from the Pi ---
    TPacket incoming;
    bool frameOk = receiveFrame(&incoming);
    if (frameOk) {
        handleCommand(&incoming);
    }
}