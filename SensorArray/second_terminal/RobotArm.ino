#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>
#include <stdint.h>

#define NUM_SERVOS 4

enum ServoEnum { BASE, SHOULDER, ELBOW, GRIPPER };
volatile ServoEnum currServo = BASE;

volatile uint8_t pos[NUM_SERVOS] = {90, 90, 90, 90};
volatile uint8_t target[NUM_SERVOS] = {90, 90, 90, 90};
volatile uint16_t pulseTicks[NUM_SERVOS];

volatile uint8_t msPerDeg = 10;
unsigned long lastUpdate[NUM_SERVOS] = {0, 0, 0, 0};

#define SERVO_MASK ((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3))

// TPacket protocol
typedef enum {
  PACKET_TYPE_COMMAND = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE = 2
} TPacketType;

typedef enum {
  COMMAND_ESTOP = 0,
  COMMAND_COLOR_SENSOR = 1,
  COMMAND_FORWARD = 2,
  COMMAND_BACKWARD = 3,
  COMMAND_LEFT = 4,
  COMMAND_RIGHT = 5,
  COMMAND_SET_SPEED = 6,
  COMMAND_STOP = 7,
  COMMAND_ARM_BASE = 20,
  COMMAND_ARM_SHOULDER = 21,
  COMMAND_ARM_ELBOW = 22,
  COMMAND_ARM_GRIPPER = 23,
  COMMAND_ARM_HOME = 24,
  COMMAND_ARM_SET_SPEED = 25
} TCommandType;

typedef enum {
  RESP_OK = 0,
  RESP_STATUS = 1,
  RESP_COLOR_SENSOR = 2,
  RESP_MOTOR_STATUS = 3,
  RESP_ARM_STATUS = 4
} TResponseType;

typedef struct {
  uint8_t packetType;
  uint8_t command;
  uint8_t dummy[2];
  char data[32];
  uint32_t params[16];
} TPacket;

#define MAGIC_HI 0xDE
#define MAGIC_LO 0xAD
#define TPACKET_SIZE ((uint8_t)sizeof(TPacket))
#define FRAME_SIZE (2 + TPACKET_SIZE + 1)

#define ARM_BASE_MIN 0
#define ARM_BASE_MAX 180
#define ARM_SHOULDER_MIN 70
#define ARM_SHOULDER_MAX 120
#define ARM_ELBOW_MIN 60
#define ARM_ELBOW_MAX 120
#define ARM_GRIPPER_MIN 70
#define ARM_GRIPPER_MAX 100
#define ARM_SPEED_MIN 1
#define ARM_SPEED_MAX 50

uint16_t degToTicks(uint8_t deg) {
  long us = map(deg, 0, 180, 500, 2500);
  return us / 4;
}

void updateMotion() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    if (pos[i] == target[i]) continue;
    if (now - lastUpdate[i] < msPerDeg) continue;
    lastUpdate[i] = now;
    if (pos[i] < target[i]) pos[i]++;
    else pos[i]--;
  }
  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    pulseTicks[i] = degToTicks(pos[i]);
  }
}

ISR(TIMER1_COMPA_vect) {
  currServo = (ServoEnum)((currServo + 1) % NUM_SERVOS);
  OCR1B = pulseTicks[currServo];
  PORTC |= (1 << currServo);
}

ISR(TIMER1_COMPB_vect) {
  PORTC &= ~(1 << currServo);
}

void setupTimer1() {
  TCCR1A = 0;
  TCCR1B = 0;
  OCR1A = 1250;
  TIMSK1 |= (1 << OCIE1A) | (1 << OCIE1B);
  TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
}

static uint8_t computeChecksum(const uint8_t *data, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i = 0; i < len; i++) cs ^= data[i];
  return cs;
}

static void sendFrame(const TPacket *pkt) {
  uint8_t frame[FRAME_SIZE];
  frame[0] = MAGIC_HI;
  frame[1] = MAGIC_LO;
  memcpy(&frame[2], pkt, TPACKET_SIZE);
  frame[2 + TPACKET_SIZE] = computeChecksum((const uint8_t *)pkt, TPACKET_SIZE);
  Serial.write(frame, FRAME_SIZE);
}

static bool receiveFrame(TPacket *pkt) {
  static uint8_t state = 0;
  static uint8_t raw[TPACKET_SIZE];
  static uint8_t index = 0;

  while (Serial.available() > 0) {
    uint8_t byte = (uint8_t)Serial.read();
    switch (state) {
      case 0:
        if (byte == MAGIC_HI) state = 1;
        break;
      case 1:
        if (byte == MAGIC_LO) {
          state = 2;
          index = 0;
        } else if (byte != MAGIC_HI) {
          state = 0;
        }
        break;
      case 2:
        raw[index++] = byte;
        if (index >= TPACKET_SIZE) state = 3;
        break;
      case 3: {
        uint8_t expected = computeChecksum(raw, TPACKET_SIZE);
        if (byte == expected) {
          memcpy(pkt, raw, TPACKET_SIZE);
          state = 0;
          return true;
        }
        state = (byte == MAGIC_HI) ? 1 : 0;
        break;
      }
    }
  }
  return false;
}

static void sendOk(const char *msg) {
  TPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.packetType = PACKET_TYPE_RESPONSE;
  pkt.command = RESP_OK;
  if (msg) {
    strncpy(pkt.data, msg, sizeof(pkt.data) - 1);
    pkt.data[sizeof(pkt.data) - 1] = '\0';
  }
  sendFrame(&pkt);
}

static void sendArmStatus() {
  TPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.packetType = PACKET_TYPE_RESPONSE;
  pkt.command = RESP_ARM_STATUS;
  pkt.params[0] = target[BASE];
  pkt.params[1] = target[SHOULDER];
  pkt.params[2] = target[ELBOW];
  pkt.params[3] = target[GRIPPER];
  pkt.params[4] = msPerDeg;
  sendFrame(&pkt);
}

static bool applyJointTarget(uint8_t joint, uint32_t value, uint8_t minVal, uint8_t maxVal) {
  if (value < minVal || value > maxVal) {
    sendOk("ERR_RANGE");
    return false;
  }
  target[joint] = (uint8_t)value;
  return true;
}

static void handleArmCommand(const TPacket *cmd) {
  uint32_t value = cmd->params[0];

  switch (cmd->command) {
    case COMMAND_ARM_BASE:
      if (applyJointTarget(BASE, value, ARM_BASE_MIN, ARM_BASE_MAX)) {
        sendOk("BASE_OK");
        sendArmStatus();
      }
      break;

    case COMMAND_ARM_SHOULDER:
      if (applyJointTarget(SHOULDER, value, ARM_SHOULDER_MIN, ARM_SHOULDER_MAX)) {
        sendOk("SHLDR_OK");
        sendArmStatus();
      }
      break;

    case COMMAND_ARM_ELBOW:
      if (applyJointTarget(ELBOW, value, ARM_ELBOW_MIN, ARM_ELBOW_MAX)) {
        sendOk("ELBOW_OK");
        sendArmStatus();
      }
      break;

    case COMMAND_ARM_GRIPPER:
      if (applyJointTarget(GRIPPER, value, ARM_GRIPPER_MIN, ARM_GRIPPER_MAX)) {
        sendOk("GRIP_OK");
        sendArmStatus();
      }
      break;

    case COMMAND_ARM_HOME:
      target[BASE] = 90;
      target[SHOULDER] = 90;
      target[ELBOW] = 90;
      target[GRIPPER] = 90;
      sendOk("HOME_OK");
      sendArmStatus();
      break;

    case COMMAND_ARM_SET_SPEED:
      if (value < ARM_SPEED_MIN || value > ARM_SPEED_MAX) {
        sendOk("ERR_SPEED");
      } else {
        msPerDeg = (uint8_t)value;
        sendOk("SPEED_OK");
        sendArmStatus();
      }
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(9600);
  DDRC |= SERVO_MASK;
  PORTC &= ~SERVO_MASK;

  for (uint8_t i = 0; i < NUM_SERVOS; i++) {
    pulseTicks[i] = degToTicks(pos[i]);
  }

  cli();
  setupTimer1();
  sei();

  sendOk("ARM_READY");
  sendArmStatus();
}

void loop() {
  updateMotion();

  TPacket incoming;
  if (receiveFrame(&incoming)) {
    if (incoming.packetType == PACKET_TYPE_COMMAND) {
      handleArmCommand(&incoming);
    }
  }
}
