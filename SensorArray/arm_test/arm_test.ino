/*
 * arm_test.ino
 *
 * Standalone robot-arm test sketch.
 *
 * This keeps the same framed serial protocol and arm commands used by
 * sensor_miniproject_template.ino, but removes the color sensor, motor,
 * E-Stop, and Timer2 logic so the arm can be tested in isolation.
 *
 * Suggested use:
 *   1. Upload this sketch to the Arduino Mega.
 *   2. Run your existing second terminal / Pi tooling.
 *   3. Test each joint and confirm which physical pin really works.
 */

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <stdint.h>
#include <string.h>

// =============================================================
// Pin mapping
// Change these if you want to test a different joint-to-pin map.
// On Arduino Mega: A9-A12 are PK1-PK4.
// =============================================================

#define ARM_BASE_PIN      A12
#define ARM_SHOULDER_PIN  A10
#define ARM_ELBOW_PIN     A11
#define ARM_GRIPPER_PIN   A9

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

// =============================================================
// TPacket protocol
// Must stay in sync with packets.py / second_terminal.py
// =============================================================

typedef enum {
  PACKET_TYPE_COMMAND  = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
  COMMAND_ESTOP         = 0,
  COMMAND_COLOR_SENSOR  = 1,
  COMMAND_FORWARD       = 2,
  COMMAND_BACKWARD      = 3,
  COMMAND_LEFT          = 4,
  COMMAND_RIGHT         = 5,
  COMMAND_SET_SPEED     = 6,
  COMMAND_STOP          = 7,
  COMMAND_ARM_BASE      = 20,
  COMMAND_ARM_SHOULDER  = 21,
  COMMAND_ARM_ELBOW     = 22,
  COMMAND_ARM_GRIPPER   = 23,
  COMMAND_ARM_HOME      = 24,
  COMMAND_ARM_SET_SPEED = 25
} TCommandType;

typedef enum {
  RESP_OK           = 0,
  RESP_STATUS       = 1,
  RESP_COLOR_SENSOR = 2,
  RESP_MOTOR_STATUS = 3,
  RESP_ARM_STATUS   = 4
} TResponseType;

typedef struct {
  uint8_t  packetType;
  uint8_t  command;
  uint8_t  dummy[2];
  char     data[32];
  uint32_t params[16];
} TPacket;

#define MAGIC_HI     0xDE
#define MAGIC_LO     0xAD
#define TPACKET_SIZE ((uint8_t)sizeof(TPacket))
#define FRAME_SIZE   (2 + TPACKET_SIZE + 1)

typedef enum {
  ARM_JOINT_BASE = 0,
  ARM_JOINT_SHOULDER = 1,
  ARM_JOINT_ELBOW = 2,
  ARM_JOINT_GRIPPER = 3,
  ARM_JOINT_COUNT = 4
} ArmJoint;

// Bare-metal routing table. If you want to remap joints, update this table.
// Current mapping is:
//   BASE     -> PK1 -> A9
//   SHOULDER -> PK2 -> A10
//   ELBOW    -> PK3 -> A11
//   GRIPPER  -> PK4 -> A12
static const uint8_t armBitMasks[ARM_JOINT_COUNT] = {
  _BV(PK1), _BV(PK2), _BV(PK3), _BV(PK4)
};
#define ARM_OUTPUT_MASK (_BV(PK1) | _BV(PK2) | _BV(PK3) | _BV(PK4))

static uint8_t armPos[ARM_JOINT_COUNT] = {90, 90, 90, 90};
static uint8_t armTarget[ARM_JOINT_COUNT] = {90, 90, 90, 90};
static uint8_t armMsPerDeg = 10;
static uint32_t armLastUpdate[ARM_JOINT_COUNT] = {0, 0, 0, 0};
static volatile uint16_t armPulseTicks[ARM_JOINT_COUNT];
static volatile uint8_t armActiveJoint = ARM_JOINT_GRIPPER;

static uint16_t degToTicks(uint8_t deg) {
  long us = map(deg, 0, 180, 500, 2500);
  return (uint16_t)(us / 4);
}

static void armRefreshPulseTicks() {
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    const uint16_t ticks = degToTicks(armPos[i]);
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      armPulseTicks[i] = ticks;
    }
  }
}

ISR(TIMER1_COMPA_vect) {
  armActiveJoint = (uint8_t)((armActiveJoint + 1) % ARM_JOINT_COUNT);
  OCR1B = armPulseTicks[armActiveJoint];
  PORTK |= armBitMasks[armActiveJoint];
}

ISR(TIMER1_COMPB_vect) {
  PORTK &= (uint8_t)~armBitMasks[armActiveJoint];
}

static void armTimerInit() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 1250; // 5 ms slot at 16 MHz / 64 => 4 us per tick
  OCR1B = armPulseTicks[ARM_JOINT_BASE];
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

static void sendArmStatus() {
  TPacket pkt;
  memset(&pkt, 0, sizeof(pkt));
  pkt.packetType = PACKET_TYPE_RESPONSE;
  pkt.command = RESP_ARM_STATUS;
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

static void armInit() {
  DDRK |= ARM_OUTPUT_MASK;
  PORTK &= (uint8_t)~ARM_OUTPUT_MASK;
  armRefreshPulseTicks();
}

static bool armApplyTarget(uint8_t joint, uint32_t value, uint8_t minV, uint8_t maxV) {
  if (value < minV || value > maxV) {
    sendOkWithMsg("ERR_RANGE");
    return false;
  }

  armTarget[joint] = (uint8_t)value;
  armPos[joint] = armTarget[joint];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    armPulseTicks[joint] = degToTicks(armPos[joint]);
  }
  armLastUpdate[joint] = millis();
  return true;
}

static void armHome() {
  armTarget[ARM_JOINT_BASE] = 90;
  armTarget[ARM_JOINT_SHOULDER] = 90;
  armTarget[ARM_JOINT_ELBOW] = 90;
  armTarget[ARM_JOINT_GRIPPER] = 90;

  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    armPos[i] = armTarget[i];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      armPulseTicks[i] = degToTicks(armPos[i]);
    }
    armLastUpdate[i] = millis();
  }
}

static void armUpdateMotion() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < ARM_JOINT_COUNT; i++) {
    if (armPos[i] == armTarget[i]) continue;
    if ((now - armLastUpdate[i]) < armMsPerDeg) continue;
    armLastUpdate[i] = now;
    if (armPos[i] < armTarget[i]) armPos[i]++;
    else armPos[i]--;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      armPulseTicks[i] = degToTicks(armPos[i]);
    }
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
      sendOkWithMsg("IGNORED");
      break;
  }
}

void setup() {
  Serial.begin(9600);
  armInit();
  cli();
  armTimerInit();
  sei();
  sendOkWithMsg("ARM_TEST_READY");
  sendArmStatus();
}

void loop() {
  armUpdateMotion();

  TPacket incoming;
  if (receiveFrame(&incoming)) {
    if (incoming.packetType == PACKET_TYPE_COMMAND) {
      handleArmCommand(&incoming);
    }
  }
}
