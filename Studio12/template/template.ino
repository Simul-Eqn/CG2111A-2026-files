// ============================================================
// Studio 12: Remote Control
// Arduino Template - template.ino
//
// This sketch is used for Activity 3 (Simple Command Protocol
// and E-Stop).  Complete the TODO sections marked below.
//
// ============================================================

#include <stdint.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>

// ============================================================
// PACKET DEFINITIONS  (must match pi_template.py)
// ============================================================

#define MAX_STR_LEN   32
#define PARAMS_COUNT  16

typedef enum {
  PACKET_TYPE_COMMAND  = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

// Commands sent from Pi to Arduino
typedef enum {
  COMMAND_ESTOP = 0,
} TCommandType;

// Responses sent from Arduino to Pi
typedef enum {
  RESP_OK     = 0,
  RESP_STATUS = 1,
} TResponseType;

// TPacket: 1 + 1 + 2 + 32 + (16 * 4) = 100 bytes.
// The dummy[2] field ensures params (uint32_t, 4-byte aligned) starts
// at offset 4, so the layout is identical on every platform.
typedef struct {
  uint8_t  packetType;               // TPacketType
  uint8_t  command;                  // TCommandType / TResponseType
  uint8_t  dummy[2];                 // alignment padding
  char     data[MAX_STR_LEN];        // optional null-terminated string
  uint32_t params[PARAMS_COUNT];     // numeric parameters
} TPacket;

// ============================================================
// BUTTON / E-STOP STATE
// ============================================================

typedef enum {
  STATE_RUNNING = 0,
  STATE_STOPPED = 1,
} TButtonState;

volatile TButtonState buttonState  = STATE_RUNNING;
volatile bool         stateChanged = false;

unsigned long lastDebounceTime = 0;

// ============================================================
// PACKET UTILITIES
// ============================================================

// Transmit a TPacket as raw bytes.
void sendPacket(TPacket *pkt) {
  Serial.write((uint8_t *)pkt, sizeof(TPacket));
}

// Accumulate incoming bytes until a complete TPacket has
// arrived.  Returns true (and fills *pkt) when done.
bool receivePacket(TPacket *pkt) {
  static uint8_t buf[sizeof(TPacket)];
  static uint8_t count = 0;

  while (Serial.available() > 0) {
    buf[count++] = (uint8_t)Serial.read();
    if (count == sizeof(TPacket)) {
      memcpy(pkt, buf, sizeof(TPacket));
      count = 0;
      return true;
    }
  }
  return false;
}

// TODO (Activity 3): Implement sendResponse().
//
// Create a TPacket with:
//   packetType = PACKET_TYPE_RESPONSE
//   command    = resp
//   params[0]  = param
// then call sendPacket() to transmit it.
void sendResponse(TResponseType resp, uint32_t param) {
  TPacket PacketToSend;
  
  PacketToSend.packetType = PACKET_TYPE_RESPONSE;
  PacketToSend.command = resp;
  PacketToSend.params[0] = param;

  sendPacket(&PacketToSend);
}

// TODO (Activity 3): Implement sendStatus().
//
// Call sendResponse correctly
void sendStatus() {
  // YOUR CODE HERE
  sendResponse(RESP_STATUS,buttonState);
}

// Process an incoming COMMAND packet.
void handleCommand(TPacket *pkt) {
  if (pkt->command == COMMAND_ESTOP) {
    buttonState  = STATE_STOPPED;
    stateChanged = true;
    sendResponse(RESP_OK, 0);
  }
}

// ============================================================
// BUTTON: INTERRUPT SERVICE ROUTINE
// ============================================================
//
// TODO (Activity 3): Complete ISR to implement

// Required behaviour:
//   STATE_RUNNING + button pressed  -> STATE_STOPPED; stateChanged = true
//   STATE_STOPPED + button released -> STATE_RUNNING; stateChanged = true
//
ISR(INT5_vect) {
  uint16_t timeCurrent = TCNT1;

  if(timeCurrent - lastDebounceTime > 250) {
    lastDebounceTime = timeCurrent;
    int state = digitalRead(3);

    if(state == HIGH && buttonState == STATE_RUNNING) {
      buttonState = STATE_STOPPED;
      stateChanged = true;
    } 
    else if(state == LOW && buttonState == STATE_STOPPED) {
      buttonState = STATE_RUNNING;
      stateChanged = true;
    }
  }
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(9600);
  cli();
  EICRB = 0b00000100;
  EIMSK = 0b00100000;

  TCCR1A = 0;          // normal mode
  TCCR1B = 0;

  TCNT1 = 0;           // reset counter

  TCCR1B |= (1 << CS11) | (1 << CS10);   // prescaler = 64

  pinMode(3, INPUT);
 // TODO (Activity 3a): Enable the button to fire an interrupt on any
  // logical change (both rising and falling edges).
  sei();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  // Check for incoming command packets from the Pi.
  TPacket incoming;
  if (receivePacket(&incoming)) {
    if (incoming.packetType == PACKET_TYPE_COMMAND) {
      handleCommand(&incoming);
    }
  }

  // If the button state changed (hardware ISR), send a status
  // update to the Pi.
  if (stateChanged) {
    stateChanged = false;
    sendStatus();
  }
}
