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
uint16_t lastDebounceTime = 0; //debounce variable
bool currentStatus = true;     //helper variable for INT5 ISR
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

// =============================================================
// E-Stop state machine
// =============================================================

volatile TState buttonState = STATE_RUNNING;
volatile bool   stateChanged = false;

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
 ISR(INT5_vect) {
    uint16_t timeCurrent = TCNT1;

    if(timeCurrent - lastDebounceTime > 250) {
      lastDebounceTime = timeCurrent;
      int state = (PINE & (1 << 5));

      if(state && buttonState == STATE_RUNNING && currentStatus == true) {
        buttonState = STATE_STOPPED;
        stateChanged = true;
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
// PD0 -> sensor OUT
// PJ0 -> S0
// PJ1 -> S1
// PH0 -> S2
// PH1 -> S3

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
    uint8_t lastState = (PIND & (1 << PIND0));

    uint16_t start = TCNT1;

    // Timer1 prescaler = 64 at 16 MHz
    // 1 tick = 4 us
    // 100 ms = 25000 ticks
    while ((uint16_t)(TCNT1 - start) < 25000) {
        uint8_t nowState = (PIND & (1 << PIND0));

        // count rising edge
        if(!lastState && nowState) {
            count++;
        }

        lastState = nowState;
    }

    return count;
}

static void readColorChannels(uint32_t *r, uint32_t *g, uint32_t *b) {
    selectRed();
    _delay_ms(5);
    *r = measureChannel100ms() * 10;   // Hz

    selectGreen();
    _delay_ms(5);
    *g = measureChannel100ms() * 10;   // Hz

    selectBlue();
    _delay_ms(5);
    *b = measureChannel100ms() * 10;   // Hz
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

    switch (cmd->command) {
        case COMMAND_ESTOP:
            cli();
            buttonState  = STATE_STOPPED;
            stateChanged = false;
            sei();
            {
                // The data field of a TPacket can carry a short debug string (up to
                // 31 characters).  pi_sensor.py prints it automatically for any packet
                // where data is non-empty, so you can use it to send debug messages
                // from the Arduino to the Pi terminal -- similar to Serial.print().
                TPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.packetType = PACKET_TYPE_RESPONSE;
                pkt.command    = RESP_OK;
                strncpy(pkt.data, "This is a debug message", sizeof(pkt.data) - 1);
                pkt.data[sizeof(pkt.data) - 1] = '\0';
                sendFrame(&pkt);
            }
            sendStatus(STATE_STOPPED);
            break;

        // TODO (Activity 2): add COMMAND_COLOR case here.
        //   Call your color-reading function (which returns Hz), then send a
        //   response packet with the three channel frequencies in Hz.
        case COMMAND_COLOR_SENSOR: {
        uint32_t r, g, b;
        readColorChannels(&r, &g, &b);

        TPacket pkt;
        memset(&pkt, 0, sizeof(pkt));
        pkt.packetType = PACKET_TYPE_RESPONSE;
        pkt.command    = RESP_COLOR_SENSOR;
        pkt.params[0]  = r;
        pkt.params[1]  = g;
        pkt.params[2]  = b;
        sendFrame(&pkt);
        break;
}
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
    EICRB = 0b00000100;
    EIMSK = 0b00100000;
    TCCR1A = 0;          // normal mode
    TCCR1B = 0;
    TCNT1 = 0;           // reset counter
    TCCR1B |= (1 << CS11) | (1 << CS10);   // prescaler = 64
    DDRE &= ~(1 << 5);

    // TO DO
    // SET PD0 AS AN INTERRUPT WITH RISING EDGE
    // COMMAND_COLOUR_SENSOR RESP_COLOUR
    
    // Color Sensor
    // PD0 = OUT from TCS3200 -> input
    DDRD &= ~(1 << DDD0);

    // PJ0 = S0, PJ1 = S1 -> outputs
    DDRJ |= (1 << DDJ0) | (1 << DDJ1);

    // PH0 = S2, PH1 = S3 -> outputs
    DDRH |= (1 << DDH0) | (1 << DDH1);

    // 20% output scaling: S0 = HIGH, S1 = LOW
    PORTJ |=  (1 << PJ0);   // S0 high
    PORTJ &= ~(1 << PJ1);   // S1 low


    
  // TODO (Activity 3a): Enable the button to fire an interrupt on any
    // logical change (both rising and falling edges).
    sei();
}

void loop() {
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
    if (receiveFrame(&incoming)) {
        handleCommand(&incoming);
    }
}
