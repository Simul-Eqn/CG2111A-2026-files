// ============================================================
// Studio 12: Remote Control - Challenge Activity
// serial_isr_challenge.ino
//
// Bare-metal ISR-driven USART using circular TX and RX buffers.
// Complete the five TODO sections marked below.
//
// The ATmega328P USART registers you will need:
//   UBRR0H:UBRR0L  baud-rate register
//   UCSR0B          control & status register B
//     TXEN0           transmitter enable bit
//     RXEN0           receiver enable bit
//     UDRIE0          USART Data Register Empty interrupt enable
//     RXCIE0          RX Complete interrupt enable
//   UCSR0C          control & status register C (frame format)
//     UCSZ01, UCSZ00  character size bits (11 = 8 bits)
//   UDR0            data register: write to send, read to receive
//   USART_UDRE_vect ISR vector for the TX Data Register Empty interrupt
//   USART_RX_vect   ISR vector for the RX Complete interrupt
// ============================================================

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdint.h>
#include <util/delay.h>

// ============================================================
// PACKET DEFINITIONS  (must match template.ino / pi_template.py)
// ============================================================

#define MAX_STR_LEN   32
#define PARAMS_COUNT  16

typedef enum {
  PACKET_TYPE_COMMAND  = 0,
  PACKET_TYPE_RESPONSE = 1,
  PACKET_TYPE_MESSAGE  = 2,
} TPacketType;

typedef enum {
  COMMAND_ESTOP = 0,
} TCommandType;

typedef enum {
  RESP_OK     = 0,
  RESP_STATUS = 1,
} TResponseType;

// TPacket: 1 + 1 + 2 + 32 + (16 * 4) = 100 bytes.
// The dummy[2] field ensures params (uint32_t, 4-byte aligned) starts
// at offset 4, so the layout is identical on every platform.
typedef struct {
  uint8_t  packetType;
  uint8_t  command;
  uint8_t  dummy[2];
  char     data[MAX_STR_LEN];
  uint32_t params[PARAMS_COUNT];
} TPacket;

typedef enum {
  STATE_RUNNING = 0,
  STATE_STOPPED = 1,
} TButtonState;

// ============================================================
// CIRCULAR TX BUFFER
// ============================================================

// TX_BUFFER_SIZE is a power of 2 so that the wrap-around
// can be computed with a fast bitwise AND:
//   index = (index + 1) & TX_BUFFER_MASK
// (Any positive buffer size works; non-power-of-2 sizes use % instead.)
#define TX_BUFFER_SIZE  128
#define TX_BUFFER_MASK  (TX_BUFFER_SIZE - 1)

static volatile uint8_t tx_buf[TX_BUFFER_SIZE];
static volatile uint8_t tx_head = 0;   // producer writes here
static volatile uint8_t tx_tail = 0;   // ISR reads from here

// ============================================================
// CIRCULAR RX BUFFER
// ============================================================

// Same power-of-2 sizing as the TX buffer.
#define RX_BUFFER_SIZE  128
#define RX_BUFFER_MASK  (RX_BUFFER_SIZE - 1)

static volatile uint8_t rx_buf[RX_BUFFER_SIZE];
static volatile uint8_t rx_head = 0;   // ISR writes here
static volatile uint8_t rx_tail = 0;   // consumer reads from here

// ============================================================
// TODO Part A: Implement txEnqueue
// ============================================================
//
// Attempt to copy all `len` bytes from `data` into the circular
// TX buffer.
//
// If the buffer does not have enough free space for all `len`
// bytes, do NOT enqueue anything and return false.
//
// If all bytes were successfully enqueued, enable the UDRE
// interrupt so the ISR begins draining and return true.
//
// Requirements:
//   - Must return immediately (never block waiting for space).
//   - Must not allow tx_head to equal tx_tail after a write
//     (that state means "empty").
//
bool txEnqueue(const uint8_t *data, uint8_t len) {
  // YOUR CODE HERE
  return false;
}

// ============================================================
// TODO Part B: USART Data Register Empty ISR
// ============================================================
//
// When UDR0 is ready for a new byte, the hardware raises the
// UDRE interrupt (if UDRIE0 is set).  This ISR should:
//   1. Load tx_buf[tx_tail] into UDR0.
//   2. Advance tx_tail (with wrap-around).
//   3. If the buffer is now empty (tx_tail == tx_head),
//      clear UDRIE0 to stop the ISR from firing on an
//      empty buffer.
//
ISR(USART_UDRE_vect) {
  // YOUR CODE HERE
}

// ============================================================
// BARE-METAL USART INITIALISATION  (provided, do not modify)
// ============================================================

// Configure USART0 for 8N1 at the given baud rate with both
// transmit and receive enabled.
// ubrr = (F_CPU / (16 * baud)) - 1
// For 9600 baud at 16 MHz: ubrr = 103
void usartInit(uint16_t ubrr) {
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)(ubrr);
  UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1 << RXCIE0); // TX + RX + RX-complete interrupt
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);         // 8 data bits, 1 stop bit
}

// ============================================================
// TODO Part C: Implement rxDequeue
// ============================================================
//
// Attempt to copy `len` bytes from the circular RX buffer into
// `data`.
//
// If fewer than `len` bytes are available, do NOT copy anything
// and return false.
//
// If `len` bytes are available, copy them out (advancing
// rx_tail) and return true.
//
// Requirements:
//   - Must return immediately (never block waiting for data).
//   - Must not read more bytes than are currently in the buffer.
//
bool rxDequeue(uint8_t *data, uint8_t len) {
  // YOUR CODE HERE
  return false;
}

// ============================================================
// TODO Part D: RX Complete ISR
// ============================================================
//
// When the USART finishes receiving a byte, the hardware places
// it in UDR0 and raises the RX Complete interrupt (if RXCIE0 is
// set).  This ISR should:
//   1. Read the received byte from UDR0 immediately (reading
//      UDR0 also clears the interrupt flag).
//   2. Compute the next write position:
//        next = (rx_head + 1) & RX_BUFFER_MASK
//   3. If next == rx_tail the buffer is full: discard the byte.
//   4. Otherwise store the byte in rx_buf[rx_head] and set
//      rx_head = next.
//
ISR(USART_RX_vect) {
  // YOUR CODE HERE
}

// ============================================================
// TODO Part E: setup() and loop()
// ============================================================

void setup() {
  // 1. Initialise USART0 at 9600 baud using usartInit().
  //    Hint: ubrr = (F_CPU / (16 * baud)) - 1 = 103 for 9600 baud at 16 MHz.
  //    usartInit already enables the receiver and RX-complete interrupt.
  // 2. Enable global interrupts.
}

void loop() {
  // TX: Build a TPacket (PACKET_TYPE_RESPONSE / RESP_STATUS / STATE_RUNNING)
  // and transmit it using txEnqueue(), blocking until the buffer accepts the
  // full packet (i.e. loop while txEnqueue returns false).
  //
  // RX: Check whether a complete TPacket has arrived using rxDequeue().
  // If one has arrived and its packetType is PACKET_TYPE_COMMAND with
  // command == COMMAND_ESTOP, reply with RESP_OK using txEnqueue().
  //
  // Wait some delay before the next status transmission.
  //
  // Verify on the Pi with: python3 pi_template.py
  //   - You should see "Status: RUNNING" printed once per second.
  //   - Press Enter in the Pi terminal to send COMMAND_ESTOP; you should
  //     see "Response: OK" confirming the Arduino received it via the RX ISR.
  _delay_ms(200);
}
