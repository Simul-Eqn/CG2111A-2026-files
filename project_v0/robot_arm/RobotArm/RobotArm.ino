#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define NUM_SERVOS 4   // number of servos: BASE, SHOULDER, ELBOW, GRIPPER

enum ServoEnum { BASE, SHOULDER, ELBOW, GRIPPER }; //0,1,2,3

volatile ServoEnum currServo = BASE;

// ---- positions (8-bit, ISR safe) ----
volatile uint8_t pos[NUM_SERVOS] = {90, 90, 90, 90};
volatile uint8_t target[NUM_SERVOS] = {90, 50, 90, 90};

volatile uint16_t pulseTicks[NUM_SERVOS];

uint8_t msPerDeg = 10;
unsigned long lastUpdate[NUM_SERVOS] = {0,0,0,0};

// ---- constants ---- (Mask for Setting Servo as I/O)
#define SERVO_MASK ((1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3))

// ---- convert deg → timer ticks ---- 
uint16_t degToTicks(uint8_t deg) {
  long us = map(deg, 0, 180, 500, 2500);
  return us / 4; // 4us per tick
}

// ---- update motion smoothly ----
void updateMotion() {
  unsigned long now = millis();

  for (uint8_t i=0;i<NUM_SERVOS;i++) {
    if (pos[i] == target[i]) continue; // Same Pos?

    if (now - lastUpdate[i] < msPerDeg) continue; // Check Timing
    lastUpdate[i] = now;

    if (pos[i] < target[i]) pos[i]++; // Servo Pos Move
    else pos[i]--;
  }
  
  // Update PWM ticks (Pulse width = desired angle)
  for (int i=0;i<NUM_SERVOS;i++)
    pulseTicks[i] = degToTicks(pos[i]);
}

// ---- serial parsing ---- (3 digits Only)
int parse3(const char *p) {
  if (!isdigit(p[0]) || !isdigit(p[1]) || !isdigit(p[2])) return -1;
  return (p[0]-'0')*100 + (p[1]-'0')*10 + (p[2]-'0');
}

// ---- timer setup ----
void setupTimer1() {
  // Clear A and B Registers
  TCCR1A = 0;
  TCCR1B = 0;

  OCR1A = 1250; // 5ms slot - to split into 4 slots for motors pulses.
  // Servo 1 is active for 500–2500 µs(DegtoTicks) in slot 1 (0–5 ms).
  TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B);
  // CTC mode - Prescaler 64 → 4 µs per tick because too large clucky, too small unable to respo
  TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
}

// ---- ISR A: Start servo pulse slot ----
ISR(TIMER1_COMPA_vect) {
  // Start with Shoulder first thou. But all move in 20ms
  currServo = (ServoEnum)((currServo+1) % NUM_SERVOS); // Warps back to 4%4 = 0
  
  // Get Updated Pulses from loop to generate the High (Moving the servo)
  OCR1B = pulseTicks[currServo];
  // If currServo = 0 → 1 << 0 = 00000001 (bit 0). - Set High
  PORTC |= (1 << currServo);
}

// ---- ISR B: end servo's pulse ----
ISR(TIMER1_COMPB_vect) {
  // Set Servo Low 00000000
  PORTC &= ~(1 << currServo);
}

void setup() {
  Serial.begin(115200);

  DDRC |= SERVO_MASK; // Servo Output as direction for PINs
  PORTC &= ~SERVO_MASK; // Set Output as low

  // Save Current Pos for servos, so that timer knows its ticks for PWM Move
  for (int i=0;i<NUM_SERVOS;i++){
    pulseTicks[i] = degToTicks(pos[i]);
  }
  cli();
  setupTimer1(); // Timer setup
  sei();
  Serial.println("READY");
}

void loop() {

  updateMotion();

  if (!Serial.available()) return; // Check for Serial Input

  // Get the serial Input - read to newline
  char buf[16];
  int n = Serial.readBytesUntil('\n', buf, sizeof(buf)-1);
  buf[n] = '\0';

  // Empty Input
  if (n==0) return;

  // Home Input Process
  if (buf[0]=='H') {
    for(int i = 0; i < NUM_SERVOS; i++){
      target[i] = 90;
    }
    Serial.println("HOME");
    return;
  }

  // Out of bounds Input (B090XXX???)
  if (n != 4) {
    Serial.println("ERR: Invalid length");
    return;
  }

  // Val from Parsing Serial Command
  int val = parse3(&buf[1]);
  if (val < 0) {
    Serial.println("ERR: Non-numeric value");
    return;
  }

  switch(buf[0]) {
    case 'V': msPerDeg = val; break;
    case 'B': target[BASE]=constrain(val,0,180); break;
    case 'S': target[SHOULDER]=constrain(val,70,120); break;
    case 'E': target[ELBOW]=constrain(val,60,120); break;
    case 'G': target[GRIPPER]=constrain(val,70,100); break;
    default: Serial.println("ERR: Unknown command"); break;
  }
}
