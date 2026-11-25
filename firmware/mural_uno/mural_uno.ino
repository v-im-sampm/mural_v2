// Mural v2 motion controller for Arduino Uno + CNC Shield V3.
//
// The Uno only does the real-time work: coordinated stepping of the two belt
// motors and driving the pen servo. Everything else (web UI, SVG processing,
// Mural's kinematic model, the setup phases) runs on a PC in
// host/mural_bridge.py, which streams moves to this sketch over USB serial.
//
// Serial protocol (115200 baud, one command per line, '\n' terminated).
// Every line gets exactly one reply: "ok" or "err <reason>".
//
//   M <left> <right> <speed>   Queue a straight move to ABSOLUTE step positions.
//                              <speed> = steps/s of the motor that moves furthest;
//                              the other motor is scaled so both finish together.
//   P <angle> [deg_per_s]      Queue a slow pen-servo move (runs after queued moves).
//   S <angle>                  Set the servo right now (calibration).
//   J <l> <r> [speed]          Jog: l, r in {-1, 0, 1}; runs until "J 0 0".
//   Z <left> <right>           Set current position (steps). Only when idle.
//   E <0|1>                    Disable / enable the stepper drivers.
//   W                          Reply "ok" once all queued work has finished.
//   ?                          Reply "<Idle|Run|Jog> <left> <right> <queued>" then "ok".
//   V                          Reply firmware version then "ok".
//   !                          Emergency stop (single character, no newline needed):
//                              drop the queue, stop motors immediately.
//
// The reply to M/P is delayed while the queue is full; that is the flow control.

#include <Arduino.h>
#include <Servo.h>
#include <util/atomic.h>
#include <stdlib.h>
#include "config.h"

#define FW_VERSION "MuralUno 1.0"

#if LEFT_STEP_PIN > 7 || RIGHT_STEP_PIN > 7 || LEFT_DIR_PIN > 7 || RIGHT_DIR_PIN > 7
#error "Step/dir pins must be on PORTD (Arduino pins 0-7)"
#endif

#define LEFT_STEP_BIT (1 << LEFT_STEP_PIN)
#define RIGHT_STEP_BIT (1 << RIGHT_STEP_PIN)
#define LEFT_DIR_BIT (1 << LEFT_DIR_PIN)
#define RIGHT_DIR_BIT (1 << RIGHT_DIR_PIN)
#define QUEUE_MASK (QUEUE_SIZE - 1)

#if (QUEUE_SIZE & QUEUE_MASK) != 0
#error "QUEUE_SIZE must be a power of two"
#endif

enum BlockType : uint8_t { BLOCK_MOVE, BLOCK_PEN };

struct Block {
  uint8_t type;
  int32_t dl;          // BLOCK_MOVE: left delta in steps
  int32_t dr;          // BLOCK_MOVE: right delta in steps
  uint32_t ticks;      // BLOCK_MOVE: duration in timer ticks
  int16_t angle;       // BLOCK_PEN: target angle
  uint16_t degPerSec;  // BLOCK_PEN: ramp speed
};

static Block queue[QUEUE_SIZE];
static volatile uint8_t qHead = 0;  // advanced by the ISR (moves) or loop() (pen)
static volatile uint8_t qTail = 0;  // advanced by loop() only

// State of the segment being stepped by the ISR.
static volatile bool segActive = false;
static volatile uint32_t segTicks, segTick, absL, absR, accL, accR;
static volatile int8_t sgnL, sgnR;

// Current motor positions in steps (updated by the ISR).
static volatile int32_t posL = 0, posR = 0;

// Jogging.
static volatile int8_t jogL = 0, jogR = 0;
static volatile uint16_t jogPeriod = (TICK_HZ + DEFAULT_JOG_SPEED / 2) / DEFAULT_JOG_SPEED, jogCount = 0;

// Position at the end of everything queued so far (loop() only).
static int32_t plannedL = 0, plannedR = 0;

static Servo pen;
static int16_t penAngle = SERVO_BOOT_ANGLE;

static char line[48];
static uint8_t lineLen = 0;
static bool lineReady = false;
static bool lineOverflow = false;

// ---------------------------------------------------------------------------
// Step generation (Timer2 compare-match interrupt, TICK_HZ)
// ---------------------------------------------------------------------------

static inline void setDirBits(int8_t l, int8_t r) {
  uint8_t port = PORTD & ~(LEFT_DIR_BIT | RIGHT_DIR_BIT);
  if ((l > 0) != INVERT_LEFT_DIR) port |= LEFT_DIR_BIT;
  if ((r > 0) != INVERT_RIGHT_DIR) port |= RIGHT_DIR_BIT;
  PORTD = port;
}

static inline void pulse(uint8_t bits) {
  PORTD |= bits;
  __builtin_avr_delay_cycles(40);  // ~2.5 us; A4988 needs 1 us, DRV8825 1.9 us
  PORTD &= ~bits;
}

ISR(TIMER2_COMPA_vect) {
  if (!segActive) {
    uint8_t h = qHead;
    if (h != qTail && queue[h].type == BLOCK_MOVE) {
      const Block &b = queue[h];
      sgnL = b.dl < 0 ? -1 : 1;
      sgnR = b.dr < 0 ? -1 : 1;
      absL = b.dl < 0 ? -b.dl : b.dl;
      absR = b.dr < 0 ? -b.dr : b.dr;
      segTicks = b.ticks;
      segTick = 0;
      accL = accR = segTicks / 2;
      setDirBits(sgnL, sgnR);
      qHead = (h + 1) & QUEUE_MASK;
      segActive = true;
      return;  // first step on the next tick gives the drivers direction setup time
    }
    if (jogL != 0 || jogR != 0) {
      if (++jogCount >= jogPeriod) {
        jogCount = 0;
        uint8_t bits = 0;
        if (jogL != 0) { bits |= LEFT_STEP_BIT; posL += jogL; }
        if (jogR != 0) { bits |= RIGHT_STEP_BIT; posR += jogR; }
        pulse(bits);
      }
    }
    return;
  }

  // DDA: each motor steps exactly abs* times, evenly spread over segTicks ticks.
  uint8_t bits = 0;
  accL += absL;
  if (accL >= segTicks) { accL -= segTicks; bits |= LEFT_STEP_BIT; posL += sgnL; }
  accR += absR;
  if (accR >= segTicks) { accR -= segTicks; bits |= RIGHT_STEP_BIT; posR += sgnR; }
  if (bits) pulse(bits);
  if (++segTick >= segTicks) segActive = false;
}

static void startStepTimer() {
  // Timer2, CTC mode, prescaler 8 -> 2 MHz, compare at 2 MHz / TICK_HZ.
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS21);
  OCR2A = (uint8_t)(F_CPU / 8 / TICK_HZ - 1);
  TCNT2 = 0;
  TIMSK2 = _BV(OCIE2A);
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static inline uint8_t queued() { return (qTail - qHead) & QUEUE_MASK; }
static inline bool queueFull() { return queued() == QUEUE_SIZE - 1; }
static inline bool jogging() { return jogL != 0 || jogR != 0; }
static inline bool idle() { return qHead == qTail && !segActive; }

static void readPositions(int32_t &l, int32_t &r) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    l = posL;
    r = posR;
  }
}

static void enqueue(const Block &b) {
  queue[qTail] = b;
  qTail = (qTail + 1) & QUEUE_MASK;  // publish after the block is fully written
}

static void slowServoMove(int16_t target, uint16_t degPerSec) {
  // Same behaviour as Mural's doSlowMove(): ramp, then let the servo settle.
  if (degPerSec == 0) degPerSec = 90;
  int16_t start = penAngle;
  if (start != target) {
    unsigned long t0 = millis();
    while (true) {
      unsigned long dt = millis() - t0;
      long progress = (long)dt * degPerSec / 1000;
      int16_t a = target > start ? start + progress : start - progress;
      if ((target > start && a >= target) || (target < start && a <= target)) break;
      pen.write(a);
      delay(10);
    }
    pen.write(target);
    delay(200);
  }
  penAngle = target;
}

static void emergencyStop() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    segActive = false;
    qHead = qTail;
    jogL = jogR = 0;
  }
  readPositions(plannedL, plannedR);
  if (lineReady) {
    Serial.println(F("err stopped"));  // answer the line we were holding
    lineReady = false;
    lineLen = 0;
  }
  Serial.println(F("# stopped"));
}

// ---------------------------------------------------------------------------
// Command handling. Returns false if the command must be retried later.
// ---------------------------------------------------------------------------

static bool parseLongs(const char *s, long *out, uint8_t count, uint8_t required) {
  char *end;
  for (uint8_t i = 0; i < count; i++) {
    long v = strtol(s, &end, 10);
    if (end == s) return i >= required;
    out[i] = v;
    s = end;
  }
  return true;
}

static bool execute(const char *cmd) {
  long a[3] = {0, 0, 0};
  const char *args = cmd + 1;

  switch (cmd[0]) {
    case 'M': {
      if (!parseLongs(args, a, 3, 3)) { Serial.println(F("err args")); return true; }
      if (jogging()) { Serial.println(F("err jogging")); return true; }
      if (queueFull()) return false;
      int32_t dl = a[0] - plannedL, dr = a[1] - plannedR;
      if (dl == 0 && dr == 0) { Serial.println(F("ok")); return true; }
      uint32_t maxd = max((uint32_t)labs(dl), (uint32_t)labs(dr));
      long speed = constrain(a[2], 1L, (long)TICK_HZ);
      uint32_t ticks = (uint32_t)(((uint64_t)maxd * TICK_HZ + speed - 1) / speed);
      if (ticks < maxd) ticks = maxd;
      Block b = {BLOCK_MOVE, dl, dr, ticks, 0, 0};
      enqueue(b);
      plannedL = a[0];
      plannedR = a[1];
      Serial.println(F("ok"));
      return true;
    }
    case 'P': {
      a[1] = 90;
      if (!parseLongs(args, a, 2, 1)) { Serial.println(F("err args")); return true; }
      if (queueFull()) return false;
      Block b = {BLOCK_PEN, 0, 0, 0, (int16_t)constrain(a[0], 0, 180), (uint16_t)constrain(a[1], 1, 1000)};
      enqueue(b);
      Serial.println(F("ok"));
      return true;
    }
    case 'S': {
      if (!parseLongs(args, a, 1, 1)) { Serial.println(F("err args")); return true; }
      penAngle = constrain(a[0], 0, 180);
      pen.write(penAngle);
      Serial.println(F("ok"));
      return true;
    }
    case 'J': {
      a[2] = DEFAULT_JOG_SPEED;
      if (!parseLongs(args, a, 3, 2)) { Serial.println(F("err args")); return true; }
      if (!idle()) { Serial.println(F("err busy")); return true; }
      int8_t l = a[0] > 0 ? 1 : (a[0] < 0 ? -1 : 0);
      int8_t r = a[1] > 0 ? 1 : (a[1] < 0 ? -1 : 0);
      long speed = constrain(a[2], 1L, (long)TICK_HZ);
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setDirBits(l, r);
        jogPeriod = (TICK_HZ + speed / 2) / speed;  // rounded
        jogCount = 0;
        jogL = l;
        jogR = r;
      }
      if (l == 0 && r == 0) readPositions(plannedL, plannedR);
      Serial.println(F("ok"));
      return true;
    }
    case 'Z': {
      if (!parseLongs(args, a, 2, 2)) { Serial.println(F("err args")); return true; }
      if (!idle() || jogging()) { Serial.println(F("err busy")); return true; }
      ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        posL = a[0];
        posR = a[1];
      }
      plannedL = a[0];
      plannedR = a[1];
      Serial.println(F("ok"));
      return true;
    }
    case 'E': {
      if (!parseLongs(args, a, 1, 1)) { Serial.println(F("err args")); return true; }
      digitalWrite(ENABLE_PIN, a[0] ? LOW : HIGH);
      Serial.println(F("ok"));
      return true;
    }
    case 'W':
      if (!idle()) return false;
      Serial.println(F("ok"));
      return true;
    case '?': {
      int32_t l, r;
      readPositions(l, r);
      Serial.print(jogging() ? F("Jog ") : (idle() ? F("Idle ") : F("Run ")));
      Serial.print(l);
      Serial.print(' ');
      Serial.print(r);
      Serial.print(' ');
      Serial.println(queued() + (segActive ? 1 : 0));
      Serial.println(F("ok"));
      return true;
    }
    case 'V':
      Serial.println(F(FW_VERSION));
      Serial.println(F("ok"));
      return true;
    case '\0':
      Serial.println(F("ok"));
      return true;
    default:
      Serial.println(F("err unknown"));
      return true;
  }
}

static void readSerial() {
  while (Serial.available()) {
    if (lineReady) {
      // Holding a command until there is room; only an emergency stop gets through.
      if (Serial.peek() == '!') { Serial.read(); emergencyStop(); }
      return;
    }
    char c = Serial.read();
    if (c == '!') { emergencyStop(); continue; }
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLen] = '\0';
      if (lineOverflow) {
        Serial.println(F("err too long"));
        lineOverflow = false;
        lineLen = 0;
      } else {
        lineReady = true;
      }
      continue;
    }
    if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = (c >= 'a' && c <= 'z') ? c - 32 : c;
    } else {
      lineOverflow = true;
    }
  }
}

static void servicePen() {
  // Pen blocks run in loop() (not the ISR) once all moves before them are done.
  if (segActive) return;
  uint8_t h = qHead;
  if (h == qTail || queue[h].type != BLOCK_PEN) return;
  slowServoMove(queue[h].angle, queue[h].degPerSec);
  qHead = (h + 1) & QUEUE_MASK;
}

void setup() {
  // Enable the drivers right away so the motors hold the robot on the wall.
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pen.attach(SERVO_PIN);
  pen.write(SERVO_BOOT_ANGLE);

  Serial.begin(SERIAL_BAUD);
  startStepTimer();
  Serial.println(F(FW_VERSION " ready"));
}

void loop() {
  readSerial();
  if (lineReady && execute(line)) {
    lineReady = false;
    lineLen = 0;
  }
  servicePen();
}
