// Mural v2 on Arduino Uno + CNC Shield V3 - hardware configuration.
//
// Pin numbers follow the standard CNC Shield V3 layout (the same one grbl uses):
//   X slot  -> LEFT belt motor
//   Y slot  -> RIGHT belt motor
//   Z+ end-stop header (signal pin, D11) -> pen-lift servo signal
#ifndef MURAL_UNO_CONFIG_H
#define MURAL_UNO_CONFIG_H

#define SERIAL_BAUD 115200

// Stepper pins on the CNC Shield V3. Direct port writes are used in the ISR,
// so these must stay on PORTD (Arduino pins 0-7).
#define LEFT_STEP_PIN 2   // X.STEP
#define RIGHT_STEP_PIN 3  // Y.STEP
#define LEFT_DIR_PIN 5    // X.DIR
#define RIGHT_DIR_PIN 6   // Y.DIR
#define ENABLE_PIN 8      // EN, shared by all drivers, active LOW

// Pen servo signal. D11 is the "Z+" / "Z-" end-stop header on the CNC Shield V3.
#define SERVO_PIN 11

// Flip these if a motor turns the wrong way: "retract" (l-ret / r-ret in the
// Mural UI) must WIND the belt onto the pulley. The upstream Mural firmware
// inverts the left motor, so that is the default here too.
#define INVERT_LEFT_DIR true
#define INVERT_RIGHT_DIR false

// Servo angle written at power-up. 90 is "pen up" in Mural.
#define SERVO_BOOT_ANGLE 90

// Step-generation timer rate. 20 kHz gives 50 us resolution and allows up to
// 20000 steps/s per motor, far above what Mural uses (<= 3000 steps/s).
#define TICK_HZ 20000UL

// Default jog speed in steps/s (matches Mural's printSpeedSteps).
#define DEFAULT_JOG_SPEED 650

// Number of queued motion/pen blocks. Must be a power of two.
#define QUEUE_SIZE 16

#endif
