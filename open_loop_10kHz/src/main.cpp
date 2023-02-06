// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 02/05/23
// The following code makes our Arduino UNO output a 10kHz PWM signal.
// This implementation using the Timer1 peripheral yields a PWM bit-resolution of approximately 10.64   (R_bit = log(TOP + 1) / log(2))
// This is the slowest PWM signal allowed by the project constraints.

#include <Arduino.h>

void setup()
{
  // We modify the following register to:
  //  - Set the wave generation mode to mode 14 (fast PWM with TOP defined by the ICR1 register) (bits 1 and 0 of the 4-bit substring)
  //  - Set pin 9 to Fast PWM compare output mode to mode 2 (non-inverting)
  TCCR1A = _BV(COM1A1) | _BV(WGM11);

  // We modify the following register to:
  //  -  Set the wave generation mode to mode 14 (fast PWM with TOP defined by the ICR1 register) (bits 3 and 2 of the 4-bit substring)
  //  -  Set the clock select mode to mode 1 (enabled, but without prescaling)
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  // We modify the following register to:
  //  - Specify the TOP value. At this value, the 16-bit counter will clear. 
  //  - Output frequency formula: f_PWM = f_I/O / (N(1 + TOP))
  //  - For our prescale N = 1 and f_I/O = 16MHz, we write TOP = 1599 to the register.
  ICR1 = 1599;

  // Using the analogWrite() is a nice abstraction that has the same effect as manually setting up a pin's output compare register.
  // Duty Cycle D = (second argument of analogWrite + 1) / (TOP + 1)
  analogWrite(9, 799);
}

void loop()
{
  // Nothing! This is just a basic test.
}