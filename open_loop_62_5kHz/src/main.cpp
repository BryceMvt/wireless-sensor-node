// ECE-2804 Wireless Sensor Node Project
// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 02/03/23
// The following code makes our Arduino UNO output a 62.5kHz PWM signal.
// This is the fastest PWM that a stock UNO dev-board can achieve with 8-bit resolution.

#include <Arduino.h>

void setup()
{
  // Use digital pin 6 for our PWM
  // This pin corresponds to output A of timer 0 (OC0A)
  pinMode(6, OUTPUT);

  // First Timer/Counter Control Register for Timer0 (TCCR0A): holds the bit substrings:
  //  - "Timer0 Waveform Generation Mode" ((WGM02 is in TCCR0B), WGM01, WGM00): we set this to 0b011 which corresponds to wave generation mode 3. This is "fast PWM" mode with the "TOP" value set to binary 255 for full 8-bit resolution.
  //  - "Timer0 Compare Match Output A Mode" (COM0A1, COM0A0): we set this to 0b10 in order to use mode 2: "clear OC0A (pin 6) on compare match." This enables the PWM signal on pin 6.
  TCCR0A = _BV(COM0A1) | _BV(WGM01) | _BV(WGM00);

  // Second Timer/Counter Control Register for Timer0 (TCCR0B): holds the prescaler factor
  //  - Prescaler formula for Timer0 and Timer2:    f_PWM = f_CRYSTAL / (M * 256)     where M is the prescale factor
  //  - For f_PWM = 62.5kHz, we need a prescale factor of 1, so we set the bit substring "clock select" (CS02, CS01, CS00) to 001 for a prescale of 1 (technically no prescale).
  //  - We don't need to set any other bits in this register for our purposes.
  TCCR0B = _BV(CS00);

  // Timer0 Output A Output Compare Register (OCR0A): This essentially holds the 8-bit duty cycle value. This is the hard-coding method. In practice, analogWrite() accomplishes the same task.
  //  - Duty cycle:   D = (N + 1) / 256     where N is the value you set this register to.
  //  - For this test, we'll set D = 0.5, so we set this register to N = 127
  OCR0A = 127;
}

void loop()
{
  // Nothing! This is just a basic test.
}