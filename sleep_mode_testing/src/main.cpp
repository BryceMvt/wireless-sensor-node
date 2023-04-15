// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 03/28/23
// Used to test and implement one of the six builtin sleep modes the ATmega328p offers.

#include <Arduino.h>

// One 30s sleep period composed of individual 8-8-8-4-2 second sleeps
void sleep_30s()
{
  // Configure the WDT timeout period to 8s (resets it in the process)
  cli();                                      // Stop all interrupts
  WDTCSR = _BV(WDE) | _BV(WDCE);              // Set watchdog enable and change enable (starts the timed config process)
  WDTCSR = _BV(WDP3) | _BV(WDP0) | _BV(WDIE); // Set the prescaler to 1024K (8s) and set the WDT mode to interrupt only (no reset)
  sei();                                      // Enable interrupts once again

  // Disable the ADC (large power savings)
  ADCSRA &= ~(_BV(ADEN));

  // Enable sleep mode
  SMCR |= _BV(SE);

  // Disable brownout detection (BOD) (large power savings)
  // This has to be done right before the SLEEP instruction
  MCUCR |= _BV(BODS) | _BV(BODSE); // Set these 2 bits to start a timed config process much like with the WDT
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS; // BODSE needs to be cleared, but BODS stays set.

  // 3 consecutive 8-second sleep periods (SLEEP instruction using the inline assembler)
  // We need to take care of the BOD for each sleep period
  __asm__ __volatile__ ("sleep" "\n\t" ::); // Round 1

  __asm__ __volatile__ ("wdr"); // WDT reset instruction
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__ ("sleep" "\n\t" ::); // Round 2

  __asm__ __volatile__ ("wdr");
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__ ("sleep" "\n\t" ::); // Round 3


  // Now we reconfigure the WDT to timeout in 4 seconds (resets it in the process)
  cli();                          // Stop all interrupts
  WDTCSR = _BV(WDE) | _BV(WDCE);  // Set watchdog enable and change enable (starts the timed config process)
  WDTCSR = _BV(WDP3) | _BV(WDIE); // Set the prescaler to 512K (4s) and set the WDT mode to interrupt only (no reset)
  sei();                          // Enable interrupts once again

  // Go to sleep for 4 seconds
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__ ("sleep" "\n\t" ::);


  // Now we reconfigure the WDT to timeout in 2 seconds (resets it in the process)
  cli();                                      // Stop all interrupts
  WDTCSR = _BV(WDE) | _BV(WDCE);              // Set watchdog enable and change enable (starts the timed config process)
  WDTCSR = _BV(WDP2) | _BV(WDP1) | _BV(WDIE); // Set the prescaler to 128K (2s) and set the WDT mode to interrupt only (no reset)
  sei();                                      // Enable interrupts once again

  // Go to sleep for 2 seconds (last sleep period)
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__ ("sleep" "\n\t" ::);

  // Disable sleep mode
  SMCR &= ~(_BV(SE));

  // Enable the ADC (important for temp measurement and boost converter)
  ADCSRA |= _BV(ADEN);
}

ISR(WDT_vect)
{
  // Nothing, just wake up
}

void setup() {
  // Wait 3 seconds before we start messing with interrupts for the sake of the bootloader
  delay(3000);

  // 1: Set the sleep mode to "power down" mode. We only do this once.
  SMCR = (SMCR & ~(_BV(SM0) | _BV(SM1) | _BV(SM2))) | _BV(SM1);

  pinMode(LED_BUILTIN, OUTPUT); // blink LED
}

void loop() {
  // Flash the led for 2 seconds
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);

  // Sleep for 30 seconds
  sleep_30s();
}