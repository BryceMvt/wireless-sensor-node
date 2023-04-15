// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 04/15/23
// A fully functional wireless sensor node that meets all deliverables through milestone 4

#include <Arduino.h>
#include <SoftwareSerial.h>

// Constants and Global Variables:

const int load_resistance{12}; // Ohms
const int cc_set_point{100};   // mA

const long r_p{47000}; // Ohms (fixed thermistor resistor)

// Constants used for our Steinhart-Hart equation (thermistor)
const double A{0.0002842};
const double B{0.0003677};
const double C{-0.0000004056};

// Analog read pins
const int battery_pin{A0};
const int top_pin{A1};
const int thermistor_pin{A2};
const int v_in_pin{A3};

// ADC voltage divider reference numbers
const int top_ten_point_two{662}; // 10.2V
const int top_ten{652};           // 10V
const int top_seven{479};         // 7V

const int battery_ten_point_two{638}; // 10.2V
const int battery_ten{630};           // 10V
const int battery_seven{468};         // 7V

const int v_in_4V{823}; // 4V on A3 with the zener diode

// Temperature
double current_temperature{};              // degrees C
const int temperature_update_period{5000}; // ms
bool temp_updated{true};

// Print message
const int print_message_period{500}; // ms

// The update period of our feedback loop should be significantly longer than our boost converter's switching period
// to allow the converter system to respond to changes in duty cycle.
const int error_update_period{1};      // ms
const int duty_cycle_update_period{5}; // ms

// Number of elements in our error arrays
const int cc_error_length{10};
const int cv_error_length{10};

double cc_error[cc_error_length]{};
double cv_error[cv_error_length]{};

// Mean values are the most useful numbers for integral feedback control
double cc_mean{};
double cv_mean{};

// This is a multiplier that determines how sensitive our feedback loop is to small changes in measured voltage
const double cc_integral_gain{0.06};
const double cv_integral_gain{0.04}; // 0.04 is good for 1ms error and 5ms duty updates

// We can limit the allowed duty cycle values using the constrain function to avoid the possibility of getting a positive feedback.
// The following minimum and maximum allowed duty cycles were determined experimentally using our open-loop boost converter.
// They are in the form of integers we input to the second function argument of the analogWrite() function.
const bool limit_duty_cycle{true};
const int d_min{145}; // Corresponds to an approximate duty cycle of D = 0.52 (original 150)
const int d_max{188}; // Corresponds to an approximate duty cycle of D = 0.65

// We require a cooldown period before the charger is able to change charging schemes for stability.
const int mode_change_cooldown_duration{10000};

// We use the millis functionality for simplicity
unsigned long current_millis{};

// "Previous" values keep track of the last call of a certain part of the code
unsigned long previous_error_millis{};
unsigned long previous_duty_cycle_millis{};
unsigned long previous_temperature_millis{};
unsigned long previous_charge_mode_change_millis{};
unsigned long previous_print_message_millis{};

// Stores the most recently updated mean value of the error arrays
double cc_current_mean{};
double cv_current_mean{};

// Stores the most recently updated duty cycle integer used for the analogWrite() function
double surrogate_d_current{d_min};
int d_current{d_min};

// FSM enum:
enum charging_scheme : unsigned char
{
  CONSTANT_CURRENT,
  CONSTANT_VOLTAGE
};
charging_scheme current_charging_scheme{CONSTANT_VOLTAGE};

enum power_mode : unsigned char
{
  ASLEEP,
  AWAKE
};
power_mode current_power_mode{ASLEEP};

// Real-time voltages
double top_voltage{};
double battery_voltage{};
double battery_current{};

// Bluetooth stuff
const int rx_pin{2};
const int tx_pin{3};

SoftwareSerial bluetooth(rx_pin, tx_pin);

// Watchdog timer interrupt service routing
ISR(WDT_vect)
{
  // Do nothing, it's only used to wake up the CPU
}

// Functions used for the boost converter battery charging when the input "solar" voltage is greater than 4V
void update_cv_error_array()
{
  // This value keeps track of the oldest output voltage reading
  static int cv_error_index{}; // the static modifier is useful because we want to reference it in future function calls like a global variable.

  cv_error[cv_error_index] = top_ten_point_two - top_voltage;

  top_voltage = map(top_voltage, top_seven, top_ten, 7000, 10000);
  battery_voltage = map(battery_voltage, battery_seven, battery_ten, 7000, 10000);
  battery_current = (top_voltage - battery_voltage) / load_resistance;

  top_voltage /= 1000;
  battery_voltage /= 1000;

  // Update the error_index according to the error array length error_length
  if (!(cv_error_index < cv_error_length))
  {
    cv_error_index = 0;
  }
  else
  {
    cv_error_index++;
  }
}

// Functions used for the boost converter battery-charging process when awake (Vin >= 4V)
void update_cc_error_array()
{
  // This value keeps track of the oldest output voltage reading
  static int cc_error_index{}; // the static modifier is useful because we want to reference it in future function calls like a global variable.

  top_voltage = map(top_voltage, top_seven, top_ten, 7000, 10000);
  battery_voltage = map(battery_voltage, battery_seven, battery_ten, 7000, 10000);
  battery_current = (top_voltage - battery_voltage) / load_resistance;

  top_voltage /= 1000;
  battery_voltage /= 1000;

  cc_error[cc_error_index] = cc_set_point - battery_current;

  // Update the error_index according to the error array length error_length
  if (!(cc_error_index < cc_error_length))
  {
    cc_error_index = 0;
  }
  else
  {
    cc_error_index++;
  }
}

void update_cc_error_mean()
{
  cc_mean = 0;

  for (int i{}; i < cc_error_length; i++)
  {
    cc_mean += cc_error[i];
  }

  cc_mean /= cc_error_length;
}

void update_cv_error_mean()
{
  cv_mean = 0;

  for (int i{}; i < cv_error_length; i++)
  {
    cv_mean += cv_error[i];
  }

  cv_mean /= cv_error_length;
}

void wipe_cc_array()
{
  for (int i{}; i < cc_error_length; i++)
  {
    cc_error[i] = 0;
  }
  cc_mean = 0;
}

void wipe_cv_array()
{
  for (int i{}; i < cv_error_length; i++)
  {
    cv_error[i] = 0;
  }
  cv_mean = 0;
}

void update_cc_duty_cycle()
{
  surrogate_d_current += (cc_mean * cc_integral_gain);
  if (limit_duty_cycle)
  {
    d_current = constrain(surrogate_d_current, d_min, d_max);
  }
  else
  {
    d_current = surrogate_d_current;
  }

  analogWrite(9, d_current);
}

void update_cv_duty_cycle()
{
  surrogate_d_current += (cv_mean * cv_integral_gain);
  if (limit_duty_cycle)
  {
    d_current = constrain(surrogate_d_current, d_min, d_max);
  }
  else
  {
    d_current = surrogate_d_current;
  }

  analogWrite(9, d_current);
}

double measure_temperature(int thermistor_pin)
{
  // Read the voltage on the thermistor pin
  double v_thermistor{};
  v_thermistor = analogRead(thermistor_pin);
  v_thermistor = map(v_thermistor, 0, 1023, 0, 50000);
  v_thermistor /= 10000;

  // Calculate the thermistor resistance
  double r_thermistor{};
  r_thermistor = (v_thermistor * r_p) / (5 - v_thermistor);

  // Calculate the ambient temperature
  double temperature{};
  temperature = A + (B * log(r_thermistor)) + (C * pow(log(r_thermistor), 3));
  temperature = pow(temperature, -1);
  temperature -= 273.15;

  return temperature;
}

// New as of milestone 4: A function that checks if the input voltage is in the charging range
bool vin_is_at_least_4V()
{
  return (analogRead(v_in_pin) >= v_in_4V);
}

// New as of milestone 4: One 30s sleep period composed of individual 8-8-8-4-2 second sleeps
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
  MCUCR |= _BV(BODS) | _BV(BODSE);        // Set these 2 bits to start a timed config process much like with the WDT
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS; // BODSE needs to be cleared, but BODS stays set.

  // 3 consecutive 8-second sleep periods (SLEEP instruction using the inline assembler)
  // We need to take care of the BOD for each sleep period
  __asm__ __volatile__("sleep"
                       "\n\t" ::); // Round 1

  __asm__ __volatile__("wdr"); // WDT reset instruction
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__("sleep"
                       "\n\t" ::); // Round 2

  __asm__ __volatile__("wdr");
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__("sleep"
                       "\n\t" ::); // Round 3

  // Now we reconfigure the WDT to timeout in 4 seconds (resets it in the process)
  cli();                          // Stop all interrupts
  WDTCSR = _BV(WDE) | _BV(WDCE);  // Set watchdog enable and change enable (starts the timed config process)
  WDTCSR = _BV(WDP3) | _BV(WDIE); // Set the prescaler to 512K (4s) and set the WDT mode to interrupt only (no reset)
  sei();                          // Enable interrupts once again

  // Go to sleep for 4 seconds
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__("sleep"
                       "\n\t" ::);

  // Now we reconfigure the WDT to timeout in 2 seconds (resets it in the process)
  cli();                                      // Stop all interrupts
  WDTCSR = _BV(WDE) | _BV(WDCE);              // Set watchdog enable and change enable (starts the timed config process)
  WDTCSR = _BV(WDP2) | _BV(WDP1) | _BV(WDIE); // Set the prescaler to 128K (2s) and set the WDT mode to interrupt only (no reset)
  sei();                                      // Enable interrupts once again

  // Go to sleep for 2 seconds (last sleep period)
  MCUCR |= _BV(BODS) | _BV(BODSE);
  MCUCR = (MCUCR & ~(_BV(BODSE))) | BODS;
  __asm__ __volatile__("sleep"
                       "\n\t" ::);

  // Disable sleep mode
  SMCR &= ~(_BV(SE));

  // Enable the ADC (important for temp measurement and boost converter)
  ADCSRA |= _BV(ADEN);
}

void stop_boost_pwm()
{
  analogWrite(9, LOW);
  TCCR1B &= ~(_BV(CS12) | _BV(CS11) | _BV(CS10)); // Clear the Timer1 clock-select bits. (no clock source)
}

void start_boost_pwm()
{
  TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10); // Enable the Timer1 clock source without any prescaling
  analogWrite(9, d_min);
  delay(10); // Let the boost converter output voltage approach steady state.
}

void awake_routine()
{
  __asm__ __volatile__("wdr"); // WDT reset instruction
  current_millis = millis();

  if (!((current_millis - previous_error_millis) < error_update_period)) // Executes if an entire error_update_period has passed
  {
    // Update the 2 output voltage values
    top_voltage = analogRead(top_pin);
    battery_voltage = analogRead(battery_pin);

    // Enter the charging-scheme finite state machine

    switch (current_charging_scheme)
    {
    case CONSTANT_CURRENT:
      update_cc_error_array();
      update_cc_error_mean();
      if (battery_voltage < 10.2)
      {
        previous_charge_mode_change_millis = current_millis;
      }
      else if (!((current_millis - previous_charge_mode_change_millis) < mode_change_cooldown_duration))
      {
        // Change from CC to CV charging mode
        current_charging_scheme = CONSTANT_VOLTAGE;
        wipe_cc_array();
        previous_charge_mode_change_millis = current_millis;
      }
      break;

    case CONSTANT_VOLTAGE:
      update_cv_error_array();
      update_cv_error_mean();
      if (battery_voltage > 10.15)
      {
        previous_charge_mode_change_millis = current_millis;
      }
      else if (!((current_millis - previous_charge_mode_change_millis) < mode_change_cooldown_duration))
      {
        // Change from CV to CC charging mode
        current_charging_scheme = CONSTANT_CURRENT;
        wipe_cv_array();
        previous_charge_mode_change_millis = current_millis;
      }
      break;
    }

    previous_error_millis = current_millis;
  }

  if (!((current_millis - previous_duty_cycle_millis) < duty_cycle_update_period)) // Executes if an entire duty_cycle_update_period has passed
  {
    if (current_charging_scheme == CONSTANT_CURRENT)
    {
      update_cc_duty_cycle();
    }
    else
    {
      update_cv_duty_cycle();
    }

    previous_duty_cycle_millis = current_millis;
  }

  if (!((current_millis - previous_temperature_millis) < temperature_update_period)) // Executes if an entire temperature_update_period has passed
  {
    // Update the temperature
    current_temperature = measure_temperature(thermistor_pin);
    temp_updated = true;

    previous_temperature_millis = current_millis;
  }

  if (!((current_millis - previous_print_message_millis) < print_message_period)) // Executes if an entire print_message_period has passed
  {

    // Send the most recent temperature data to the bluetooth module
    if (temp_updated)
    {
      bluetooth.print("Temp: ");
      bluetooth.print(current_temperature, 2);
      bluetooth.print("C\n");
      temp_updated = false;
    }

    previous_print_message_millis = current_millis;
  }

  if (vin_is_at_least_4V())
  {
    current_power_mode = ASLEEP;
    stop_boost_pwm();
  }
}

void asleep_routine()
{
  // Update temperature
  current_temperature = measure_temperature(thermistor_pin);

  // Send the temperature data over bluetooth
  bluetooth.print("Temp: ");
  bluetooth.print(current_temperature, 2);
  bluetooth.print("C\n");

  // Sleep for 30 seconds
  sleep_30s();

  if (vin_is_at_least_4V())
  {
    // Make the WDT run in reset mode
    cli();                                     // Stop all interrupts
    WDTCSR = _BV(WDE) | _BV(WDCE);             // Set watchdog enable and change enable (starts the timed config process)
    WDTCSR = _BV(WDP3) | _BV(WDP0) | _BV(WDE); // Set the prescaler to 256K (2s) and set the WDT mode to reset only mode
    sei();                                     // Enable interrupts once more

    current_power_mode = AWAKE;
    start_boost_pwm();
  }
}

void setup()
{
  // Set the MCU sleep mode to "power down" mode. It's the only mode we use.
  SMCR = (SMCR & ~(_BV(SM0) | _BV(SM1) | _BV(SM2))) | _BV(SM1);

  // Timer1 configuration for our boost converter PWM
  TCCR1A = _BV(COM1A1) | _BV(WGM11); // Set the wavegen mode to "fast PWM" (mode 14) and set the output compare mode to "non-inverting" (mode 2) (UNO board pin 9)
  TCCR1B = _BV(WGM13) | _BV(WGM12);  // The 3 other wavegen bits are in this second Timer1 register. We don't give it a clock source.
  ICR1 = 290;                        // Important for setting the PWM frequency of 55kHz

  // Bluetooth radio connection
  bluetooth.begin(9600);

  if (vin_is_at_least_4V())
  {
    // Start the WDT in system reset mode
    cli();                                     // Stop all interrupts
    WDTCSR = _BV(WDE) | _BV(WDCE);             // Set watchdog enable and change enable (starts the timed config process)
    WDTCSR = _BV(WDP3) | _BV(WDP0) | _BV(WDE); // Set the prescaler to 256K (2s) and set the WDT mode to reset only mode
    sei();                                     // Enable interrupts once more

    current_power_mode = AWAKE;
    start_boost_pwm();
  }
  else
  {
    current_power_mode = ASLEEP;
  }
}

void loop()
{
  // Power mode FSM
  switch (current_power_mode)
  {
  case ASLEEP:
    asleep_routine();
    break;

  case AWAKE:
    awake_routine();
    break;
  }
}