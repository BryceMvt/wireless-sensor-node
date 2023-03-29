// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 03/28/23
// Closed-loop constant current and constant voltage battery charging circuit
// The variables update_period, error_length, and integral gain can be used to tune the behavior of our integral feedback loop

#include <Arduino.h>

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

// ADC voltage divider reference numbers
const int top_ten_point_two{730}; // 10.2V
const int top_ten{715};           // 10V
const int top_seven{500};         // 7V

const int battery_ten_point_two{734}; // 10.2V
const int battery_ten{703};           // 10V
const int battery_seven{491};         // 7V

// Temperature
double current_temperature{};               // degrees C
const int temperature_update_period{30000}; // ms

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
const double cc_integral_gain{0.04};
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

// Real-time voltages
double top_voltage{};
double battery_voltage{};

// This function replaces the oldest output-voltage error reading with a fresh one
void update_cv_error_array()
{
  // This value keeps track of the oldest output voltage reading
  static int cv_error_index{}; // the static modifier is useful because we want to reference it in future function calls like a global variable.

  int top_voltage = analogRead(top_pin);
  cv_error[cv_error_index] = top_ten_point_two - top_voltage;

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

void update_cc_error_array()
{
  // This value keeps track of the oldest output voltage reading
  static int cc_error_index{}; // the static modifier is useful because we want to reference it in future function calls like a global variable.

  double battery_current{};

  top_voltage = map(top_voltage, top_seven, top_ten, 7000, 10000);
  battery_voltage = map(battery_voltage, battery_seven, battery_ten, 7000, 10000);

  battery_current = (top_voltage - battery_voltage) / load_resistance;

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

void setup()
{

  //Serial.begin(115200);

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
  //  - For our prescale N = 1 and f_I/O = 16MHz, we write TOP = 290 to the register.
  //  - Because we quantize our TOP value to the nearest integer value, our actual frequency is more like 54.983kHz. Exact frequency is not important for this application.
  ICR1 = 290;

  // Using the analogWrite() is a nice abstraction that has the same effect as manually setting up a pin's output compare register.
  // Duty Cycle D = (second argument of analogWrite + 1) / (TOP + 1)
  // We initially set this to d_min, the lowest duty cycle we allow.
  analogWrite(9, d_min);

  // Give the (currently open-loop) output signal to approach steady state
  // According to our LTspice simulation waveforms, we need at least 1.2ms for this to happen
  delay(10);
}

void loop()
{
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
      if (battery_voltage < battery_ten_point_two)
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
      if (battery_voltage > battery_ten_point_two)
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

    previous_temperature_millis = current_millis;
  }
}