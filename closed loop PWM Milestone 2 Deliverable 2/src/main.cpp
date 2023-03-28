// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 02/21/23
// Closed-loop PWM integral controller for Milestone 2 Deliverable 2
// The variables update_period, error_length, and integral gain can be used to tune the behavior of our integral feedback loop

#include <Arduino.h>

// Constants and Global Variables:

// We connect our zener diode-protected voltage reader subcircuit to this analog input pin:
const int vout_read_pin{A0};

// The update period of our feedback loop should be significantly longer than our boost converter's switching period
// to allow the converter system to respond to changes in duty cycle.
const int error_update_period{1};      // ms
const int duty_cycle_update_period{5}; // ms

// Number of elements in our error array
const int error_length{10};

// This is a multiplier that determines how sensitive our feedback loop is to small changes in measured voltage
const double integral_gain{0.04}; //0.04 is good for 1ms error and 5ms duty updates

// More elements yields greater precision, but consumes more response time and a higher risk of "integral wind-up"
// This array will store error_length number of past values of our voltage output readings
// Taking the mean of all elements in the error array is similar to integrating over a continuous voltage signal
double error[error_length]{};

// When using the analogRead() function for our voltage divider with 5.6V zener diode protection:
//  - The 10-bit value 616 maps to 10.00V (found using our DC bench power supply)
const int ten_volt_byte{618}; // 616, 618

//  - The 10-bit value 461 maps to  7.00V
const int seven_volt_byte{461}; // 461

// We must limit the allowed duty cycle values using the constrain function to avoid the possibility of getting a positive feedback.
// The following minimum and maximum allowed duty cycles were determined experimentally using our open-loop boost converter.
// They are in the form of integers we input to the second function argument of the analogWrite() function.
const int d_min = 145; // Corresponds to an approximate duty cycle of D = 0.52 (original 150)
const int d_max = 188; // Corresponds to an approximate duty cycle of D = 0.65

// We use the absolute timing functionality millis() of our Arduino board
// Absolute timing is inferior to timer interrupts because overflow can occur (after approximately 47 days).
// We are using millis() now because it is easier to implement than timer interrupts.
// This functionality will not be used in our final design.
unsigned long current_millis{};

unsigned long previous_error_millis{};
unsigned long previous_duty_cycle_millis{};

// Stores the most recently updated mean value of the error array
double current_mean{};

// Stores the most recently updated duty cycle integer used for the analogWrite() function
double surrogate_d_current{d_min};
int d_current{d_min};

// Other printable values:
double duty_percent{};
double perceived_voltage{};
int newest_analog_read{};

// Functions:

// This function replaces the oldest output-voltage error reading with a fresh one
void update_error_array()
{
  // This value keeps track of the oldest output voltage reading
  static int error_index{}; // the static modifier is useful because we want to reference it in future function calls like a global variable.

  newest_analog_read = analogRead(vout_read_pin);
  error[error_index] = ten_volt_byte - newest_analog_read;

  // Update the error_index according to the error array length error_length
  if (!(error_index < error_length))
  {
    error_index = 0;
  }
  else
  {
    error_index++;
  }
}

void update_error_mean()
{
  double error_mean{};
  for (int i{}; i < error_length; i++)
  {
    error_mean += error[i];
  }

  error_mean /= error_length;

  current_mean = error_mean;
}

void update_duty_cycle()
{
  surrogate_d_current += (current_mean * integral_gain);
  d_current = constrain(surrogate_d_current, d_min, d_max);

  analogWrite(9, d_current);
}

void display_values()
{
  // Duty Cycle Percentage = 100 * (d_current + 1) / (TOP + 1)
  duty_percent = 100 * (d_current + 1) / 291;

  // Perveived output voltage: use the 7V and 10V reference points
  perceived_voltage = map(newest_analog_read, 461, 616, 700, 1000);
  perceived_voltage /= 100;

  Serial.print("Duty Cycle = ");
  Serial.print(duty_percent, 2); // rounded to 2 decimal places
  Serial.print("%\tPerceived Voltage = ");
  Serial.print(perceived_voltage);
  Serial.print("V\n");
}

void setup()
{

  Serial.begin(115200);

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

  // Give the (currently open-loop) output signal to approack steady state
  // According to our LTspice simulation waveforms, we need at least 1.2ms for this to happen
  delay(5);
}

void loop()
{

  current_millis = millis();

  if (!((current_millis - previous_error_millis) < error_update_period)) // Executes if an entire error_update_period has passed
  {
    // Take an output-voltage reading and replace the oldest reading with it:
    update_error_array();
    update_error_mean();

    previous_error_millis = current_millis;
  }

  if (!((current_millis - previous_duty_cycle_millis) < duty_cycle_update_period)) // Executes if an entire duty_cycle_update_period has passed
  // mistakenly error_duty_cycle the whole time
  {
    // Take an output-voltage reading and replace the oldest reading with it:
    update_duty_cycle();
    display_values();

    previous_duty_cycle_millis = current_millis;
  }
}