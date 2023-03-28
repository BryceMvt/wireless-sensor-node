// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 03/27/23
// Thermistor temperature reader for Milestone 3 Deliverable 3

#include <Arduino.h>

// Our thermistor circuit is a voltage divider composed of our thermistor and a fixed resistor.
// Our thermistor is connected from ground to out analog input pin of choice. Our fixed resistor is connected from 5V to the same analog input pin.
const int thermistor_pin{A2};

// Constants used for our Steinhart-Hart equation
const double A{0.0002842};
const double B{0.0003677};
const double C{-0.0000004056};

// Our fixed resistor value
const long r_p{47000};

// Real-time value to store our temperature
double current_temperature{};

double measureTemperature(int thermistor_pin)
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
  Serial.begin(9600);

  while (!Serial) {
    delay(10);
    // Needs to wait a little bit! 
  }
}

void loop()
{
  // Measure the temperature
  current_temperature = measureTemperature(thermistor_pin);

  // Display the temperature on the serial monitor
  Serial.print("Temperature: ");
  Serial.print(current_temperature); // 3 decimal places
  Serial.print(" C\n");

  delay(200);
}