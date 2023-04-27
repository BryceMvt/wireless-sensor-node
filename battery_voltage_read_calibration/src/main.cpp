// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 02/08/23
// Use this sketch to determine the 10-bit values displayed on the serial monitor corresponding to an applied 7V and an applied 10.5V
// Change the "monitor_port" inside platformio.ini for your specific computer's USB port to use the serial monitor

#include <Arduino.h>

double raw_value{};
double voltage_value{};

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
  // Collect data from analog input pin 0
  raw_value = analogRead(A3);
  voltage_value = map(raw_value, 479, 652, 400, 500);
  voltage_value /= 100;

  // Display our message on the serial monitor
  Serial.print("Raw 10-bit value: ");
  Serial.print(raw_value, 0);

  //Serial.print("\t\tCalculated Voltage: ");
  //Serial.print(voltage_value);
  //Serial.print("V\n");
  Serial.print("\n");

  // Wait a reasonable amount of time before remeasuring
  delay(1000);
}

// Bryce's lab-kit UNO with a 100k + 180k ohm voltage divider:
// - 7.00V  source analogRead() output is 502
// - 10.50V source analogRead() output is 753

// Milestone 3,4 with zener diodes
// - A0: 7V = 468, 10V = 630, 10.2V = 638
// - A1: 7V = 479, 10V = 652, 10.2V = 662
