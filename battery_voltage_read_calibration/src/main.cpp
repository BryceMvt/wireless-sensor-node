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
  raw_value = analogRead(A0);
  voltage_value = map(raw_value, 500, 715, 700, 1000);
  voltage_value /= 100;

  // Display our message on the serial monitor
  Serial.print("Raw 10-bit value: ");
  Serial.print(raw_value);

  Serial.print("\t\tCalculated Voltage: ");
  Serial.print(voltage_value);
  Serial.print("V\n");

  // Wait a reasonable amount of time before remeasuring
  delay(1000);
}

// Bryce's lab-kit UNO with a 100k + 180k ohm voltage divider:
// - 7.00V  source analogRead() output is 502
// - 10.50V source analogRead() output is 753
