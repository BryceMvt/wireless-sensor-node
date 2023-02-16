// Bryce Mooney, Pengcheng Su
// Last Modified: Bryce Mooney, 02/08/23
// This program is the basis of our completed wireless sensor node's ability to determine the voltage level of the connected 9V battery.
// Our battery voltage is read and returned to the serial monitor.
// We use a voltage divider appropriately connected to battery-positive, analog input pin 0, and ground.
// This mitigates our MCU's ADC voltage input range limitation of 0 to 5V since we need to collect meaningful data from our battery's voltage which can range from 7 to 10.5V.
// We are able to calculate the original battery voltage from the attenuated voltage the ADC receives because the voltage-divider's resistance values are known.

#include <Arduino.h>

// Constants
const int batteryAnalogInputPin{A0};
const int batteryR1{180}; // The resistance value of the resistor connected from battery positive to the analog input pin in kΩ
const int batteryR2{100}; // The resistance value of the resistor connected from ground to the analog input pin in kΩ



// Global Variables


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(analogRead(A0));
  delay(200);
}