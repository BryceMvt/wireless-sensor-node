// Master bluetooth module

#include <Arduino.h>
#include <SoftwareSerial.h>

// Pins
const int master_rx_pin{7};
const int master_tx_pin{8};

const int v_in_pin = A3;
const int v_in_4V = 823;

SoftwareSerial BTMaster(master_rx_pin, master_tx_pin);

String BTMessage{};
int reading{};


bool vin_is_at_least_4V()
{
  return (analogRead(v_in_pin) >= v_in_4V);
}

void setup()
{
  //BTMessage = "temperature";
  //Serial.begin(9600);
  BTMaster.begin(9600);
}

void loop()
{
  reading = analogRead(A3);
  BTMaster.print("Analog Reading: ");
  BTMaster.print(reading);

  if (vin_is_at_least_4V())
  {
    BTMaster.print(" (enough to charge!)");
  }
  
  BTMaster.print("\n");
  //Serial.println(BTMessage);

  delay(5000);
  
}
