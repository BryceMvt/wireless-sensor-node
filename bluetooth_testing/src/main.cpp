// Master bluetooth module

#include <Arduino.h>
#include <SoftwareSerial.h>

// Pins
const int master_rx_pin{2};
const int master_tx_pin{3};

SoftwareSerial BTMaster(master_rx_pin, master_tx_pin);

String BTMessage{};


void setup()
{
  BTMessage = "temperature";
  Serial.begin(9600);
  BTMaster.begin(9600);
}

void loop()
{
  BTMaster.println(BTMessage);
  Serial.println(BTMessage);

  delay(1000);
  
}
