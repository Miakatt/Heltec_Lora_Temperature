/*
Send BMP180 temperature data via Lora. 
Read with OLED LoRa Receiver.ino from examples. 
This is a mashup of the BMP180 example and the OLED LoRa Sender example.
BMP180 sensor is soldered to Pins 4 (SDA) and 15 (SCL), sharing I2C with the OLED. 
*/
#include "Arduino.h"
#include <BMP180.h>
#include <Wire.h>
#include "heltec.h"
#include "images.h"
#include "string.h"

BMP085 bmp;

#define BAND    868E6  //you can set band here directly,e.g. 868E6,915E6

unsigned int counter = 0;
String rssi = "RSSI --";
String packSize = "--";
String packet ;


void setup()
{
   //WIFI Kit series V1 not support Vext control
  Heltec.begin(true /*DisplayEnable Enable*/, true /*Heltec.Heltec.Heltec.LoRa Disable*/, true /*Serial Enable*/, true /*PABOOST Enable*/, BAND /*long BAND*/);

  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);
  delay(1000);
}

void loop()
{
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  delay(1000);
  while (1) {}
  }
  Heltec.display->clear(); 
  double T = bmp.readTemperature();

// Write value to OLED
  Heltec.display->drawString(0, 0, "Sending temp: ");
  Heltec.display->drawString(0, 10, String(T));
  Heltec.display->drawString(6, 10, "C");
  Heltec.display->display();
  // send packet
  LoRa.beginPacket();
  
/*
 * LoRa.setTxPower(txPower,RFOUT_pin);
 * txPower -- 0 ~ 20
 * RFOUT_pin could be RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 *   - RF_PACONFIG_PASELECT_PABOOST -- LoRa single output via PABOOST, maximum output 20dBm
 *   - RF_PACONFIG_PASELECT_RFO     -- LoRa single output via RFO_HF / RFO_LF, maximum output 14dBm
*/
  LoRa.setTxPower(16,RF_PACONFIG_PASELECT_PABOOST);
  LoRa.print("Temp: ");
  LoRa.print(T);
  LoRa.endPacket();
  Serial.print("Temp: ");
  Serial.println(T);
  Serial.println();
 
  counter++;
  digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
