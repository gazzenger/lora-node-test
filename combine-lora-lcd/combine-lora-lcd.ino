#include <SPI.h>
#include <LoRa.h>



#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 4; 
const int pin_EN = 5; 
const int pin_d4 = 0; 
const int pin_d5 = 1; 
const int pin_d6 = A5; 
const int pin_d7 = 3; 

LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);





int counter = 0;

void setup() {

 lcd.begin(16, 2);
// lcd.setCursor(0,0);
// lcd.print("Electropeak.com");
// lcd.setCursor(0,1);
// lcd.print("Press Key:");



  
//  Serial.begin(9600);
//  while (!Serial);
//
//  Serial.println("LoRa Receiver");

  pinMode(13, OUTPUT);

  if (!LoRa.begin(915000000)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  else {
//      LoRa.setTxPower(20);
      LoRa.setSignalBandwidth(250e3);
      //LoRa.setFrequency(917200000);
      LoRa.setSpreadingFactor(12);
      LoRa.setCodingRate4(5);
//      LoRa.setPreambleLength(8);
//      LoRa.enableCrc();
  }
}

void loop() {


char str[9];
byte i=0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    // read packet

    while (LoRa.available()) { 
      str[i] = (char)LoRa.read();
//      Serial.print((char)LoRa.read());
      i++;
    }
    lcd.setCursor(0,0);
    lcd.print("Message: ");
    lcd.print(str);

     lcd.setCursor(0,1);
     lcd.print("RSSI: ");
     lcd.println(LoRa.packetRssi());
    
//Serial.print(" with RSSI ");
//    Serial.println(LoRa.packetRssi());

// lcd.setCursor(0,0);
// lcd.print("Electropeak.com");
// lcd.setCursor(0,1);
// lcd.print("Press Key:");
    
  }


//
//  Serial.print("Sending packet: ");
//  Serial.println(counter);
//
//  // send packet
//  LoRa.beginPacket();
////  LoRa.print("asdf ");
//  LoRa.print(counter);
//  LoRa.endPacket();
//
//  counter++;
//
//  delay(1000);

}



//void loop() {
//  // try to parse packet
//  int packetSize = LoRa.parsePacket();
//  if (packetSize) {
//    // received a packet
//    Serial.print("Received packet '");
//
//    // read packet
//    while (LoRa.available()) {
//      Serial.print((char)LoRa.read());
//    }
//
//    // print RSSI of packet
//    Serial.print("' with RSSI ");
//    Serial.println(LoRa.packetRssi());
//  }
//}
