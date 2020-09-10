#include <SPI.h>
#include <LoRa.h>

#include <SoftwareSerial.h> 
#include <TinyGPS.h>
float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object   

SoftwareSerial gpsSerial(A0,A1);//rx,tx 
TinyGPS gps; // create gps object 

int counter = 0;

void setup() {

  gpsSerial.begin(9600);
  
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

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

while (gpsSerial.available() > 0)
    Serial.write(gpsSerial.read());

  Serial.print("Sending packet: ");
  Serial.println(counter);

  // send packet
  LoRa.beginPacket();
  LoRa.print("asdf");
//  LoRa.print(counter);
  LoRa.endPacket();

  counter++;

  delay(5000);

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
