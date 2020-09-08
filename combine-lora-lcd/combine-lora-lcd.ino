#include <SPI.h>
#include <LoRa.h>

#include <SoftwareSerial.h> 
#include <TinyGPS.h> 
float lat = 28.5458,lon = 77.1703; // create variable for latitude and longitude object  

#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 4; 
const int pin_EN = 5; 
const int pin_d4 = 0; 
const int pin_d5 = 1; 
const int pin_d6 = A5; 
const int pin_d7 = 3; 

LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);


SoftwareSerial gpsSerial(A1,A2);//rx,tx 
TinyGPS gps; // create gps object 


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
    lcd.clear(); 
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






while(gpsSerial.available()){ // check for gps data 
//  Serial.println("gps data");
  if(gps.encode(gpsSerial.read()))// encode gps data 
  {  
  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  // display position 
//  lcd.clear(); 
//  lcd.setCursor(1,0); 
//  lcd.print("GPS Signal"); 
//  Serial.print("Position: "); 
//  Serial.print("Latitude:"); 
//  Serial.print(lat,6); 
//  Serial.print(";"); 
//  Serial.print("Longitude:"); 
//  Serial.println(lon,6);  
//  lcd.setCursor(1,0); 
//  lcd.print("LAT:"); 
//  lcd.setCursor(5,0); 
//  lcd.print(lat); 
  //Serial.print(lat); 
  //Serial.print(" "); 
//  lcd.setCursor(0,1); 
//  lcd.print(",LON:"); 
//  lcd.setCursor(5,1); 
//  lcd.print(lon); 
 } 
}











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
