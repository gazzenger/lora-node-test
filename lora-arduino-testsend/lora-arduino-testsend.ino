#include <SPI.h>
#include <LoRa.h>

#include <SoftwareSerial.h> 
#include <TinyGPS.h>
float lat = 28.5458,lon = 77.1703, elv = 0; // create variable for latitude and longitude object   

SoftwareSerial gpsSerial(A0,A1);//rx,tx 
TinyGPS gps; // create gps object 

int counter = 0;

static void smartdelay(unsigned long ms);
static void print_date(TinyGPS &gps);
static void print_int(unsigned long val, unsigned long invalid, int len);

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

//while (gpsSerial.available() > 0)
//    Serial.write(gpsSerial.read());

//  Serial.print("Sending packet: ");
//  Serial.println(counter);

  // send packet
//  LoRa.beginPacket();
//  LoRa.print("asdf");
//  LoRa.print(counter);
//  LoRa.endPacket();

//  counter++;




  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  elv = gps.f_altitude();
  
  // display position 
  Serial.print("Position: "); 
  Serial.print("Latitude:"); 
  Serial.print(lat,6); 
  Serial.print(";"); 
  Serial.print("Longitude:"); 
  Serial.println(lon,6);  
  Serial.println(elv,1);
  print_date(gps);  



  LoRa.beginPacket();
  LoRa.print(lat,3);
  LoRa.print(",");
  LoRa.print(lon,3);
  LoRa.print(",");
  LoRa.print(elv,1);
//  LoRa.print(counter);
  LoRa.endPacket();


  
  smartdelay(5000);



  

}





static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}




static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}



static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
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
