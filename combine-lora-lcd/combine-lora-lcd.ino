#include <SPI.h>
#include <LoRa.h>


#include <SoftwareSerial.h> 
#include <TinyGPS.h> 
float lat = 28.5458,lon = 77.1703, elv = 0;  // create variable for latitude and longitude object  


#include <LiquidCrystal.h>
//LCD pin to Arduino
const int pin_RS = 4; 
const int pin_EN = 5; 
const int pin_d4 = A3; 
const int pin_d5 = A4; 
const int pin_d6 = A5; 
const int pin_d7 = 3; 

LiquidCrystal lcd( pin_RS,  pin_EN,  pin_d4,  pin_d5,  pin_d6,  pin_d7);


SoftwareSerial gpsSerial(A2,A1);//rx,tx 
TinyGPS gps; // create gps object 


int counter = 0;

static void smartdelay(unsigned long ms);
static void print_date(TinyGPS &gps);
static void print_int(unsigned long val, unsigned long invalid, int len);


#define MAX_WORLD_COUNT 5
#define MIN_WORLD_COUNT 2
char *Words[MAX_WORLD_COUNT];




void setup() {

 lcd.begin(16, 2);
// lcd.setCursor(0,0);
// lcd.print("Electropeak.com");
// lcd.setCursor(0,1);
// lcd.print("Press Key:");


  gpsSerial.begin(9600);
  
  Serial.begin(9600);
  while (!Serial);
//
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




//Button Functions
//Serial.println(analogRead(A0));
//  delay(100);

//Button 1
//177 <= analogRead(A0) <= 180 
//Button 2
//509 <= analogRead(A0) <= 511
//Button 3
//697 <= analogRead(A0) <= 699




char lorastr[21];
char recvstr[21];

byte i=0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    // read packet

    while (LoRa.available()) { 
      lorastr[i] = (char)LoRa.read();
      recvstr[i] = lorastr[i];
      i++;
    }

lorastr[i] = '\0';
recvstr[i] = '\0';

split_message(lorastr);

Serial.println(Words[0]);
Serial.println(Words[1]);
Serial.println(Words[2]);
    
    lcd.clear(); 
    lcd.setCursor(0,0);
    lcd.print(Words[0]);
    lcd.print(Words[1]);
    lcd.print(Words[2]);

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




//GPS Stuff

//while (gpsSerial.available() > 0)
//    Serial.write(gpsSerial.read());

//while(gpsSerial.available()){ // check for gps data 
//  Serial.println("gps data");
//  if(gps.encode(gpsSerial.read()))// encode gps data 
//  {  
  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  elv = gps.f_altitude();
  
  // display position 
//  lcd.clear(); 
//  lcd.setCursor(0,0); 
//  lcd.print(lat,3);  
//  lcd.setCursor(8,0); 
//  lcd.print(elv,1); 
//  lcd.setCursor(0,1); 
//  lcd.print(lon,3); 

  Serial.print("Position: "); 
  Serial.print("Latitude:"); 
  Serial.print(lat,6); 
  Serial.print(";"); 
  Serial.print("Longitude:"); 
  Serial.println(lon,6);  
  Serial.println(elv,1);
  print_date(gps);  
// } 
 smartdelay(1000);
//}

}





////////// ////////// ////////// ////////// ////////// ////////// //////////
// Split string into individual words and store each word into an array
// this function return word_count (number of words)
////////// ////////// ////////// ////////// ////////// ////////// //////////
byte split_message(char* str) {
  byte word_count = 0; //number of words
  char * item = strtok (str, " ,"); //getting first word (uses space & comma as delimeter)

  while (item != NULL) {
    if (word_count >= MAX_WORLD_COUNT) {
      break;
    }
    Words[word_count] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    word_count++;
  }
  return  word_count;
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
