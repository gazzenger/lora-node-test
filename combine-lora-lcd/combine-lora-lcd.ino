#include <SPI.h>
#include <LoRa.h>
#include <math.h>

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


struct coord
{
  float lat;
  float lon;
  float elv;
};

typedef struct coord Coord;

Coord initial;
Coord final;



struct point
{
  float x;
  float y;
  float z;
  float radius;
  float nx;
  float ny;
  float nz;
};

typedef struct point Point;



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




char lorastr[30];

byte i=0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    // read packet

    while (LoRa.available()) { 
      lorastr[i] = (char)LoRa.read();
      i++;
    }

lorastr[i] = '\0';


split_message(lorastr);

Serial.println(Words[0]);
Serial.println(Words[1]);
Serial.println(Words[2]);
    
    lcd.clear(); 
    lcd.setCursor(0,0);
//    lcd.print(Words[0]);
//    lcd.print(Words[1]);
//    lcd.print(Words[2]);

initial.lat = (float)strtod(Words[0],NULL);
initial.lon = (float)strtod(Words[1],NULL);
initial.elv = (float)strtod(Words[2],NULL);
    
lcd.print(Calculate()*1000,8);

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

final.lat = lat;
final.lon = lon;
final.elv = elv;
  
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






//GPS Distance calculations

    float EarthRadiusInMeters(float latitudeRadians)
    {
        // latitudeRadians is geodetic, i.e. that reported by GPS.
        // http://en.wikipedia.org/wiki/Earth_radius
        float a = 6378137.0;  // equatorial radius in meters
        float b = 6356752.3;  // polar radius in meters
        float cosrad = cos(latitudeRadians);
        float sinrad = sin(latitudeRadians);
        float t1 = a * a * cosrad;
        float t2 = b * b * sinrad;
        float t3 = a * cosrad;
        float t4 = b * sinrad;
        return sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4));
    }

    float  GeocentricLatitude(float latitude)
    {
        // Convert geodetic latitude 'lat' to a geocentric latitude 'clat'.
        // Geodetic latitude is the latitude as given by GPS.
        // Geocentric latitude is the angle measured from center of Earth between a point and the equator.
        // https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
        float e2 = 0.00669437999014;
        float clat = atan((1.0 - e2) * tan(latitude));
        return clat;
    }

Point LocationToPoint(Coord c)
    {
        // Convert (lat, lon, elv) to (x, y, z).
        float latitude = c.lat * PI / 180.0;
        float longitude = c.lon * PI / 180.0;
        float radius = EarthRadiusInMeters(latitude);
        float clat   = GeocentricLatitude(latitude);

        float cosLon = cos(longitude);
        float sinLon = sin(longitude);
        float cosLat = cos(clat);
        float sinLat = sin(clat);
        float x = radius * cosLon * cosLat;
        float y = radius * sinLon * cosLat;
        float z = radius * sinLat;

        // We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
        // Now we use geodetic latitude to calculate normal vector from the surface, to correct for elevation.
        float cosGlat = cos(latitude);
        float sinGlat = sin(latitude);

        float nx = cosGlat * cosLon;
        float ny = cosGlat * sinLon;
        float nz = sinGlat;

        x += c.elv * nx;
        y += c.elv * ny;
        z += c.elv * nz;

        Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.radius = radius;
        point.nx = nx;
        point.ny = ny;
        point.nz = nz;

        return point;
    }

    float Distance (Point ap,Point bp)
    {
        float dx = ap.x - bp.x;
        float dy = ap.y - bp.y;
        float dz = ap.z - bp.z;
        return sqrt (dx*dx + dy*dy + dz*dz);
    }

    Point RotateGlobe (Coord b, Coord a,float bradius,float aradius)
    {
        // Get modified coordinates of 'b' by rotating the globe so that 'a' is at lat=0, lon=0.
        Coord br;
        br.lat = b.lat;
        br.lon= (b.lon - a.lon);
        br.elv=b.elv;
        
        Point brp = LocationToPoint(br);

        // Rotate brp cartesian coordinates around the z-axis by a.lon degrees,
        // then around the y-axis by a.lat degrees.
        // Though we are decreasing by a.lat degrees, as seen above the y-axis,
        // this is a positive (counterclockwise) rotation (if B's longitude is east of A's).
        // However, from this point of view the x-axis is pointing left.
        // So we will look the other way making the x-axis pointing right, the z-axis
        // pointing up, and the rotation treated as negative.

        float alat = GeocentricLatitude(-a.lat * PI / 180.0);
        float acosrad = cos(alat);
        float asinrad = sin(alat);

        float bx = (brp.x * acosrad) - (brp.z * asinrad);
        float by = brp.y;
        float bz = (brp.x * asinrad) + (brp.z * acosrad);

        Point point;
        point.x = bx;
        point.y = by;
        point.z = bz;
        point.radius = bradius;

        return point;
    }

Point NormalizeVectorDiff(Point b,Point a)
    {
      Point point;      
        // Calculate norm(b-a), where norm divides a vector by its length to produce a unit vector.
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float dz = b.z - a.z;
        float dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 == 0) {
            return point;
        }
        float dist = sqrt(dist2);
        point.x = (dx/dist);
        point.y = (dy/dist);
        point.z = (dz/dist);
        point.radius = 1.0;
        return point;
    }



    float Calculate()
    {

                Point ap = LocationToPoint(initial);
                Point bp = LocationToPoint(final);
                float distKm = 0.001 * Distance(ap,bp);
                return distKm;


//                // Let's use a trick to calculate azimuth:
//                // Rotate the globe so that point A looks like latitude 0, longitude 0.
//                // We keep the actual radii calculated based on the oblate geoid,
//                // but use angles based on subtraction.
//                // Point A will be at x=radius, y=0, z=0.
//                // Vector difference B-A will have dz = N/S component, dy = E/W component.
//                var br = RotateGlobe (b, a, bp.radius, ap.radius);
//                if (br.z*br.z + br.y*br.y > 1.0e-6) {
//                    var theta = Math.atan2(br.z, br.y) * 180.0 / Math.PI;
//                    var azimuth = 90.0 - theta;
//                    if (azimuth < 0.0) {
//                        azimuth += 360.0;
//                    }
//                    if (azimuth > 360.0) {
//                        azimuth -= 360.0;
//                    }
//                    $('div_Azimuth').innerHTML = azimuth.toFixed(4) + '&deg;';
//                }
//
//                var bma = NormalizeVectorDiff(bp, ap);
//                if (bma != null) {
//                    // Calculate altitude, which is the angle above the horizon of B as seen from A.
//                    // Almost always, B will actually be below the horizon, so the altitude will be negative.
//                    // The dot product of bma and norm = cos(zenith_angle), and zenith_angle = (90 deg) - altitude.
//                    // So altitude = 90 - acos(dotprod).
//                    var altitude = 90.0 - (180.0 / Math.PI)*Math.acos(bma.x*ap.nx + bma.y*ap.ny + bma.z*ap.nz);
//                    $('div_Altitude').innerHTML = altitude.toFixed(4).replace(/-/g,'&minus;') + '&deg;';
//                }
          
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
