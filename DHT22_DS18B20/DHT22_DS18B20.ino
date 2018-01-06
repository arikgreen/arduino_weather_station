#include <DHT.h>          // DHT11/DHT22/AM2302/RHT03
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
//#include "i2c_BMP280.h"
#include <Adafruit_BMP085.h>

#define VERSION "0.7.0"
#define DHTPIN 2          // DHT data pin
#define DSPIN 5           // DS18B20 data pin
#define BACKLIGHTPIN 3    // LCD backlight
#define BTNPIN 6          // button
#define PIRPIN 8          // PIR

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);
 
DHT dht;
OneWire ds(DSPIN);

//BMP280 bmp280;
Adafruit_BMP085 bmp180;

// begin ethernet configuration
// ethernet interface mac address, must be unique on the LAN
static byte mymac[] = { 0x74,0x69,0x69,0x2D,0x30,0x31 };
static byte myip[] = { 192,168,1,100 };

byte Ethernet::buffer[500];
BufferFiller bfill;
// end ethernet configuration

/*
 * backlightPeriod    - backlight time
 * readSensorPeriod   - the frequency of reading the sensors
 * scrollSpped        - display scrolling speed
 * h_prev             - previous humidity DHT22
 * tempDHT_prev       - previous temperature from DHT22 sensor
 * tempDS_prev        - previous temperature from DS18B20 sensor
 * addr[8]            - address of DS18B20 sensor
 * stringLength       - lenghth of second line
 * positionCounter    - cursor position in second line
 * prevXXXTime        - times of last action
 * content            - second line
 */

float tempDHT_prev, tempDS_prev;
unsigned short pascal_prev;
byte addr[8], h_prev, stringLength, positionCounter=0;
unsigned short backlightPeriod=10000, readSensorPeriod=60000, scrollSpeed=400;
unsigned long prevBacklightTime=0, nowTime=0, prevSensorTime=0, prevScrollTime=0;

String content;

// byte arrowUp[8] = {
//   B00100,
//   B01110,
//   B10101,
//   B00100,
//   B00100,
//   B00100,
//   B00100,
//   B11111
// };

void setup() {
  pinMode(BTNPIN,INPUT);
  pinMode(PIRPIN,INPUT);
  
  // lcd.createChar(0, arrowUp);
  lcd.begin(16,2);
  lcd.setBacklightPin(BACKLIGHTPIN,POSITIVE);
  lcd.setBacklight(1);
  lcd.backlight();
  
  if (ether.begin(sizeof Ethernet::buffer, mymac) == 0) {
    lcd.print(F("Failed to access"));
    lcd.setCursor(0,1);
    lcd.print(F("Eth controller"));
  } else {
    ether.staticSetup(myip);
  }

  dht.setup(DHTPIN);            // DHT sensor initialization
  ds.reset_search();
  
  while(ds.search(addr)) {
    if (addr[0] != 0x28)
      continue;
    if (OneWire::crc8(addr, 7) != addr[7]){
      lcd.print(F("DS:CRC is not valid!"));
      delay(4000);
    }
    delay(250);
  }
  lcd.print(F("Weather Station"));
  lcd.setCursor(0,1);
  lcd.print(VERSION);
  delay(2000);
  delay(dht.getMinimumSamplingPeriod());

  /*if (bmp280.initialize())
    Serial.println("Sensor found");
  else {
    Serial.println("Sensor missing");
    while (1) {}
  }

  // onetime-measure:
  bmp280.setEnabled(0);
  bmp280.triggerMeasurement(); */

  if (!bmp180.begin()) {
    lcd.clear();
    lcd.print(F("Could not find a"));
    lcd.setCursor(0,1);
    lcd.print(F("valid BMP180."));
    delay(4000);
  }
}

void loop(){
  /*
   * h        - humidity
   * tempDHT  - temperature form DHT
   * tempDS   - temperature form DS
   * pirPIN   - state on input PIR
   * btnPIN   - state on button
   * data     - data from DS18B20
   * type_s   - global variable from OneWire library
   */
  
  byte i, h, pirPin, btnPin;
  unsigned short pascal;
  float tempDHT, tempDS;
  word len = ether.packetReceive();
  word pos = ether.packetLoop(len);

  pirPin = digitalRead(PIRPIN);
  btnPin = digitalRead(BTNPIN);
  
  nowTime = millis();

  if(pirPin == HIGH || btnPin == HIGH){
    lcd.setBacklight(1);
    prevBacklightTime = nowTime;
  }

  if(nowTime - prevSensorTime >= readSensorPeriod || btnPin == HIGH || content.length() == 0) {
    lcd.setCursor(0,1);
    lcd.print(F("Read sensors.   "));
    prevSensorTime = nowTime;
    
    tempDS = readDS18B20() / 16.0;
    //fahrenheit = celsius * 1.8 + 32.0;
  
    tempDHT = dht.getTemperature();
    h = dht.getHumidity();
    pascal = bmp180.readPressure() / 100;
    // metersold = bmp180.readAltitude();
    
    if(abs(tempDS-tempDS_prev)>=0.5  || abs(tempDHT-tempDHT_prev)>=0.5 || abs(h-h_prev)>=2 || abs(pascal-pascal_prev) >= 2) {
      lcd.setBacklight(1);
      prevBacklightTime = millis();
    }

    tempDS_prev = tempDS;
    tempDHT_prev = tempDHT;
    h_prev = h;
    pascal_prev = pascal;

    content = "Humidity:" + String(h) + "%  Pressure:" + String(pascal) + "hPa "; // Altitude: " + String(metersold) + "m";
    stringLength = content.length();

    lcd.clear();
    if(tempDS <= -10)
      lcd.setCursor(0,0);
    else if(tempDS >= 10 || tempDS < 0)
      lcd.setCursor(1,0);
    else
      lcd.setCursor(2,0);
    
    lcd.print(tempDS);
    lcd.print((char)223);
    lcd.print(F("C"));
    if(tempDHT < 10)
      lcd.setCursor(10,0);
    else
      lcd.setCursor(9,0);
    lcd.print(tempDHT);
    lcd.print((char)223);
    lcd.print(F("C"));

    if(pos) // check if valid tcp data is received
      ether.httpServerReply(homePage(h,pascal,tempDS,tempDHT));  // send web page data
  }

  // display the scrolling second line
  lcd.setCursor(0,1);
  if(nowTime - prevScrollTime >= scrollSpeed) {
    prevScrollTime = nowTime;
    if(positionCounter > stringLength-16){
      lcd.print(content.substring(positionCounter,stringLength));
      lcd.print(" ");
      lcd.print(content.substring(0,15-(stringLength-positionCounter)));
    } else {
      lcd.print(content.substring(positionCounter,16+positionCounter));
    }
    if(positionCounter == stringLength) {
      positionCounter = 0;
    } else {
      positionCounter++;
    }
  }
  // end scrolling

  if(nowTime - prevBacklightTime >= backlightPeriod) lcd.setBacklight(0);
  
  /*bmp280.awaitMeasurement();

  float temperature;
  bmp280.getTemperature(temperature);

  float pascal;
  bmp280.getPressure(pascal);

  static float meters, metersold;
  bmp280.getAltitude(meters);
  metersold = (metersold * 10 + meters)/11;

  bmp280.triggerMeasurement();
  Serial.println("***********************BMP 280**********************************");
  Serial.print(" HeightPT1: ");
  Serial.print(metersold);
  Serial.print(" m; Height: ");
  Serial.print(meters);
  Serial.print(" Pressure: ");
  Serial.print(pascal);
  Serial.print(" Pa; T: ");
  Serial.print(temperature);
  Serial.println(" C");

  Serial.println("***********************BMP 180**********************************");
  Serial.print("Temperature = ");
  Serial.print(bmp180.readTemperature());
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bmp180.readPressure());
  Serial.println(" Pa");
  
  // Calculate altitude assuming 'standard' barometric
  // pressure of 1013.25 millibar = 101325 Pascal
  Serial.print("Altitude = ");
  Serial.print(bmp180.readAltitude());
  Serial.println(" meters");

  Serial.print("Pressure at sealevel (calculated) = ");
  Serial.print(bmp180.readSealevelPressure());
  Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
  Serial.print("Real altitude = ");
  Serial.print(bmp180.readAltitude(101500));
  Serial.println(" meters"); 
  Serial.println("*********************************************************");*/
}

float readDS18B20(){
  byte i, type_s, data[12];
  
  ds.reset();
  ds.select(addr);
  ds.write(0x44);   
  // start conversion, use ds.write(0x44,1) with parasite power on at the end
  // maybe 750ms is enough, maybe not
  // we might do a ds.depower() here, but the reset will take care of it.
  delay(800);

  ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  for (i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  }

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  int16_t raw = (data[1] << 8) | data[0];
  
  if (type_s) {
    raw = raw << 3;             // 9 bit resolution default
    if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return raw;
}

static word homePage(byte humidity, unsigned short preasure, float tempOUT, float tempIN) {
  long t = millis() / 1000;
  word h = t / 3600;
  byte m = (t / 60) % 60;
  byte s = t % 60;
  bfill = ether.tcpOffset();
  bfill.emit_p(PSTR(
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: text/html\r\n"
    "Pragma: no-cache\r\n"
    "\r\n"
    "<!doctype html>\r\n"
    "<html><head><meta charset=\"UTF-8\">\r\n"
    "<meta http-equiv='refresh' content='3'>\r\n"
    "<title>RBBB server</title></head>\r\n"
    "<body><h1>Home Weather Station</h1><h2>Temperature - inside: $D$D</h2><h2>Humidity - inside: $D$D</h2><h2>Temperature - outside: $D$D</h2>"
    "<h2>Preasure: $D$D</h2><p>Last update: $D$D:$D$D:$D$D</body></html>"),
      tempIN, humidity, tempOUT, preasure,h/10, h%10, m/10, m%10, s/10, s%10);
  return bfill.position();
}