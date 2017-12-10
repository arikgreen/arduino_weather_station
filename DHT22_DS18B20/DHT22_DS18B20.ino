#include <DHT.h>          // DHT11/DHT22/AM2302/RHT03
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
//#include "i2c_BMP280.h"
#include <Adafruit_BMP085.h>

#define VERSION "0.6.0"
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

byte addr[8];

/*
 * backlight          - backlight time
 * readSensorPeriod   - the frequency of reading the sensors
 * h_prev             - previous humidity
 * tempDHT_prev       - previous temperature from DHT sensor
 * tempDS_prev        - previous temperature from DS18B20 sensor
 */
 
float tempDHT_prev, tempDS_prev;
int pascal_prev;
byte h_prev, stringLength, positionCounter=0;
short backlight=0, readSensorPeriod=0;
String content;

void setup() {
  Serial.begin(9600);
  pinMode(BTNPIN,INPUT);
  pinMode(PIRPIN,INPUT);
  
  lcd.begin(16,2);
  lcd.setBacklightPin(BACKLIGHTPIN,POSITIVE);
  lcd.setBacklight(1);
  lcd.backlight();
  
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

  readSensorPeriod = 10001; // read sensor in first loop

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
    lcd.print("Could not find a");
    lcd.setCursor(0,1);
    lcd.print("valid BMP180.");
    delay(4000);
  }
  
  lcd.clear();
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
  byte type_s;
  byte data[12];
  int pascal,metersold;
  float tempDHT, tempDS;

  pirPin = digitalRead(PIRPIN);
  btnPin = digitalRead(BTNPIN);
  
  
  if(pirPin == HIGH || btnPin == HIGH){
    lcd.setBacklight(1);
    backlight = 0;
  }

  if(readSensorPeriod > 2000 || btnPin == HIGH) {
    readSensorPeriod=0;
    ds.reset();
    ds.select(addr);
    ds.write(0x44); // start conversion, use ds.write(0x44,1) with parasite power on at the end
    delay(1000);    // maybe 750ms is enough, maybe not
                    // we might do a ds.depower() here, but the reset will take care of it.
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
    tempDS = (float)raw / 16.0;
    //fahrenheit = celsius * 1.8 + 32.0;
  
    tempDHT = dht.getTemperature();
    h = dht.getHumidity();
    pascal = bmp180.readPressure() / 100;
    metersold = bmp180.readAltitude();
    
    if(abs(tempDS-tempDS_prev)>=0.5  || abs(tempDHT-tempDHT_prev)>=0.5 || abs(h-h_prev)>=2 || abs(pascal-pascal_prev) >= 2) {
      lcd.setBacklight(1);
      backlight = 0;
    }

    tempDS_prev = tempDS;
    tempDHT_prev = tempDHT;
    h_prev = h;
    pascal_prev = pascal;

    content = "Humidity:" + String(h) + "% Pressure: " + String(pascal) + "hPa Altitude: " + String(metersold) + "m";
    stringLength = content.length();

    if(tempDS < 0)
      lcd.setCursor(0,0);
    else
      lcd.setCursor(1,0);
    lcd.print(tempDS);
    lcd.print((char)223);
    lcd.print(F("C"));
    if(tempDS >= 0)
      lcd.setCursor(10,0);
    else
      lcd.setCursor(9,0);
    lcd.print(tempDHT);
    lcd.print((char)223);
    lcd.print(F("C"));
  }
  lcd.setCursor(0,1);

// display the scrolling second line  
  if(readSensorPeriod%2 == 0) {
    if (positionCounter > stringLength)
      positionCounter = 0;
    if(positionCounter > stringLength-16){
      lcd.print(content.substring(positionCounter,stringLength));
      lcd.print(" ");
      lcd.print(content.substring(0,15-(stringLength-positionCounter)));
    } else {
      lcd.print(content.substring(positionCounter,16+positionCounter));
    }  
    positionCounter++;
  }
// end scrolling

  backlight++;
  readSensorPeriod++;

  delay(200);
  
  if(backlight > 100) lcd.setBacklight(0);

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

