#include <DHT.h>          // DHT11/DHT22/AM2302/RHT03
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>

#define VERSION "0.4.9"
#define DHTPIN 2          // DHT data pin
#define DSPIN 5           // DS18B20 data pin
#define BACKLIGHTPIN 3    // LCD backlight
#define BTNPIN 6          // button
#define PIRPIN 8          // PIR

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);
 
DHT dht;
OneWire ds(DSPIN);

byte addr[8];

/*
 * backlight          - backlight time
 * readSensorPeriod   - the frequency of reading the sensors
 * h_prev             - previous humidity
 * tempDHT_prev       - previous temperature from DHT sensor
 * tempDS_prev        - previous temperature from DS18B20 sensor
 */
 
float tempDHT_prev, tempDS_prev;
byte h_prev;
short backlight=0, readSensorPeriod=0;

void setup() {
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
    }
    delay(250);
  }
  lcd.print(F("Weather Station"));
  lcd.setCursor(0,1);
  lcd.print(VERSION);
  delay(2000);
  delay(dht.getMinimumSamplingPeriod());

  readSensorPeriod = 10001; // read sensor in first loop
  
  lcd.clear();
  lcd.print(F("DHT: "));
  lcd.setCursor(0,1);
  lcd.print(F("DS: "));
  lcd.setCursor(10,0);
  lcd.print((char)223);
  lcd.print(F("C"));
  lcd.setCursor(15,0);
  lcd.print(F("%"));
  lcd.setCursor(10,1);
  lcd.print((char)223);
  lcd.print(F("C"));
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
  float tempDHT, tempDS;

  pirPin = digitalRead(PIRPIN);
  btnPin = digitalRead(BTNPIN);
  
  if(pirPin == HIGH || btnPin == HIGH){
    lcd.setBacklight(1);
    backlight = 0;
  }

  if(readSensorPeriod > 10000 || btnPin == HIGH) {
    lcd.setCursor(15,1);
    lcd.print(F("R"));
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

    if(abs(tempDS-tempDS_prev)>=0.5  || abs(tempDHT-tempDHT_prev)>=0.5 || abs(h-h_prev)>=2) {
      lcd.setBacklight(1);
      backlight = 0;
    }

    tempDS_prev = tempDS;
    tempDHT_prev = tempDHT;
    h_prev = h;
    
    lcd.setCursor(5,0);
    if(tempDHT < 10) lcd.print(F(" "));
    lcd.print(tempDHT);
    lcd.setCursor(13,0);
    lcd.print(h);
    lcd.setCursor(5,1);
    if(tempDS < 10 && tempDS >= 0) lcd.print(F(" "));
    lcd.print(tempDS);
  }
  
  if(readSensorPeriod%20==0){
    lcd.setCursor(15,1);
    lcd.print(F("*"));
  }
  if(readSensorPeriod%40==0){
    lcd.setCursor(15,1);
    lcd.print(F(" "));
  }
  backlight++;
  readSensorPeriod++;

  delay(50);
  
  if(backlight > 400) lcd.setBacklight(0);
}
