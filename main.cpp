#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>    
#include <SoftwareSerial.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

#include <Adafruit_Sensor.h>
#include <DHT.h>                // https://github.com/adafruit/DHT-sensor-library
//#include <MHZ19.h>            // https://github.com/WifWaf/MH-Z19
#include <PMS.h>                // //https://github.com/fu-hsi/pms
#include <MQUnifiedsensor.h>    // https://github.com/miguel5612/MQSensorsLib

#include <DS3231.h>             // https://github.com/NorthernWidget/DS3231

#define PIN_TS_IRQ  2
#define PIN_PMS_RX  3           // Rx pin which the PMS7003 Tx pin is attached to
#define PIN_PMS_TX  4           // Tx pin which the PMS7003 Rx pin is attached to (Use voltage divider to lower voltage up to 3V3)
//#define PIN_CO2_RX  5         // NOT used.
//#define PIN_CO2_TX  6         // NOT used.
#define PIN_CO2_IN 5
#define PIN_BZ 6
#define PIN_TFT_CS  7
#define PIN_TFT_RST 8 
#define PIN_TFT_DC  9
#define PIN_TS_CS  10
//#define PIN_MOSI 11
//#define PIN_MISO 12
//#define PIN_SCK 13

#define PIN_DHT A0
#define PIN_VOC_EN A1
#define PIN_O3_EN A2
#define PIN_O3_IN A3
//#define PIN_SDA A4
//#define PIN_SCL A5
#define PIN_BAT A6
#define PIN_VOC_IN A7

//#define DEBUG

#define BOARD "Arduino UNO"
#define ADC_BIT_RES 10
#define ADC_VOLTAGE_RES 5
#define VOC_SENSOR "MP-503"
#define O3_SENSOR "MQ-131"
#define R0RS_O3_RATIO_CLEANAIR 1
#define R0RS_VOC_RATIO_CLEANAIR 1
#define BAUDRATE 9600 

// See MH-Z19B datasheet
#define DHT_TYPE DHT22
#define CO2_PWM_CYCLE_MS 1004
#define CO2_PPM_MAX_RANGE 5000

Adafruit_ILI9341 Tft = Adafruit_ILI9341(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);
//XPT2046_Touchscreen Ts(PIN_TS_CS);
XPT2046_Touchscreen Ts(PIN_TS_CS, PIN_TS_IRQ);

// Temperature and humidity sensor
DHT Dht(PIN_DHT, DHT_TYPE);

SoftwareSerial SerialPMS(PIN_PMS_RX, PIN_PMS_TX);
PMS Pms(SerialPMS);
PMS::DATA PmsData;

// Get Co2 using Rx/Tx -> Not used. PPM used instead.
//SoftwareSerial SerialCO2(PIN_CO2_RX, PIN_CO2_TX);
//MHZ19 Co2;

MQUnifiedsensor mq131("Arduino UNO", ADC_VOLTAGE_RES, ADC_BIT_RES, PIN_O3_IN, O3_SENSOR);
MQUnifiedsensor mp503("Arduino UNO", ADC_VOLTAGE_RES, ADC_BIT_RES, PIN_VOC_IN, VOC_SENSOR);

const int addrO3 = 0;
const int addrVOC = 4;
float Mq131_R0, Mp503_R0;

DS3231 clock;
bool century = false;
bool h12Flag;
bool pmFlag;

bool bO3, bVOC, tO3, tVOC;



int countO3 = 0;
int countVOC = 0;

void initTFT(void);
void getTime(void);
void print2digit(char* to, byte val, char behind);

void getDHT22(void); 
void getCO2(void);
void getVOC(void);
void getO3(void);
void getPMS(void);
void getBat(void);

void senseO3(void);
void senseVOC(void);
void startCal(void);


void printDebug(void);
void printDHT22(void);
void printCO2(void);
void printVOC(void);
void printO3(void);
void printPMS(void);
void printBat(void);

float MP503Calibration(void);
float MQ131Calibration(void);
float getMP503EnvCorrectRatio(float h, float temp);
float getMQ131EnvCorrectRatio(float h, float temp);

String s;
const String ST = "C";
const String SH = "%";
const String WEEK[7] = {"  Lunes", " Martes", "Miercoles", " Jueves", " Viernes", " Sabado", " Domingo"};

int digitCursor = 0;
int co2, pms1, pms25, pms10;
float h, t, heat, voc, o3, bat;
bool resetScreen = false;

void setup() {

  voc = 0;
  o3 = 0;

  Serial.begin(BAUDRATE);
  SerialPMS.begin(BAUDRATE);

  pinMode(PIN_BAT, INPUT);
  
  // Disable SPI CS for avoid collision
  pinMode(PIN_TFT_CS, OUTPUT);
  pinMode(PIN_TS_CS, OUTPUT);
  pinMode(PIN_DHT, INPUT_PULLUP);
  digitalWrite(PIN_TFT_CS, HIGH);
  digitalWrite(PIN_TS_CS, HIGH);

  // Get Co2 using Rx/Tx -> Not used. PPM used instead.
  //SerialCO2.begin(BAUDRATE); 
  //Co2.begin(SerialCO2);
  //Co2.autoCalibration();
  pinMode(PIN_CO2_IN, INPUT);
  
  // PMS and Temp/Humidity sensor
  Pms.passiveMode();
  Dht.begin();

  // VOC Sensor (Smoke and Alcohol)
  mp503.init();
  mp503.setRegressionMethod(0); //ppm_log = (log10(ratio)-b)/a;
  EEPROM.get(addrVOC, Mp503_R0);
  mq131.setR0(Mp503_R0);
  pinMode(PIN_VOC_EN, OUTPUT);
  pinMode(PIN_VOC_IN, INPUT);
  bVOC = true;
  tVOC = false; 

  // Ozone Sensor    
  mq131.init();
  mq131.setRegressionMethod(1); //_PPM =  a*ratio^b
  mq131.setA(9.4783); mq131.setB(2.3348); // Configurate the ecuation values to get O3 concentration
  EEPROM.get(addrO3, Mq131_R0);
  mq131.setR0(Mq131_R0);
  pinMode(PIN_O3_EN, OUTPUT);
  pinMode(PIN_VOC_IN, INPUT);
  bO3 = true;
  tO3 = false; 

  Serial.print("VOC R0 = ");
  Serial.println(Mp503_R0);
  Serial.print("O3 R0 = ");
  Serial.println(Mq131_R0);


  // Init TFT Display
  Tft.begin();
  Tft.setRotation(3);
    
  initTFT();
      
  // Init Touch
  Ts.begin();
  Ts.setRotation(3);

  while (!Serial && (millis() <= 1000));
}

void loop() {

  if (Ts.tirqTouched()) {
    if (Ts.touched()) {
      TS_Point p = Ts.getPoint();

      
      if (p.x < 1000 && p.y > 3400) {
        // Start Calibration
        startCal();
      } else if (p.x > 2300 && p.y < 1200) {
        // CO2

      } else if (p.x > 1200 && p.y < 1200) {
        // VOC
        bVOC = true;
        tVOC = true;
        Tft.fillRect(185, 180, 5, 5, ILI9341_GREEN);

      } else if (p.y < 1200) {
        // O3
        bO3 = true;
        tO3 = true;
        Tft.fillRect(280, 180, 5, 5, ILI9341_GREEN);
      }



      
      Serial.print("Pressure = ");
      Serial.print(p.z);
      Serial.print(", x = ");
      Serial.print(p.x);
      Serial.print(", y = ");
      Serial.print(p.y);
      delay(30);
      Serial.println();
    }
  }
  
  
  // Each 15 min activate O3 & VOC sensors
  if (millis() % 900000 == 0) {
    bO3 = true;
    bVOC = true;
  } 
  

  if (millis() % 1500  == 0) {

    if (bO3) senseO3();
    if (bVOC) senseVOC();

    getTime();
    getBat();
    getDHT22();
    getCO2();
    getPMS();
    
  #ifdef DEBUG
  printDebug();
  #endif
    printDHT22();
    printCO2();
    printPMS();
    printBat();
  }

}

void senseO3() {
  if (countO3 == 0) digitalWrite(PIN_O3_EN, HIGH);
  countO3++;

  
  // 80 sec preheat
  if (countO3 > 80) {
    // Check if is one time. 
    if (tO3) {
      // Get time left
      int tLeft = 380 - countO3;
      Tft.setTextSize(2);
      Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      Tft.setCursor(295,175);
      if (tLeft > 240) Tft.print("4s");
      else if (tLeft > 180) Tft.print("3s");
      else if (tLeft > 120) Tft.print("2s");
      else if (tLeft > 60) Tft.print("1s");

    } else {
      digitalWrite(PIN_O3_EN, LOW);
      bO3 = false;
      countO3 = 0;
      Tft.fillRect(295, 175, 25, 20, ILI9341_BLACK);
    } 

    getO3();
    printO3();

    // 5 min + 80 sec preheat, stop sensor.
    if (countO3 > 380) {
      digitalWrite(PIN_O3_EN, LOW);
      bO3 = false;
      tO3 = false;
      countO3 = 0;
      Tft.fillRect(275, 175, 45, 20, ILI9341_BLACK);
    }
  } else {
    if (countO3 % 2 == 0) {
      Tft.fillRect(295, 175, 25, 20, ILI9341_BLACK);
    } else {
      Tft.setTextSize(2);
      Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      Tft.setCursor(295,175);
      Tft.print("H");
    }
  }
}

void senseVOC() {
  if (countVOC == 0) digitalWrite(PIN_VOC_EN, HIGH);
  countVOC++;

  // 80 sec preheat
  if (countVOC > 80) {

    // Check if is one time. 
    if (tVOC) {
      // Get time left
      int tLeft = 380 - countVOC;
      Tft.setTextSize(2);
      Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      Tft.setCursor(200,175);
      if (tLeft > 240) Tft.print("4s");
      else if (tLeft > 180) Tft.print("3s");
      else if (tLeft > 120) Tft.print("2s");
      else if (tLeft > 60) Tft.print("1s");
  
    } else {
      digitalWrite(PIN_VOC_EN, LOW);
      bVOC = false;
      countVOC = 0;
      Tft.fillRect(200, 175, 25, 20, ILI9341_BLACK);
    }

    getVOC();
    printVOC();

    // 5 min + 80 sec preheat, stop sensor.
    if (countVOC > 380) {
      digitalWrite(PIN_VOC_EN, LOW);
      bVOC = false;
      tVOC = false;
      countVOC = 0;
      Tft.fillRect(180, 175, 45, 20, ILI9341_BLACK);
    }
  } else {
    if (countVOC % 2 == 0) {
      Tft.fillRect(200, 175, 25, 20, ILI9341_BLACK);
    } else {
      Tft.setTextSize(2);
      Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
      Tft.setCursor(200,175);
      Tft.print("H");
    }
  }
}

void startCal() {
  Tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  Tft.setTextColor(ILI9341_LIGHTGREY);
  Tft.setTextSize(2);
  Tft.setCursor(10,6);
  Tft.print("Start Calibration in 3");
  delay(1000);

  Tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  Tft.setCursor(10,6);
  Tft.print("Start Calibration in 2");
  delay(1000);

  Tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  Tft.setCursor(10,6);
  Tft.print("Start Calibration in 1");
  delay(1000);

  Tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  Tft.setCursor(10,6);
  Tft.print("Calibration in process.");

  MP503Calibration();
  Tft.print(".");
  MQ131Calibration();
  Tft.print(".");

  initTFT();

}

void getTime() {
  static char time[10];
  print2digit(time, clock.getHour(h12Flag, pmFlag), ':');
  print2digit(time + 3, clock.getMinute(), ':');
  print2digit(time + 6, clock.getSecond(), 0);

  static char date[10];
  print2digit(date, clock.getDate(), '/');
  print2digit(date + 3, clock.getMonth(century), '/');
  print2digit(date + 6, clock.getYear(), 0);

  // Reset screen each day
  if (clock.getHour(h12Flag, pmFlag) == 0) {
    if (!resetScreen) initTFT();
    resetScreen = true;
  } else resetScreen = false;

  
  //String date = WEEK[clock.getDoW()-1] + clock.getDate() +SDATE+ clock.getMonth(century) +SDATE+ clock.getYear();

  Tft.fillRect(0, 0, 100, 30, ILI9341_NAVY);
  Tft.setTextColor(ILI9341_WHITE);
  Tft.setTextSize(1);
  Tft.setCursor(5,5);
  Tft.print(time);
  Tft.setCursor(5,20);
  Tft.print(date);

  Serial.print(clock.getYear());
  Serial.print("-");
  Serial.print(clock.getMonth(century));
  Serial.print("-");
  Serial.print(clock.getDate());
  Serial.print(" ");
  Serial.print(clock.getHour(h12Flag, pmFlag), DEC); //24-hr
  Serial.print(":");
  Serial.print(clock.getMinute(), DEC);
  Serial.print(":");
  Serial.println(clock.getSecond(), DEC);
}

void print2digit(char* to, byte val, char behind) {
  *to++ = '0' + val / 10;
  *to++ = '0' + val % 10;
  *to++ = behind;
}

void getDHT22() {
	h = Dht.readHumidity();
  t = Dht.readTemperature();
  heat = Dht.computeHeatIndex(t, h, false);
}

void getCO2() {
	//co2 = Co2.getCO2();
  unsigned long th, tl = 0;
  do {
      th = pulseIn(PIN_CO2_IN, HIGH, 10041000) / 1000;
      tl = CO2_PWM_CYCLE_MS - th;
      co2 = CO2_PPM_MAX_RANGE * (th - 2) / (th + tl - 4);
  } while (th == 0);
  // Compensate offset
  co2 += 22;
}

void getVOC() {
  mp503.setR0(Mp503_R0/getMP503EnvCorrectRatio(h, t));
  mp503.update(); // Update data, the arduino will be read the voltage on the analog pin

  // See https://jayconsystems.com/blog/understanding-a-gas-sensor
  mp503.setA(-0.6155);        // Configurate the ecuation values to get Smoke concentration
  // mp503.setA(-0.6797);  // Configurate the ecuation values to get Alcohol concentration
  mp503.setB(0);
  voc = mp503.readSensor();
}

void getO3() {
	mq131.setR0(Mq131_R0/getMQ131EnvCorrectRatio(h, t));
  mq131.update(); // Update data, the arduino will be read the voltage on the analog pin
  o3 = mq131.readSensor();
}

void getPMS() {
  Pms.requestRead();
  if (Pms.readUntil(PmsData)) {
      pms1 = PmsData.PM_AE_UG_1_0;
      pms25 = PmsData.PM_AE_UG_2_5;
      pms10 = PmsData.PM_AE_UG_10_0;
  }
}

void getBat() {
  int batVal = analogRead(PIN_BAT);
  Serial.println(batVal);
  // Scale analog reading to voltage value up to two decimals.
  batVal = map(batVal, 0, 1024, 0, 494);
  Serial.println(batVal);
  // Scale analog reading to voltage value up to two decimals.
  bat = float(batVal) / 100;
  Serial.println(bat);
}

void printDHT22() {
  // Temperature
  if (t > 35) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (t > 30) Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (t > 25) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK); 
  else if (t > 20) Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  else if (t > 10) Tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK); 
  else Tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK); 

  Tft.setTextSize(4);
  Tft.setCursor(32,60);
  Tft.print(t,1);

  // Humidity
  if (h > 60) Tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK);
  else if (h > 40) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (t > 20) Tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK); 
  else Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  
  Tft.setTextSize(3); 
  Tft.setCursor(15,120);
  s = int(h) + SH;
  Tft.print(s);

  // Heat
  if (heat > 35) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (heat > 30) Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (heat > 25) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK); 
  else if (heat > 20) Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  else if (heat > 10) Tft.setTextColor(ILI9341_CYAN, ILI9341_BLACK); 
  else Tft.setTextColor(ILI9341_BLUE, ILI9341_BLACK); 
  Tft.setCursor(90,120);
  s = int(heat) + ST;
  Tft.print(s);
}

void printCO2() {
  /* CO2 
    0-500 -> Good (GREEN)
    501-1000 -> Normal (YELLOW)
    1001-1500 -> Poor (ORANGE)
    1501-2000 -> Unhealthy (RED)
    > 2000 -> Very Unhealthy (PINK)
  */
  Tft.fillRect(10, 190, 85, 35, ILI9341_BLACK);
  Tft.setTextSize(3); 
  if (co2 > 2000) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (co2 > 1500) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (co2 > 1000) Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (co2 > 500) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK); 
  Tft.setCursor(10,200);
  Tft.print(co2);
}

void printVOC() {
  /* VOC (Volatile Organic Compounds)
    0-200 -> Good (GREEN)
    201-600 -> Normal (ORANGE)
    601-1000 -> Unhealthy (RED)
    > 1001 -> Very Unhealthy (PINK)
  */
  Tft.fillRect(145, 190, 75, 35, ILI9341_BLACK);
  Tft.setTextSize(3); 
  if (voc > 1000) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (voc > 600) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (voc > 200) Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  
  Tft.setCursor(145,200);
  if (voc >= 100) Tft.print(voc,0);
  else if (voc >= 10) Tft.print(voc,1);
  else Tft.print(voc);
}

void printO3() {
  /* Ozone
      0-50 -> Good (GREEN)
      51-100 -> Moderate (ORANGE)
      101-150 -> Unhealthy for Sensitive Groups (RED)
      > 151 -> Unhealthy (PINK)
  */
  Tft.fillRect(240, 190, 75, 35, ILI9341_BLACK);
  Tft.setTextSize(3); 
  if (o3 > 150) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (o3 >= 100) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if(o3 >= 50) Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);

  Tft.setCursor(240,200);
  if (o3 >= 100) {
    if (o3 < 1000) Tft.print(o3,0);
  }
  else if (o3 >= 10) Tft.print(o3,1);
  else Tft.print(o3,2);
}
    
void printPMS() {

  // Clean all PMS data area
  Tft.fillRect(245, 40, 300, 115, ILI9341_BLACK);
  Tft.setTextSize(3); 

  // Set PMS 1 um
  digitCursor = 280;
  if (pms1 >= 1000) digitCursor = 245;
  else if (pms1 >= 100) digitCursor = 260;
  else if (pms1 >= 10) digitCursor = 270;

  if (pms1 > 150) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (pms1 > 55) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (pms1 > 35)Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (pms1 > 12) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  
  Tft.setCursor(digitCursor,50);
  Tft.print(pms1);

  // Set PMS 2.5 um
  digitCursor = 280;
  if (pms25 >= 1000) digitCursor = 245;
  else if (pms25 >= 100) digitCursor = 260;
  else if (pms25 >= 10) digitCursor = 270;

  if (pms25 > 150) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (pms25 > 55) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (pms25 > 35)Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (pms25 > 12) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);

  Tft.setCursor(digitCursor,90);
  Tft.print(pms25);

  // Set PMS 10 um
  digitCursor = 280;
  if (pms10 >= 1000) digitCursor = 245;
  else if (pms10 >= 100) digitCursor = 260;
  else if (pms10 >= 10) digitCursor = 270;

  if (pms10 > 350) Tft.setTextColor(ILI9341_PINK, ILI9341_BLACK);
  else if (pms10 > 250) Tft.setTextColor(ILI9341_RED, ILI9341_BLACK);
  else if (pms10 > 100)Tft.setTextColor(ILI9341_ORANGE, ILI9341_BLACK);
  else if (pms10 > 50) Tft.setTextColor(ILI9341_YELLOW, ILI9341_BLACK);
  else Tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  Tft.setCursor(digitCursor,130);
  Tft.print(pms10);
}

void printBat() {
  // Clean battery area
  Tft.fillRect(260, 2, 60, 12, ILI9341_NAVY);

  Tft.setTextColor(ILI9341_WHITE);
  Tft.setTextSize(1);

  Tft.setCursor(265,5);

  if (bat > 4.15) Tft.print("100%");
  else if (bat > 4.1) Tft.print("95%");
  else if (bat > 4.05) Tft.print("90%");
  else if (bat > 4) Tft.print("85%");
  else if (bat > 3.9) Tft.print("80%");
  else if (bat > 3.85) Tft.print("70%");
  else if (bat > 3.8) Tft.print("60%");
  else if (bat > 3.75) Tft.print("50%");
  else if (bat > 3.7) Tft.print("40%");
  else if (bat > 3.65) Tft.print("30%");
  else if (bat > 3.6) Tft.print("20%");
  else if (bat > 3.55) Tft.print("10%");
  else if (bat > 3.5) Tft.print("5%");
  else  Tft.print("5%");

  Tft.fillRect(297, 7, 3, 3, ILI9341_GREEN);
  Tft.fillRect(300, 5, 15, 7, ILI9341_GREEN);
  if (bat < 3.8) {
    if (bat < 3.6) {
      Tft.fillRect(297, 7, 3, 3, ILI9341_ORANGE);
      Tft.fillRect(300, 5, 15, 7, ILI9341_ORANGE);
    } else {
      Tft.fillRect(297, 7, 3, 3, ILI9341_RED);
      Tft.fillRect(300, 5, 15, 7, ILI9341_RED);
    }
  } 

}

void printDebug() {
  Serial.println("*************************************");
  Serial.print("Temp: ");
  Serial.print(t);
  Serial.println(" ºC");
  Serial.print("Humidity: ");
  Serial.print(h);
  Serial.println(" %");
  Serial.print("Heat: ");
  Serial.print(heat);
  Serial.println(" ºC");
  Serial.print("CO2: ");
  Serial.print(co2);
  Serial.println("ppm");
  Serial.print("PMS 1.0: ");
  Serial.print(pms1);
  Serial.println("ppm");
  Serial.print("PMS 2.5: ");
  Serial.print(pms25);
  Serial.println("ppm");
  Serial.print("PMS 10: ");
  Serial.print(pms10);
  Serial.println("ppm");
  Serial.print("VOC: ");
  Serial.print(voc);
  Serial.println("ppm");
  Serial.print("O3: ");
  Serial.print(o3);
  Serial.println("ppm");
}

float MP503Calibration() {
  /*****************************  MQ CAlibration ********************************************/
  // Explanation:
  // In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
  // and now is on clean air (Calibration conditions), and it will setup R0 value.
  // We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
  // This routine not need to execute to every restart, you can load your R0 if you know the value
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor

  
  //If the RL value is different from 10K please assign your RL value with the following method:
  mp503.setRL(100);


  Serial.print("Calibrating VOC please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
      mp503.update(); // Update data, the arduino will be read the voltage on the analog pin
      calcR0 += mp503.calibrate(R0RS_VOC_RATIO_CLEANAIR);
      Serial.print(".");
  }
  Mp503_R0 = calcR0/10;
  EEPROM.put(addrVOC, Mp503_R0);
  EEPROM.get(addrVOC, Mp503_R0);
  Serial.println(Mp503_R0);

  mp503.setR0(Mp503_R0);
  Serial.println("  done!.");
}

float MQ131Calibration() {
  /*****************************  MQ CAlibration ********************************************/
  // Explanation:
  // In this routine the sensor will measure the resistance of the sensor supposing before was pre-heated
  // and now is on clean air (Calibration conditions), and it will setup R0 value.
  // We recomend execute this routine only on setup or on the laboratory and save on the eeprom of your arduino
  // This routine not need to execute to every restart, you can load your R0 if you know the value
  // Acknowledgements: https://jayconsystems.com/blog/understanding-a-gas-sensor

  
  //If the RL value is different from 10K please assign your RL value with the following method:
  mq131.setRL(1000);


  Serial.print("Calibrating O3 please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
      mq131.update(); // Update data, the arduino will be read the voltage on the analog pin
      calcR0 += mq131.calibrate(R0RS_O3_RATIO_CLEANAIR);
      Serial.print(".");
  }
  Mq131_R0 = calcR0/10;
  EEPROM.put(addrO3, Mq131_R0);
  EEPROM.get(addrO3, Mq131_R0);
  Serial.println(Mq131_R0);

  mq131.setR0(Mq131_R0);
  Serial.println("  done!.");
}

float getMP503EnvCorrectRatio(float h, float temp) {
   // Select the right equation based on humidity
  // If default value, ignore correction ratio
  if(h == 60 && temp == 25) {
    return 1.0;
  }
  // For humidity > 75%, use the 85% curve
  if(h > 75) {
    // R^2 = 0.9897
    return -0.0104 * temp + 1.1493;
  }
  // For humidity > 50%, use the 60% curve
  if(h > 50) {
    // R^2 = 0.9916
    return -0.0116 * temp + 1.3201;
  }

  // Humidity < 50%, use the 30% curve
  // R^2 = 0.9909
  return -0.0141 * temp + 1.5549;
}

float getMQ131EnvCorrectRatio(float h, float temp) {
   // Select the right equation based on humidity
  // If default value, ignore correction ratio
  if(h == 60 && temp == 25) {
    return 1.0;
  }
  // For humidity > 75%, use the 85% curve
  if(h > 75) {
    // R^2 = 0.9986
    return -0.0141 * temp + 1.5623;
  }
  // For humidity > 50%, use the 60% curve
  if(h > 50) {
    // R^2 = 0.9976
    return -0.0119 * temp + 1.3261;
  }

  // Humidity < 50%, use the 30% curve
  // R^2 = 0.996
  return -0.0103 * temp + 1.1507;
}

void initTFT() {

  Tft.fillScreen(ILI9341_BLACK);

  // Title
  Tft.fillRect(0, 0, 320, 30, ILI9341_NAVY);
  Tft.setTextColor(ILI9341_LIGHTGREY);
  Tft.setTextSize(2);
  Tft.setCursor(110,6);
  Tft.print(WEEK[clock.getDoW()-1]);
 
  Tft.setTextColor(ILI9341_WHITE);
  Tft.setTextSize(1);
  Tft.setCursor(5,5);
  Tft.print("Hour");
  Tft.setCursor(5,20);
  Tft.print("Date");

  Tft.setCursor(260,5);
  Tft.print("Battery");
  Tft.setCursor(260,20);
  Tft.print("Calibrate");

  // lines
  Tft.drawLine(160, 30, 160, 170, ILI9341_RED);   // Top vertical line 
  Tft.drawLine(0, 170, 320, 170, ILI9341_RED);    // Horizontal Line
  Tft.drawLine(130, 170, 130, 240, ILI9341_RED);  // Bottom vertical line 1
  Tft.drawLine(225, 170, 225, 240, ILI9341_RED);  // Bottom vertical line 2

  // Text Sensors
  Tft.setTextSize(2);
  Tft.setCursor(175,53);
  Tft.print("pms1");
  Tft.setCursor(175,93);
  Tft.print("pms25");
  Tft.setCursor(175,133);
  Tft.print("pms10");

  Tft.setCursor(5,175);
  Tft.print("CO2:");
  Tft.setCursor(135,175);
  Tft.print("VOC:");
  Tft.setCursor(230,175);
  Tft.print("O3:");

  Tft.setTextSize(1);
  Tft.setCursor(50,45);
  Tft.print("Temperature");
  Tft.setCursor(15,150);
  Tft.print("Humidity");
  Tft.setCursor(100,150);
  Tft.print("Heat");

  // Units
  Tft.setCursor(275,160);
  Tft.print("[ug/m3]");
  Tft.setCursor(95,230);
  Tft.print("[ppm]");
  Tft.setCursor(190,230);
  Tft.print("[ppb]");
  Tft.setCursor(285,230);
  Tft.print("[ppb]");

}