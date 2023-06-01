/*
Multiplexer - HW-616, TCA9548A
https://randomnerdtutorials.com/tca9548a-i2c-multiplexer-esp32-esp8266-arduino/
LCD - 20x4
MicroSD - https://randomnerdtutorials.com/esp32-microsd-card-arduino/
custom pins - https://randomnerdtutorials.com/esp32-microsd-card-arduino/#sdcardcustompins
OneWire, DS18B20 - https://randomnerdtutorials.com/esp32-multiple-ds18b20-temperature-sensors/
LED&KEY - https://docs.turais.de/docs/displays/7segment/led-and-key-module-tm1638/
*/

#include "FS.h"
#include "SD.h"
#include <OneWire.h>
#include <Wire.h>
#include <ClosedCube_Si7051.h>
#include <LiquidCrystal_PCF8574.h>
#include <DallasTemperature.h>
#include <TM1638.h>

// Pin definitions through multiplexer
const int lcd_bus = 5;
const int temp_short_bus = 6;
const int temp_long_bus = 7;
const int buzzer_pin = 14;

// Pin definitions for LED&KEY
const int DIO = 33;
const int CLK = 25;
const int STB = 4;

// LED&KEY
TM1638 ledKeyModule(DIO, CLK, STB);
typedef enum {
  BTN1 = 1,
  BTN2 = 2u,
  BTN3 = 4u,
  BTN4 = 8u,
  BTN5 = 16u,
  BTN6 = 32u,
  BTN7 = 64u,
  BTN8 = 128u
} BTN;
byte currentButtonState;

// Data wire connected to GPIO 27
const int one_wire_bus = 27;
// Setup a oneWire instance to communicate with a OneWire device
OneWire oneWire(one_wire_bus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// Dallas temperature sensor addresses
DeviceAddress sensor1 = { 0x28, 0xFF, 0x47, 0x27, 0x33, 0x17, 0x3, 0x5C };
DeviceAddress sensor2 = { 0x28, 0xFF, 0x31, 0x51, 0x40, 0x17, 0x5, 0x84 };

// Filename for microSD
const char *filename = "/temperature_readings.txt";
int rowIndex = 0;

// Temperature sensors
ClosedCube_Si7051 si7051;

// LCD
int lcdColumns = 20;
int lcdRows = 4;
LiquidCrystal_PCF8574 lcd(0x27);

// Celsius degree sign
byte Celsius[8] = {
  B11000,
  B11000,
  B00110,
  B01001,
  B01000,
  B01000,
  B01001,
  B00110
};

// hPa sign
byte hPa[8] = {
  B10000,
  B11100,
  B10100,
  B00000,
  B11100,
  B10100,
  B11111,
  B10011
};

// Get LED&KEY button state
bool isButtonPressed(const byte &state, const BTN check) {
  byte btn_state = (state & check);
  return btn_state / check;
}

// Select I2C BUS
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// Create and write to a file
void writeToFile(fs::FS &fs, const char *path, const char *data) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(data)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to a file
void appendToFile(fs::FS &fs, const char *path, const char *data) {
  // Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(data)) {
    // Serial.println("Data appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void setup() {
  // Initialize serial
  Serial.begin(230400);
  delay(800);
  Serial.println("ClosedCube_Si7051, LiquidCrystal_PCF8574 on TCA9548A");

  // Start I2C communication with the Multiplexer
  Wire.begin();

  // Temperature sensor on a short wire
  TCA9548A(temp_short_bus);
  si7051.begin(0x40);

  // Temperature sensor on a long wire
  TCA9548A(temp_long_bus);
  si7051.begin(0x40);

  // Initialize LCD
  Serial.println("Initializing LCD...");
  TCA9548A(lcd_bus);
  lcd.begin(lcdColumns, lcdRows);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();

  lcd.createChar(0, Celsius);
  lcd.createChar(1, hPa);

  // Initialize SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
  }

  // Create a file if doesn't exist
  File file = SD.open(filename);
  if (!file) {
    file.close();
    Serial.println("File does not exist, creating a new file...");
    String columns = "Index, Temperature (short), Temperature (long), Difference (short and long), Temperature (dallas1), Temperature (dallas2), Difference (dallas1 and dallas2) \r\n";
    writeToFile(SD, filename, columns.c_str());
  } else {
    file.close();
    Serial.println("File exists");
  }

  // Initialize a buzzer pin as an output pin
  pinMode(buzzer_pin, OUTPUT);
}

void loop() {
  // Temperature readings
  double tempShort = 0;
  double tempLong = 0;

  // Average temperatures for short and long thermometers
  double avS = 0;
  char avS_bf[8];
  double avL = 0;
  char avL_bf[8];

  // Difference between short and long thermometer
  double dLS = 0;
  char dLS_bf[8];

  // Dallas temperature readings
  double dallas1 = 0;
  double dallas2 = 0;
  char dallas1_bf[8];
  char dallas2_bf[8];

  // Difference between Dallas thermometers
  double diffDallas = 0;
  char diffDallas_bf[8];

  // Make 250 readings and calculate the average
  int readings = 250;
  for (int i = 0; i < readings; i++) {
    TCA9548A(temp_short_bus);
    tempShort += si7051.readTemperature();
    TCA9548A(temp_long_bus);
    tempLong += si7051.readTemperature();
  }

  avS = tempShort / readings;
  avL = tempLong / readings;
  dLS = avL - avS;

  // Dallas thermometers readings (doesn't work in the loop to calculate the average)
  sensors.requestTemperatures();
  dallas1 += sensors.getTempC(sensor1);
  dallas2 += sensors.getTempC(sensor2);
  diffDallas = dallas2 - dallas1;

  sprintf(avS_bf, "%3.4f", avS);
  Serial.printf("Termometer S = %3.4f C\n", avS_bf);

  sprintf(avL_bf, "%3.4f", avL);
  Serial.printf("Termometer L = %3.4f C\n", avL_bf);

  sprintf(dLS_bf, "%3.4f", dLS);
  Serial.printf("Diff S-L = %3.4f C\n", dLS * 100);

  sprintf(dallas1_bf, "%3.4f", dallas1);
  Serial.printf("Termometer Dallas1 = %3.4f C\n", dallas1_bf);

  sprintf(dallas2_bf, "%3.4f", dallas2);
  Serial.printf("Termometer Dallas2 = %3.4f C\n", dallas2_bf);

  sprintf(diffDallas_bf, "%3.4f", diffDallas);
  Serial.printf("Diff Dallas = %3.4f C\n", diffDallas * 100);

  byte buttonsState = ledKeyModule.getButtons();
  Serial.println(buttonsState, HEX);
  if (buttonsState != 0) {
    currentButtonState = buttonsState;
  }
  Serial.println(currentButtonState);
  if (isButtonPressed(currentButtonState, BTN1)) {
    // Switch to LCD bus
    TCA9548A(lcd_bus);

    // Print S (temperature on the short wire) on the LCD
    lcd.setCursor(0, 0);
    lcd.print("S ");
    lcd.setCursor(2, 0);
    lcd.printf(avS_bf, "%3.4f", avS);
    lcd.setCursor(9, 0);
    lcd.write(0);
    delay(10);

    // Print L (temperature on the long wire) on the LCD
    lcd.setCursor(0, 1);
    lcd.print("L ");
    lcd.setCursor(2, 1);
    lcd.printf(avL_bf, "%3.4f", avL);
    lcd.setCursor(9, 1);
    lcd.write(0);
    delay(10);

    // Print D (difference between L and S) on the LCD
    lcd.setCursor(0, 2);
    lcd.print("D ");
    lcd.setCursor(2, 2);
    lcd.printf(dLS_bf, "%3.4f", dLS);
    lcd.setCursor(9, 2);
    lcd.write(0);
  } else if (isButtonPressed(currentButtonState, BTN2)) {
    // Switch to LCD bus
    TCA9548A(lcd_bus);

    // Print dallas1 on the LCD
    lcd.setCursor(0, 0);
    lcd.print("1 ");
    lcd.setCursor(2, 0);
    lcd.printf(dallas1_bf, "%3.4f", dallas1);
    lcd.setCursor(9, 0);
    lcd.write(0);
    delay(10);

    // Print dallas2 on the LCD
    lcd.setCursor(0, 1);
    lcd.print("2 ");
    lcd.setCursor(2, 1);
    lcd.printf(dallas2_bf, "%3.4f", dallas2);
    lcd.setCursor(9, 1);
    lcd.write(0);
    delay(10);

    // Print D (difference between dallas) on the LCD
    lcd.setCursor(0, 2);
    lcd.print("D ");
    lcd.setCursor(2, 2);
    lcd.printf(diffDallas_bf, "%3.4f", diffDallas);
    lcd.setCursor(9, 2);
    lcd.write(0);
  }

  // Display on the LCD
  lcd.display();

  // Append to file on microSD
  String data = String(rowIndex++) + "," + String(avS) + "," + String(avL) + "," + String(dLS) + "," + String(dallas1) + "," + String(dallas2) + "," + String(diffDallas) + "\r\n";
  Serial.println(data);
  appendToFile(SD, filename, data.c_str());

  // If average temperature of a short termometer is above 25.01, use a buzzer
  if (avS > 25.01) {
    digitalWrite(buzzer_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);
    digitalWrite(buzzer_pin, LOW);  // turn the LED off by making the voltage LOW
    delay(1000);
  }
}