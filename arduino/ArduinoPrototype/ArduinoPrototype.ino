/*
Multiplexer - HW-616, TCA9548A
https://randomnerdtutorials.com/tca9548a-i2c-multiplexer-esp32-esp8266-arduino/
LCD - 20x4
*/

#include "FS.h"
#include "SD.h"
#include <Wire.h>
#include <ClosedCube_Si7051.h>
#include <LiquidCrystal_PCF8574.h>

// Pin definitions
const int lcd_bus = 5;
const int temp_short_bus = 6;
const int temp_long_bus = 7;
const int buzzer_pin = 14;

// Filename for microSD
const String filename = "/temperature_readings.txt";

// Temperature sensors
ClosedCube_Si7051 si7051;
double sShort;
double sLong;

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
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(data)) {
    Serial.println("Data appended");
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
  lcd.print("Initializing LCD...");
  TCA9548A(lcd_bus);
  lcd.begin(lcdColumns, lcdRows);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();

  lcd.createChar(0, Celsius);
  lcd.createChar(1, hPa);

  // Initialize SD card
  lcd.print("Initializing SD card...");
  if (!SD.begin()) {
    Serial.println("SD card initialization failed!");
  }

  // Create a file if doesn't exist
  File file = SD.open(filename);
  if (!file) {
    Serial.println("File does not exist, creating a new file...");
    writeToFile(SD, filename, "Index, Temperature (short), Temperature (long), Temperature Difference (short and long)");
  } else {
    Serial.println("File exists");
  }
  file.close();

  // Initialize a buzzer pin as an output pin
  pinMode(buzzer_pin, OUTPUT);
}

void loop() {
  // Average temperatures for short and long termometers
  double avS = 0;
  char avS_bf[8];
  double avL = 0;
  char avL_bf[8];

  // Difference between short and long termometer
  double dLS = 0;
  char dLS_bf[8];

  // Temperature readings
  tempShort = 0;
  tempLong = 0;

  // Make 250 readings and calculate the average
  for (int i = 0; i < 250; i++) {
    TCA9548A(temp_short_bus);
    tempShort += si7051.readTemperature();
    TCA9548A(temp_long_bus);
    tempLong += si7051.readTemperature();
  }

  avS = tempShort / 250;
  avL = tempLong / 250;
  // avL = avL - 0.0105;
  dLS = avL - avS;

  sprintf(avS_bf, "%3.4f", avS);
  Serial.print("Termometer S = ");
  Serial.print(avS_bf);
  Serial.println(" C");

  sprintf(avL_bf, "%3.4f", avL);
  Serial.print("Termometer L = ");
  Serial.print(avL_bf);
  Serial.println(" C");

  Serial.println(dLS * 100);

  sprintf(dLS_bf, "%3.4f", dLS);

  // Switch to LCD bus
  TCA9548A(lcd_bus);

  // Print S (temperature on the short wire) on the LCD
  lcd.setCursor(0, 1);
  lcd.print("S ");
  lcd.setCursor(2, 1);
  lcd.printf(avS_bf, "%3.4f", avS);
  lcd.setCursor(9, 1);
  lcd.write(0);
  delay(10);

  // Print L (temperature on the long wire) on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("L ");
  lcd.setCursor(2, 0);
  lcd.printf(avL_bf, "%3.4f", avL);
  lcd.setCursor(9, 0);
  lcd.write(0);
  delay(10);

  // Print D (difference between L and S) on the LCD
  lcd.setCursor(0, 2);
  lcd.print("D ");
  lcd.setCursor(2, 2);
  lcd.printf(dLS_bf, "%3.4f", dLS);
  lcd.setCursor(9, 2);
  lcd.write(0);
  lcd.setCursor(10, 2);

  // Display on the LCD
  lcd.display();

  // Append to file on microSD
  int index = 0;
  String data = String(index++) + "," + String(avS) + "," + String(avL) + "," + String(dLS) + "\r\n";
  Serial.println(data);
  appendToFile(SD, filepath, data.c_str());

  // If average temperature of a short termometer is above 25.01, use a buzzer
  if (avS > 25.01) {
    digitalWrite(buzzer_pin, HIGH);  // turn the LED on (HIGH is the voltage level)
    delay(500);
    digitalWrite(buzzer_pin, LOW);  // turn the LED off by making the voltage LOW
    delay(1000);
  }
}