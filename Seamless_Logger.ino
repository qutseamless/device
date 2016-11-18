

/******************************************************************************
  SEAMLESS CAPSTONE - LOGGER PROGRAM

  Resources:
  TinyGPS++ Library  - https://github.com/mikalhart/TinyGPSPlus/releases
  SD Library (Built-in)
  SoftwareSerial Library (Built-in)

  Development/hardware environment specifics:
  Arduino IDE 1.6.7
  GPS Logger Shield v2.0 - Make sure the UART switch is set to SW-UART
  Arduino Uno, RedBoard, Pro, Mega, etc.
******************************************************************************/

//#include <SPI.h>
//#include <SD.h>
#include <TinyGPS++.h>
#include "DHT.h"
#include <AltSoftSerial.h>
#include <SFE_MG2639_CellShield.h>

#define ARDUINO_USD_CS 10 // uSD card CS pin (pin 10 on SparkFun GPS Logger Shield)

////////////////////////
// Device Definitions //
////////////////////////
#define DEVICE_ID 1
#define POWER_LED 4
#define STATUS_LED 5

/////////////////////////
// Log File Defintions //
/////////////////////////
// Keep in mind, the SD library has max file name lengths of 8.3 - 8 char prefix,
// and a 3 char suffix.
// Our log files are called "gpslogXX.csv, so "gpslog99.csv" is our max file.
//#define LOG_FILE_PREFIX "gpslog" // Name of the log file.
//#define MAX_LOG_FILES 100 // Number of log files that can be made
//#define LOG_FILE_SUFFIX "csv" // Suffix of the log file
//char logFileName[13]; // Char string to store the log file name
//// Data to be logged:
//#define LOG_COLUMN_COUNT 10
//char * log_col_names[LOG_COLUMN_COUNT] = {
//  "id", "longitude", "latitude", "altitude", "speed", "course", "date", "time", "humidity", "temperature"
//}; // log_col_names is printed at the top of the file.

////////////////////////
// Server Definitions //
////////////////////////
char serverPhone[] = "61476856557";

//////////////////////
// Log Rate Control //
//////////////////////
#define LOG_RATE 5000 // Log every 5 seconds
unsigned long lastLog = 0; // Global var to keep of last time we logged

/////////////////////////
// TinyGPS Definitions //
/////////////////////////
TinyGPSPlus tinyGPS; // tinyGPSPlus object to be used throughout
#define GPS_BAUD 9600 // GPS module's default baud rate

/////////////////////////////////
// GPS Serial Port Definitions //
/////////////////////////////////
// If you're using an Arduino Uno, Mega, RedBoard, or any board that uses the
// 0/1 UART for programming/Serial monitor-ing, use SoftwareSerial:
#include <SoftwareSerial.h>
#define ARDUINO_GPS_RX 9 // GPS TX, Arduino RX pin
#define ARDUINO_GPS_TX 8 // GPS RX, Arduino TX pin
SoftwareSerial ssGPS(ARDUINO_GPS_TX, ARDUINO_GPS_RX); // Create a SoftwareSerial

// Set gpsPort to either ssGPS if using SoftwareSerial or Serial1 if using an
// Arduino with a dedicated hardware serial port
#define gpsPort ssGPS  // Alternatively, use Serial1 on the Leonardo

// Define the serial monitor port. On the Uno, Mega, and Leonardo this is 'Serial'
//  on other boards this may be 'SerialUSB'
#define SerialMonitor Serial

////////////////////////
// Sensor Definitions //
////////////////////////
#define DHTPIN 6
DHT dht(DHTPIN, DHT11);

void setup()
{
  pinMode(POWER_LED, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(POWER_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  
  SerialMonitor.begin(9600);
  dht.begin();
}

void loop()
{
  gpsPort.begin(GPS_BAUD);
  Serial.println("Delaying...");

  if ((lastLog + LOG_RATE) <= millis())
  { // If it's been LOG_RATE milliseconds since the last log:
    if (tinyGPS.satellites.value() > 0) // If the GPS data is vaild
    {
      if (writeSMS()) // Log the GPS data
      {
        SerialMonitor.println("GPS logged."); // Print a debug message
        lastLog = millis(); // Update the lastLog variable
      }
      else // If we failed to log GPS
      { // Print an error, don't update lastLog
        SerialMonitor.println("Failed to log new GPS data. Check your SIM & Ant?");
      }
    }
    else // If GPS data isn't valid
    {
      // Print a debug message. Maybe we don't have enough satellites yet.
      SerialMonitor.print("No GPS data. Sats: ");
      SerialMonitor.println(tinyGPS.satellites.value());
    }
  }

  // If we're not logging, continue to "feed" the tinyGPS object:
  while (gpsPort.available())
    tinyGPS.encode(gpsPort.read());
}

byte writeSMS()
{
  Serial.print(F("Sending a message to "));
  Serial.println(serverPhone);

  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float slat = tinyGPS.location.lat();
  float slng = tinyGPS.location.lng();
  float salt = tinyGPS.altitude.meters();
  float sspeed = tinyGPS.speed.kmph();
  float shead = tinyGPS.course.deg();
  int sdate = tinyGPS.date.value();
  int stime = tinyGPS.time.value();

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return 0;
  }
  
  gpsPort.flush();
  gpsPort.end();
  
  // Call cell.begin() to turn the module on and verify
  // communication.
  int beginStatus = cell.begin();
  if (beginStatus <= 0)
  {
    Serial.println(F("Unable to communicate with shield. Looping"));
    while (1)
      ;
  }
  // Delay a bit. If phone was off, it takes a couple seconds
  // to set up SIM.
  delay(2000);

  // Set SMS mode to text mode. This is a more commonly used,
  // ASCII-based mode. The other possible mode is
  // SMS_PDU_MODE - protocol data unit mode - an uglier,
  // HEX-valued data mode that may include address info or
  // other user data.
  sms.setMode(SMS_TEXT_MODE);

  // To begin sending an SMS, use sms.start(phoneNumber):
  sms.start(serverPhone);
    
  sms.print(DEVICE_ID);
  sms.print(',');
  sms.print(slat, 6);
  sms.print(',');
  sms.print(slng, 6);
  sms.print(',');
  sms.print(salt, 1);
  sms.print(',');
  sms.print(sspeed, 1);
  sms.print(',');
  sms.print(shead, 1);
  sms.print(',');
  sms.print(h, 1);
  sms.print(',');
  sms.print(t, 1);

  Serial.println();
  Serial.println("Sending the message.");
  Serial.println();

  // Send an sms message by calling sms.send()
  if (sms.send() < 0) {
    return 0;
  }

  gpsPort.begin(GPS_BAUD);
  digitalWrite(STATUS_LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(30000);
  Serial.println("Satellites:");
  Serial.print(tinyGPS.satellites.value());
  Serial.println();
  digitalWrite(STATUS_LED, LOW);   // turn the LED on (HIGH is the voltage level)

  return 1;
}

