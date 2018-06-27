// Arduino Libraries Used
#include <Adafruit_Si7021.h>    // SI7021 Temp/Humid I2C Library
#include <HCSR04.h>             // Ultrasonic Sensor Library - On Mega Echo 26, Trig 28
#include <Adafruit_BMP085.h>    // BMP180 Temp/Pressure I2C Library
#include <Wire.h>               // Standard Arduino Library for RTC Allows I2C Connectivity   
#include <RTClib.h>             // Real Time Clock I2C Library
#include <SPI.h>                // Standard Arduino Serial Peripheral Interface Library
#include <SD.h>                 // Standard Arduino SD Card Reader I2C Library
#include <Adafruit_MPL115A2.h>  //This is the replacement for BMP180

const int chipSelect = 4;

// Pins for HCSR04 - ultrasonic sensor. 
const int TRIG_PIN = 5;
const int ECHO_PIN = 6;
HCSR04 hc(TRIG_PIN, ECHO_PIN);        // HCSR04 object - ultrasonic sensor. 
const unsigned int MAX_DIST = 23200;  //Anything over 400 cm (23200 us pulse) is "out of range" for our sensor. 
float originalDistance; 
float currentDistance;      

Adafruit_Si7021 temperatureSensor = Adafruit_Si7021();  //Si7021 object - temp/humidity sensor. 
float tempCelsius;                    
float tempFarenheit;                  
float humidity;

Adafruit_BMP085 pressureSensor;       // BMP-180 object - pressure sensor.
float pressure;                       
float pressureInches; 

RTC_DS3231 rtc;                       // RTC object - real-time clock.

int peopleCounter;
int currentState = 0;
int previousState = 0;

File dataFile; 

void setup() {

  // Open serial comm & wait for port to open. 
  Serial.begin(9600);

  while (!Serial);          // Wait for serial port to connect. 


  // RTC setup:
  if(!rtc.begin()){
    Serial.println("Could not find RTC");
    while(1);
  }
  if(rtc.lostPower()){
    Serial.println("RTC lost power, let's set the time.");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));           //Sets RTC to date & time when sketch was compiled. 
  }
  Serial.println("RTC............OK!");


  // BMP-180 setup:
  if(!pressureSensor.begin()){
    Serial.println("Could not find a valid BMP-180 sensor, check wiring.");
    while(1);
  }
  Serial.println("Adafruit BMP-180............OK!");


  // Si7021 setup:
  if(!temperatureSensor.begin()){
    Serial.println("Did not find Si7021 sensor.");
    while(1);
  }
  Serial.println("Adafruit Si7021..............OK!");


//  // Get original distance from device & wall to compare to. 
  Serial.print("The original distance between the wall & the device is ");
  originalDistance = (int) hc.dist();
  Serial.print(originalDistance);
  Serial.println();

  


}

void loop() {

  DateTime now = rtc.now();

  pressure = pressureSensor.readPressure();         // Read barometric pressure from BMP-180.
  pressureInches = pressure * 0.00029530;           // Conversion pascals to inches.

  tempCelsius = temperatureSensor.readTemperature(), 2;    // Read temperature from Si7021.
  tempFarenheit = tempCelsius * 1.8 + 32;           // Conversion C to F. 

  humidity = temperatureSensor.readHumidity(), 2;          // Read humidity from Si7021.
  
  Serial.println();
  Serial.println("================================================================================================================");

  formatDateToSerial(now);
  formatTimeToSerial(now);

  Serial.print("Temperature (F): ");
  Serial.println(tempFarenheit);

  Serial.print("Humidity (%): ");
  Serial.println(humidity);

  Serial.print("Pressure (in): ");
  Serial.println(pressureInches);

  Serial.print("Distance station - wall (cm): ");
  Serial.println(originalDistance);

  Serial.print("People: ");
  Serial.println(peopleCounter);

  Serial.print("Distance (cm): ");
  currentDistance = calculateDistance(20);
  Serial.println(currentDistance);

  if (currentDistance + 3 < originalDistance){
 currentState = 1;
 }
 else {
 currentState = 0;
 }
 delay(5);
 if(currentState != previousState){
if(currentState == 1){
peopleCounter += 1;
}
 }



  delay(1);

  

}


int calculateDistance(int nbrOfReadings){

   unsigned long t1;
   unsigned long t2;
   unsigned long pulse_width;
   float cm;
   float inches;
   float totalCm;
   float totalInches;
   float distanceCm;
   float distanceInches;

   for (int i=0; i<nbrOfReadings; i++){

      // Hold the trigger pin high for at least 10 us.
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);

      // Wait for pulse on echo pin.
      while (digitalRead(ECHO_PIN) == 0);

      // Measure how long the echo pin was held high (pulse width).
      t1 = micros();
      while(digitalRead(ECHO_PIN) == 1);
      t2 = micros();
      pulse_width = t2 - t1;

      // Calculate distance in cm & in. Constants are from datasheet for sensor. 
      cm = pulse_width / 58.0;
      inches = pulse_width / 148.0;

      // Add to variable to later calculate average for more precise value. 
      totalCm += cm;
      totalInches += inches;
    
   }

   distanceCm = totalCm / nbrOfReadings;
   distanceInches = totalInches / nbrOfReadings;

   return distanceCm;
    
}



void formatDateToSerial(DateTime currentTime) {

  Serial.println();         //For formatting sakes on the serial monitor.
  if (currentTime.month() < 10)
    Serial.print ("0");   // Adds a 0 to the hour if it is less than 10

  Serial.print(currentTime.month(), DEC);
  Serial.print('/');

  if (currentTime.day() < 10)
    Serial.print ("0");   // Adds a 0 to the hour if it is less than 10

  Serial.print(currentTime.day(), DEC);
  Serial.print('/');

  Serial.print(currentTime.year(), DEC);
  Serial.print(" ");
}



void formatTimeToSerial(DateTime currentTime) {

  if (currentTime.hour() < 10)
    Serial.print ("0");   // Adds a 0 to the hour if it is less than 10

  Serial.print(currentTime.hour(), DEC);
  Serial.print(':');

  if (currentTime.minute() < 10)
    Serial.print ("0");   // Adds a 0 to the hour if it is less than 10

  Serial.print(currentTime.minute(), DEC);
  Serial.print(':');

  if (currentTime.second() < 10)
    Serial.print ("0");   // Adds a 0 to the hour if it is less than 10

  Serial.print(currentTime.second(), DEC);
  Serial.println(" ");

}






















/*
   S18 Final Project
   COMP407-001 Embedded Systems
   Professor M. Black Rm. 294
   by:  Mark Pasquantonio, Cassis LaFleur, and Mike Misite
   Picked up as Undergraduate Research by: Amine Toualbi
  //BUG avrdude: stk500v2_ReceiveMessage(): timeoutxx`x`
   Weather Station Research Project for Prof. Hellstrom

   The project was to use a series of sensors and an Arduino Mega to collect weather related data, like
   the temperature, humidity, and barometric pressure for a micro climate (area under pedestrian
   train bridge).  We also used an ultrasonic sensor which we configured to count people for a given
   tie that passed through the tunnel.  Finally, we connected an sd card reader to save the data so it
   can be used to migrate into a Microsoft Excel spreadsheet.

   Equipment used:

   An Arduino Mega
   A Sunfounder SD Card Reader
   An Xbee Shield with an Xbee S1 transmitter/receiver
   A portable powerpack with 2 - 3.7v 4200mAh Li-Ion batteries
   An HC-SR04 Ultrasonic Sensor
   An Adafruit BMP-180 Barometer/Humidity Sensor with I2C
   An Adafruit Si7021 Temperature/Humidity Sensor with I2C
   A DS3231 Real Time Clock Module with I2C
   Assorted Wires and Connectors
   Sealable Plastic Container to protect from the elements

   //FIND LIBRARY FOR MPL11582 INSTEAD OF BMP-180

*/





/*
  The circuit:
  SD card attached to SPI bus as follows:
  For UNO:  MOSI - pin 11, MISO - pin 12, CLK - pin 13, CS - pin 4 (CS pin can be changed)
  and pin #10 (SS) must be an output
  For Mega:  MOSI - pin 51, MISO - pin 50, CLK - pin 52, CS - pin 4 (CS pin can be changed)
  and pin #52 (SS) must be an output
  On the Ethernet Shield, CS is pin 4. Note that even if it's not
  used as the CS pin, the hardware CS pin (10 on most Arduino boards,
  53 on the Mega) must be left as an output or the SD library
  functions will not work.
*/

/*
    CLK = pin 21.
    DATA = pin 20.
    GND =
*/
