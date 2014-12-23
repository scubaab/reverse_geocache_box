/* **********************************************************************************************************************************
 Project: Reverenced geocache box
 Made by: Ab Kurk
 Date:16/12/2014
 Version:2.0
 Description:
 This project allows you to give somebody  a gift or send them on a quest at a specific set of coordinates. The project is box with a locking mechanism that will open on a specific location. Through a USB cable and a serial terminal you can communicate with the Arduino board in the box and enter latitude and longitude. Lock the box and unlock the box.
 Components:
 Push Button
 GPS Module http://www.adafruit.com/product/746
 LCD Screen http://www.sainsmart.com/arduino/arduino-shields/lcd-shields/new-sainsmart-iic-i2c-twi-1602-serial-lcd-module-display-for-arduino-uno-mega-r3.html
 Lipo Booster/Charger https://www.sparkfun.com/products/11231
 Lipo Battery https://www.sparkfun.com/products/341
 Latching Relay http://www.digikey.ca/product-detail/en/D3063/PB1114-ND/1634000
 Arduino Pro Mini 5v https://www.sparkfun.com/products/11113
 FTDI https://www.sparkfun.com/products/9716
 
 
 */
#include <math.h>
//Required Libraries for the Adafruit GPS Breakout
#include <Adafruit_GPS.h> // connected to pin 2 and 3 digital
#include <SoftwareSerial.h>
// If using software serial, keep this line enabled
// (you can change the pin numbers to match your wiring):
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);



#include <PWMServo.h> //on pin 9. This library is needed as the devault library conflicts with the Softwareserial library. Your servo will twitch if you don't use this library
int servo_pin=9;
PWMServo servo1;  // create servo object to control a servo
//int pos = 90;    // variable to store the servo position 

#include <EEPROMex.h>//Added library to make it easier to write datatypes to memory 

////Libraries required for lcd display
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h> //scl pin a5 sda pin a4

#define I2C_ADDR    0x3F  // Define I2C Address where the PCF8574A is
#define BACKLIGHT_PIN     3
#define En_pin  2
#define Rw_pin  1
#define Rs_pin  0
#define D4_pin  4
#define D5_pin  5
#define D6_pin  6
#define D7_pin  7



LiquidCrystal_I2C	lcd(I2C_ADDR,En_pin,Rw_pin,Rs_pin,D4_pin,D5_pin,D6_pin,D7_pin);






int coordordistance=0; // a switch to either show your current coordinates or the distance to target


// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false

// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
float range = 0;   // Distance to target. In the program this is used to display distance to target.

//************* user changable variables*****************************************************
float howclose=12; // To set how close you need to be to the target in meters for the box to open. On a good day the accuracy of a GPS is 3m radius, on a bad day it can increase to 10 meters. Keeping this number too small could make you stand in the middle of traffic to unlock your box.
int servo_open=90;//Depending the direction you install your servo as a locking mechanism this variable allows you to set the open position
int servo_locked=1;//Depending the direction you install your servo as a locking mechanism this variable allows you to set the locked position
long howlong=120000; //In millisecond the time interval for the box to turn its self-off to preserver battery. 1 second is a 1000 milliseconds 
//**************end **************************************************

int youhere=0; //used to let the device know that it has arrived to target open the box and shut the device down
float latthere;//destination latidute. It gets read out of arduino EEPROM memory
float lonthere;//destination longitude. It get read out of arduino EEPROM memory
int latinput=0;//toggle for input of latidute. I use this to let hte system know that the next serial input is going to be latitude address
int loninput=0;//toggle for input of longtitude. I use this to let hte system know that the next serial input is going to be latitude address
int latchpin=8;//set the pin that will latch the relay to turn power off on the project. For this project I chose digital 8
unsigned long t2=millis();// timer init for turning power off on hte system



void setup()  
{

  Serial.begin(115200);
  //Config the servo
  // attaches the servo on pin 9 to the servo object
  servo1.attach(servo_pin);  
  /*Checks if box should be locked or open. It reads it out of the arduino EEPROM memory stored at address 0. If the vavlue is 0 the box is unlocked. If the value is 1 the box is locked. 
   Depending the orientation of your servo you need to edit the values below. The value is somewhere between 0 and 180
   */
  if(EEPROM.read(0)==0){
    servo1.write(servo_open);
    servo1.write(servo_open);
  }
  if(EEPROM.read(0)==1){
    servo1.write(servo_locked);
    servo1.write(servo_locked);
  }

  //read coordinates from memmory
  latthere=EEPROM.readFloat(100);
  //Serial.println(latthere,5);
  lonthere=EEPROM.readFloat(200);
  //Serial.println(lonthere,5);
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Switch on the backlight
  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home ();                   // go home position to receive data

  // Set the GPS update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate


  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);

  //Set the pin for tripping the relay to switch off the arduino board after 2 minutes
  pinMode(latchpin, OUTPUT);
  digitalWrite(latchpin, LOW);
  t2=millis();

}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  //#ifdef UDR0
  //  if (GPSECHO)
  // if (c) UDR0 = c;  
  // writing direct to UDR0 is much much faster than Serial.print 
  // but only one character can be written at a time. 
  //#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } 
  else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();


//Reads serial input from computer console
int readline(int readch, char *buffer, int len)
{
  static int pos = 0;
  int rpos;

  if (readch > 0) {
    switch (readch) {
    case '\n': // Ignore new-lines
      break;
    case '\r': // Return on CR
      rpos = pos;
      pos = 0;  // Reset position index ready for next time
      return rpos;
      break;
    default:
      if (pos < len-1) {
        buffer[pos++] = readch;
        buffer[pos] = 0;
      }
    }
  }
  // No end of line has been found, so return -1.
  return -1;
}

void loop()                     // run over and over again
{
  //Section to read serial input from computer to configure box 
  static char buffer[80];

  if (readline(Serial.read(), buffer, 80) > 0) {
    Serial.print("You entered: >");
    Serial.print(buffer);

    Serial.println("<");
    //You cann unlock the box from serial monitor by typing UNLOCK
    if(strcmp(buffer,"UNLOCK")==0){
      servo1.write(servo_open);
      EEPROM.write(0,0);//writes to EEPROM memory 
    }
    //You cann lock the box from serial monitor by typing LOCK
    if(strcmp(buffer,"LOCK")==0){
      servo1.write(servo_locked);
      EEPROM.write(0,1);
    }
    //Writes the latitude coordinate into memory
    if(latinput==1){
      String b=buffer;
      latthere= atof(b.c_str());
      EEPROM.writeFloat(100,latthere);
      latinput=0;
      Serial.print(latthere,6);
      Serial.println(" Latitude Stored in memmory");
    }
    //You can enter the Latitude coordinate by entering LAT in the serial monitor
    if(strcmp(buffer,"LAT")==0){
      Serial.println("Please enter the Latitude now e.g. 49.262951");
      latinput=1;
    }
    //Writes the longitude coordinate into memory
    if(loninput==1){
      String b=buffer;
      lonthere= atof(b.c_str());
      EEPROM.writeFloat(200,lonthere);
      loninput=0;
      Serial.print(lonthere,6);
      Serial.println(" Longitude Stored in memmory");
    }
    //You can enter the Longitude coordinate by entering LAT in the serial monitor
    if(strcmp(buffer,"LON")==0){
      Serial.println("Please enter the Longitude now e.g. -123.1234567");
      loninput=1;
    }
    //Get info on the state of your box
    if(strcmp(buffer,"INFO")==0){
      Serial.print("Servo status. 0 is unlocked 1 is locked: ");
      Serial.println(EEPROM.read(0));
      Serial.print("Current target Latidude: ");
      Serial.println(latthere,6);
      Serial.print("Current target Longitude: ");
      Serial.println(lonthere,6);
    }

  }



  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    
  }

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 1 seconds or so, print out the current stats
  if (millis() - timer > 1000) { 
    timer = millis(); // reset the timer

    if(!GPS.fix)  t2 = millis();//resets the countdown timer for shutting down the system aslong as the box does not have a satellite fix
    if (GPS.fix) { 
      //Power down system in 2 minutes
     
      if(millis()- t2>howlong){
        
        digitalWrite(8, HIGH);
        delay(1000);
        digitalWrite(8, LOW);
      }
     //Gets the distance to target
      range = haversine(GPS.latitudeDegrees, GPS.longitudeDegrees, latthere, lonthere);
      if(coordordistance==0){
        lcd.clear();
        lcd.setCursor(0,0);

        lcd.print("Lat:");
        lcd.print(GPS.latitudeDegrees, 4); 
        lcd.print(GPS.lat);

        lcd.setCursor(0,1);
        lcd.print("Lon:");
        lcd.print(GPS.longitudeDegrees, 4); 
        lcd.print(GPS.lon); 

        coordordistance=1;
      }
      else if(coordordistance==1){
        coordordistance=0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Distance To");
        lcd.setCursor(0,1);
        lcd.print("Target:");
        if(range>1000){//displays km
          lcd.print(range/1000,3 );
          lcd.print(" km");

        }
        else if(range<howclose){

          servo1.write(90);
          servo1.write(90);
          if(youhere==0)EEPROM.write(0,0);
          youhere++;
          if(youhere>10)digitalWrite(8, HIGH);//turns off the box in 10 seconds
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Congratulations");
          lcd.setCursor(0,1);
          lcd.print("YOU ARE HERE");
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("OPEN THE BOX");
          lcd.setCursor(0,1);
          lcd.print("CLAIM YOUR PRIZE");

          servo1.write(servo_open);
          servo1.write(servo_open);
         // EEPROM.write(0,0);
          coordordistance=1;
        }
        else{
          lcd.print(range,0);
          lcd.print(" m");
          if(range<50)coordordistance=1;
        }

      }
    }
    else {//displays message it is searching for a connection to satelite
      lcd.setCursor(0,0);
      lcd.print("Searching for");
      lcd.setCursor(0,1);
      lcd.print("Satellites........");

    }
  }

  //delay(1000);
}

float haversine (float lat1, float lon1, float lat2, float lon2) {
  // returns the great-circle distance between two points (radians) on a sphere. Did not make this formula got it of the internet and modified it. Just google havesine
  float ToRad = PI / 180.0; //needed to convert degrees in to rad
  float R = 6371000.0;    // radius earth in m

  float dLat = (lat2-lat1) * ToRad;
  float dLon = (lon2-lon1) * ToRad; 

  float a = sin(dLat/2) * sin(dLat/2) +
    cos(lat1 * ToRad) * cos(lat2 * ToRad) * 
    sin(dLon/2) * sin(dLon/2); 

  float c = 2 * atan2(sqrt(a), sqrt(1-a)); 

  float d = R * c;


  return d;

};

