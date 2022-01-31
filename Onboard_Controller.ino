/*
MIT License 

Copyright (c) 2021 J. Ashby, F. Le Bars, J. Lim

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Description: 
Onboard controller code for AENG20003 AVDASI2 Design, Build, and Test group project at University of Bristol.

Hardware: 
Teensy 4.1 (with micro SD)
Adafruit RFM69HCW Packet Radio Breakout
Adafruit MPU-6050 6-DoF Accel and Gyro Sensor
Parallax standard servo
Toggle switch
6V DC Power Pack

Connections:
  Servo & Teensy + to 6V (via toggle)
  Servo & Teensy - to GND
  Servo comms to pin 6
  RF VIN pin to 3.3V
  RF GND pin to GND
  RF EN pin to 8
  RF G0 pin to 2
  RF SCK pin to 13
  RF MISO pin to 12
  RF MOSI pin to 11
  RF CS pin to 10
  RF RSI pin to 3
  IMU VCC to 3.3V
  IMU GND to GND
  IMU SCL to 19
  IMU SDA to 18
  IMU INT to 9
*/

#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Servo.h>
#include <stdio.h>
#include <stdlib.h>
#include <RFM69.h>         // get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     // get it here: https://www.github.com/lowpowerlab/rfm69
// If using teensy, RFM69.cpp needs to be modified:
// Ctrl + F to find "setDataMode"
// Comment out the line: _spi->setDataMode(SPI_MODE0);
// Add under that: pinMode( _interruptPin, INPUT );

// Radio config
#define NODEID        1    // should always be 1 for a Gateway
#define NETWORKID     100  // the same on all nodes that talk to each other
#define FREQUENCY     433
#define FREQUENCY_EXACT 433000000 // you may define an exact frequency/channel in Hz
#define IS_RFM69HW_HCW  // uncomment only for RFM69HW/HCW; Leave out if you have RFM69W/CW
#define ENABLE_ATC    // comment out this line to disable AUTO TRANSMISSION CONTROL

#define SERIAL_BAUD   115200

// Essential radio pins
#define RST_PIN 3
#define CS_PIN  10
#define G0_PIN  2   // interrupt pin 

// If else for initialising ATC and radio
#ifdef ENABLE_ATC
RFM69_ATC radio(CS_PIN, G0_PIN);
#else
RFM69 radio(CS_PIN, G0_PIN);
#endif

// IMU variables
const int MPU_addr = 0x68;
const int chipSelect = BUILTIN_SDCARD;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
unsigned long runTime;

// PID variables
int setAngle;
double Pitch, Elevator, DesiredPitch;
double Kp = 1, Ki = 0, Kd = 0, angle; // tuning parameters
//Kp - how aggressively PID reacts to current amount of error
//Ki - how aggressively PID reacts to error over time
//Kd - how aggressively PID reacts to change in error
PID elevatorPID(&Pitch, &Elevator, &DesiredPitch, Kp, Ki, Kd, REVERSE); // establish PID settings

char recvdata[19]; // array for receiving RF data - change to length of your data string
float valarray[3]; // array for storing received char to floats - change to # of values you want to store

Servo outservo; // define servo

// Servo variables
int minVal = 265;
int maxVal = 402;
double x, y, z;

//=======================================================================================
//===================================== Main Setup ======================================
//=======================================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);

  //========================================================================================
  //===================================== Radio Setup ======================================
  //========================================================================================

  // Reset radio to clear any previous code
  pinMode(RST_PIN, OUTPUT);
  delay(10);
  digitalWrite(RST_PIN, LOW);
  delay(10);
  // Initialise radio
  radio.initialize(FREQUENCY, NODEID, NETWORKID);

#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); // must include this only for RFM69HW/HCW
#endif

#ifdef FREQUENCY_EXACT
  radio.setFrequency(FREQUENCY_EXACT); // set frequency to some custom frequency
#endif

#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif

  char buff[128];
  sprintf(buff, "Listening at %d MHz...", FREQUENCY);
  Serial.println(buff);

  //==================================================================================================
  //===================================== Servo & SD Card Setup ======================================
  //==================================================================================================

  outservo.attach(6);
  elevatorPID.SetMode(AUTOMATIC);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(9600);
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

}

// Set counters for radio to 0
byte ackCount = 0;
uint32_t packetCount = 0;

//======================================================================================
//===================================== Main Loop ======================================
//======================================================================================

void loop() {

  // Process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();

    // Dump register values for debugging
    if (input == 'r')
      radio.readAllRegs();
    // Read radio temperature
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
    }
  }

  // Start receiving data
  if (radio.receiveDone())
  {
    // Serial stuff to check radio is working
    Serial.print("\n#[");
    Serial.print(++packetCount); // print packet count
    Serial.print(']');
    Serial.print('[');
    Serial.print(radio.SENDERID, DEC);
    Serial.print("] ");

    // Store each received char into the array for char
    for (byte i = 0; i < radio.DATALEN; i++) {
      recvdata[i] = (char)radio.DATA[i];

      Serial.print((char)radio.DATA[i]); // print the data to check what radio is receiving

      if (radio.DATA[i] ==  59 ) { // 59 is the ASCII value of ";" which is the end of the message
        // Separate the sent data
        chartofloat(recvdata, valarray); // convert the stored char to floats
        for (byte j = 0; j <= 3; j++) {

          // Serial stuff to check values have been stored correctly
          // Serial.println();
          // Serial.print(valarray[j]);

          // Assign the stored values into our variables
          DesiredPitch = valarray[0];
          Kp = (double)valarray[1];
          Ki = (double)valarray[2];
          Kd = (double)valarray[3];

          elevatorPID.Compute(); // PID calculation function
          Serial.println(Elevator); // Serial stuff to check the raw command value for the elevator

          // For fixed elevator settings
          if (Kp == 10) // Use arbituary Kp to indicate we want to fix the elevator - can use any other variable as long as it's sent in the RF data
            Elevator = DesiredPitch;
        }
      }
    }
    // Serial stuff to check radio RSSI
    // Serial.print("   [RX_RSSI:");
    // Serial.print(radio.RSSI);
    // Serial.print("]");

    // Acknowledgement code for controller to check if onboard has received the data
    // When the controller requests an ACK, respond to the ACK
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.print(" - ACK sent \n");

      if (ackCount++ % 5 == 0) // Send a packet requesting an ACK (every 5th one only) - increase if radio transmits too slow
      {
        // Serial stuff to check onboard has received the ACK request
        Serial.print(" - Pinging ground");
        // Serial.print(theNodeID);
        Serial.print(" - ACK ");
        delay(3);
        if (radio.sendWithRetry(theNodeID, "ACK TEST", 8, 0))  // 0 = only 1 attempt, no retries
          Serial.print("received");
        else Serial.print("not recevied");
      }
    }
  }

  //===========================================================================================
  //===================================== PID & IMU code ======================================
  //===========================================================================================

  elevatorPID.SetOutputLimits(-25, 25);//min is CW/left; max is AC // previously unsuccessful values: 10,48 and 0,50
  runTime = millis();  // prints time since program started
  File dataFile = SD.open("ElevDat.txt", FILE_WRITE);
  // Begin reading IMU data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);

  // Pitch correction (from testing of own system - will need to be adjusted)
  if (y >= 90) {
    y = y - 360;
  }
  y = -y + 8;
  y = y * 0.833;
  Pitch = y;

  // Read in the desired gains to the PID settings
  elevatorPID.SetTunings(Kp, Ki, Kd);

  // Serial stuff to check that values are correct
  Serial.print("Pitch angle = ");
  Serial.println(Pitch);
  Serial.print("Desired pitch = ");
  Serial.println(DesiredPitch);

  // Call PID calculations
  elevatorPID.Compute();

  // For fixed elevator settings
  if (Kp == 10)
    Elevator = DesiredPitch;

  // Serial stuff to check values are correct
  Serial.print("Raw elevator output = ");
  Serial.println(Elevator);
  
  double ServoOutput = (Elevator * 0.76) + 29; // adjust elevator angle to the servo's range of values

  // Serial stuff to check servo value is correct
  Serial.print("Adjusted elevator output = ");
  Serial.println(ServoOutput);
  
  outservo.write(ServoOutput); // output the value to servo

  // SD card logging
  dataFile.print("True Pitch Angle (deg):\t");
  dataFile.print(Pitch); dataFile.print(",\t");
  dataFile.print("Desired Pitch Angle (deg):\t");
  dataFile.print(DesiredPitch); dataFile.print(",\t");
  dataFile.print("Kp :\t");
  dataFile.print(Kp); dataFile.print(",\t");
  dataFile.print("Ki :\t");
  dataFile.print(Ki); dataFile.print(",\t");
  dataFile.print("Kd :\t");
  dataFile.print(Kd); dataFile.print(",\t");
  dataFile.print("Elevator angle :\t");//PID Output
  dataFile.print(Elevator); dataFile.print(",\t");
  dataFile.print("Raw servo output:\t");
  dataFile.print(ServoOutput); dataFile.print(",\t");
  dataFile.print("Timestamp (ms):\t");
  dataFile.print(runTime); dataFile.println(",\t");
  delay(10);
  // Data can easily be processed in software, for example MATLAB
}

// Function to convert char to float
byte chartofloat(char str[], float arr[]) {
  byte data_count = 0; //number of data
  char * item = strtok(str, " :"); //getting first value (uses "space" & ":" as delimeter)

  while (item) {
    if (data_count < sizeof(str))
      arr[data_count] = atof(item);
    item = strtok(NULL, " :"); // getting subsequence value
    data_count++;
  }
  //  return  arr;
}
