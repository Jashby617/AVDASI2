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
Ground station code for AENG20003 AVDASI2 Design, Build, and Test group project at University of Bristol.

Hardware: 
Teensy 4.0
Adafruit RFM69HCW Packet Radio Breakout
LCD panel (compatable with the Hitachi HD44780 driver) - Standard 16 pin
10K Resistor
4x 1K Potentiometers
Red and Green LED
6V DC Battery Pack

Connections:
   LCD RS pin to digital pin 18
   LCD Enable pin to digital pin 19
   LCD D4 pin to digital pin 20
   LCD D5 pin to digital pin 21
   LCD D6 pin to digital pin 22
   LCD D7 pin to digital pin 23
   LCD R/W pin to ground
   LCD VSS pin to ground
   LCD VCC pin to 5V
   10K resistor:
   ends to +5V and ground
   LCD V0 pin to 5V
   POT 1 central pin to A0 (14)
   POT 1 + to 3.3V
   POT 1 - to GND
   POT 2 central pin to A1 (15)
   POT 2 + to 3.3V
   POT 2 - to GND
   POT 3 central pin to A2 (16)
   POT 3 + to 3.3V
   POT 3 - to GND
   POT 4 central pin to A3 (17)
   POT 4 + to 3.3V
   POT 4 - to GND
   Power LED + pin to digital pin 4
   Power LED - pin to GND
   Status LED + pin to digital pin 5
   Status LED - pin to GND
   RF VIN pin to 3.3V
   RF GND pin to GND
   RF EN pin to 8
   RF G0 pin to 2
   RF SCK pin to 13
   RF MISO pin to 12
   RF MOSI pin to 11
   RF CS pin to 10
   RF RSI pin to 3
*/


// Include the libraries
#include <LiquidCrystal.h>
#include <SPI.h>
#include <RFM69.h>         //get it here: https://www.github.com/lowpowerlab/rfm69
#include <RFM69_ATC.h>     //get it here: https://www.github.com/lowpowerlab/rfm69
// If using teensy, RFM69.cpp needs to be modified:
// Ctrl + F to find "setDataMode"
// Comment out the line: _spi->setDataMode(SPI_MODE0);
// Add under that: pinMode( _interruptPin, INPUT );

// Radio config
#define NODEID        2    // keep UNIQUE for each node on same network
#define NETWORKID     100  // keep IDENTICAL on all nodes that talk to each other
#define GATEWAYID     1    // "central" node - don't change
#define FREQUENCY   433
#define FREQUENCY_EXACT 433000000 // you may define an exact frequency/channel in Hz
#define IS_RFM69HW_HCW  //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
#define ATC_RSSI      -70

#define SERIAL_BAUD   115200

// Essential radio pins
#define RST_PIN 3
#define CS_PIN  10
#define G0_PIN  2   // interrupt pin 
// Potentiometer pins
#define Ang_PIN 14
#define Kp_PIN 15
#define Ki_PIN 16
#define Kd_PIN 17

#define FIXED_PIN 6 // fixed elevator control switch
#define TX_LED 4 // transmission LED indicator pin
#define POWER_PIN 5 // power LED indicator pin

// Size of RF data to send
#define DATA_SIZE 21 // length of data + 1

// Initialise radio variables
int TRANSMITPERIOD = 20; //transmit a packet to gateway so often (in ms)
char msg[DATA_SIZE];
char buff[DATA_SIZE];
char msglen[50];
byte sendSize = 0;
boolean requestACK = true;

// If else for initialising ATC and radio
#ifdef ENABLE_ATC
RFM69_ATC radio(CS_PIN, G0_PIN);
#else
RFM69 radio(CS_PIN, G0_PIN);
#endif

// Initialise LCD Pins
const int rs = 18, en = 19, d4 = 20, d5 = 21, d6 = 22, d7 = 23;

// Initialise PID variables
int setAngle;
float Kp, Ki, Kd, angle, reading, angleRange = 50; // read in input from potentiometer (1-1023)
long KpRound, KiRound, KdRound;

// Initialise potentiometer variables
float read_Kp, read_Ki, read_Kd;
int potpin = 14, Kp_pin = 15, Ki_pin = 16, Kd_pin = 17;

float KpRange = 3, KiRange = 0.6; , KdRange = 0.6;
int SetAngle, SetAngle2 = 5;

// Setup LCD
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

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
  radio.enableAutoPower(ATC_RSSI);
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)\n");
#endif

  char buff[128];
  sprintf(buff, "Transmitting at %d MHz...", FREQUENCY);
  Serial.println(buff);

  // Set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Turn on power status led
  pinMode(FIXED_PIN, INPUT);
  pinMode(POWER_PIN, OUTPUT);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(POWER_PIN, HIGH);
  // Print loading screen
  lcd.setCursor(0, 0);
  lcd.print("   SILVERFISH    ");
  lcd.setCursor(0, 1);
  lcd.print(" CONTROLLER V15    ");
  delay(5000);
}

long lastPeriod = 0;

//======================================================================================
//===================================== Main Loop ======================================
//======================================================================================

void loop() {

  //Kp = -1.1;
  //Ki = -2.1;
  //Kd = -0.2;
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input >= 48 && input <= 57) //[0,9]
    {
      TRANSMITPERIOD = 100 * (input - 48);
      if (TRANSMITPERIOD == 0) TRANSMITPERIOD = 1000;
      char txtime[20];
      sprintf(txtime, "\nChanging delay to %d ms\n", TRANSMITPERIOD);
      Serial.println(txtime);
    }

    if (input == 'r') // dump register values
      radio.readAllRegs();
  }

  //Read in input from potentiometers (0-1023)
  reading = analogRead(potpin); 
  read_Kp = analogRead(Kp_pin);
  read_Ki = analogRead(Ki_pin);
  read_Kd = analogRead(Kd_pin);



  lcd.setCursor(0, 1); // set the cursor to column 0, line 1
  angle = ((reading / 1023) * angleRange) - (angleRange / 2); // calculate input angle
  Kp = ((read_Kp / 1023) * KpRange); // calculate inputted Kp angle (only +)
  Ki = ((read_Ki / 1023) * KiRange) - (KiRange / 2); // calculate inputted Ki angle (+ and -)
  Kd = ((read_Kd / 1023) * KdRange) - (KdRange / 2); // calculate inputted Kd angle (+ and -)

  // Round Kp to nearest 0.1 and Ki/Kd to nearest 0.01
  KpRound = (long)(Kp * 10);
  Kp = (double)KpRound / 10;
  KiRound = (long)(Ki * 100);
  Ki = (double)KiRound / 100;
  KdRound = (long)(Kd * 100);
  Kd = (double)KdRound / 100;

  // If fixed elevator switch is closed
  if (digitalRead(FIXED_PIN) == HIGH)
  {
    // Use Kp value of 10 (outside input range) to communicate to onboard controller that the fixed elevator mode is selected
    Kp = 10;
    /*lcd.setCursor(0, 0);
      lcd.print("    ELEVATOR      ");
      lcd.setCursor(0, 1);
      lcd.print("     LOCKED      ");*/
    // Send current demanded angle as the fixed angle and print info to screen
    SetAngle = -round(angle);
    lcd.setCursor(0, 0);
    lcd.print("ELEVATOR FIXED       ");
    lcd.setCursor(0, 1);
    lcd.print("Elev angle = ");
    lcd.print(SetAngle);
    lcd.print("          ");
  }

  // In normal PID operation
  else {
    SetAngle = round(angle); // round input angle to nearest int
    lcd.setCursor(0, 0);
    lcd.print("Pitch:");
    lcd.print(SetAngle); // print the angle on screen

    // Print PID values to screen (location will depend on size of screen)
    lcd.print(" Kp:");
    lcd.print(Kp);
    lcd.setCursor(0, 1);
    //lcd.print(Kp);
    lcd.print("Ki:");
    lcd.print(Ki);
    lcd.print(" Kd:");
    lcd.print(Kd);
    //lcd.setCursor(4, 1);
    //lcd.print(" deg.");
  }

  //check for any received packets
  if (radio.receiveDone())
  {
    Serial.print('[');
    Serial.print(radio.SENDERID, DEC);
    Serial.print("] ");
    for (byte i = 0; i < radio.DATALEN; i++)
      Serial.print((char)radio.DATA[i]);
    //      Serial.print("   [RX_RSSI:");
    //      Serial.print(radio.RSSI);
    //      Serial.print("]");

    if (radio.ACKRequested())
    {
      radio.sendACK();
      Serial.print(" - ACK sent");
    }
  }

  int currPeriod = millis() / TRANSMITPERIOD;
  if (currPeriod != lastPeriod)
  {
    lastPeriod = currPeriod;

    if (sendSize == 0)
    {
      byte buffLen = strlen(buff);
      if (radio.sendWithRetry(GATEWAYID, buff, buffLen)) {
        Serial.println("\n--- Start of transmission ---");
      }
      else {
        Serial.println("\n--- Transmission not received ---");
      }
      // Blink the transmission LED to show that the full packet has been sent
      digitalWrite(TX_LED, HIGH);
      delay(500);
      digitalWrite(TX_LED, LOW);
    }
    else
    {
      // Transmission loop
      // Read pots
      reading = analogRead(Ang_PIN);
      angle = ((reading / 1023) * angleRange) - (angleRange / 2); // calculate input angle
      setAngle = round(angle);

      // Store angle into msg
      sprintf(msg, "%d %.1f %.2f %.2f;", setAngle, Kp, Ki, Kd);

      // Print out what's being sent to serial
      sprintf(msglen, "\n[%d]: ", sendSize);
      Serial.print(msglen);
      delay(10);
      for (byte i = 0; i < sendSize; i++)
        Serial.print(msg[i]);
      delay(10);

      // Send confirmation message
      if (radio.sendWithRetry(GATEWAYID, msg, sendSize))
        Serial.print(" -- received by onboard");
      else Serial.print(" -- no response from onboard");
    }
    sendSize = (sendSize + 1) % sizeof(msg); // Increment to send next byte
  }
}
