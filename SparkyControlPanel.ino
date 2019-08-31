/*This is code for the 3 common Sparky Control Panels
 * with 7 segment

This sketch reads button, joystick and knob controls and sends command values to the Sparky robot.
It has a debug flag turned off for normal running.

Developed by Miss Daisy FRC Team 341
*/

//  this is for test mode
#include <AltSoftSerial.h>
AltSoftSerial altser;
const int mybaud = 600;
boolean runTimeMonitorEnabled = false;
// D8 - AltSoftSerial RX
// D9 - AltSoftSerial TX

// this is for 7 segment display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
Adafruit_7segment matrix = Adafruit_7segment();

// this is for send and recieve bluetooth communication channels
#include <EasyTransfer.h>
//create two objects
EasyTransfer ETin, ETout; 
// Transfer data strucures must be the same at both ends
//   so, they are defined in a library included by both sketches
#include <SparkyXfrBuffers.h>
// declare global structures for data used by the transfer objects
// reverse naming on opposite ends
FROM_SPARKY_DATA_STRUCTURE rxdata;
TO_SPARKY_DATA_STRUCTURE txdata;

//#define DRIVE_MODE        13 
#define SYSTEM_ENABLE   12
//  #define PANEL_LED_5     11  panel LEDs are controlled by setLED function, 0 to 4
#define ENABLE_LED_3     3    // pin 10  
// pin 9 used by altser
#define SHOOT_BUTTON    8
#define INTAKE_BUTTON   7
  #define INTAKEBUTTON_LED     0   //pin 5
  #define SHOOTBUTTON_LED     1    //pin 6
#define TEST_SWITCH     4
#define R_STICK_BUTTON  3
#define HC05_POWER_ON_LOW_2  2

//#define L_STICK_Y      1
#define R_STICK_X    0    // vertical forward-backward stick axis attached to A0
//#define R_STICK_Y      1
#define L_STICK_X    2    // horizontal left-right-turn stick axis attached to A2
#define SHOOTERSPEED 3    // shooter peed knob attached to A3

unsigned long triggerTime;
unsigned long headingTime;
const int panelLedArr[5] = {5,6,9,10,11}; //map of wired pins to LEDs
long int messageCounter = 0;

/////////////////////////////////////////////////////////////////////////////
// FUNCTION: setLED,   returns nothing
// ARGUMENTS: LEDnum is value 0 to 4, brightness is 0 (off) to 255 (full on)
void setLED(int LEDnum, unsigned int brightness) {
  if ( brightness > 255 ) brightness = 255;
  signed long brightness_l = brightness; 
  if ( 0 <= LEDnum && LEDnum < 2 ) {
    //  index to pins, use panelLedArr   ,  value to write is non-linear
    // these LEDs are high active
    int LEDoutput = max( ((brightness_l+7)/8), (brightness_l*3)-510 );
    analogWrite( panelLedArr[LEDnum], LEDoutput );
  } else if ( LEDnum < 5 ) {
    //  index to pins, use panelLedArr   ,  value to write is non-linear
    // these LEDs are LOW active
    int LEDoutput = min( 255-((brightness_l+7)/8), (255-brightness_l)*3 );
    analogWrite( panelLedArr[LEDnum], LEDoutput );
  }
}
/////////////////////////////////////////////////////////////////////////////////
// called once at start
void setup(){
  digitalWrite(HC05_POWER_ON_LOW_2, HIGH );  // make sure it starts up OFF
  pinMode(HC05_POWER_ON_LOW_2  , OUTPUT); // 2 LOW is active

  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  
  //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
  ETin.begin(details(rxdata), &Serial);
  ETout.begin(details(txdata), &Serial);

  altser.begin(600);

  // Wire. is a TwoWire type object declared in the include file Wire.h
  Wire.begin(); // join i2c bus (address optional for master)
  // 2wire code is at the end of loop - end of this file

  matrix.begin(0x70);

  matrix.writeDigitNum(0, 8 , true);
  matrix.writeDigitNum(1, 8 , true);
  matrix.drawColon(true);
  matrix.writeDigitNum(3, 8 , true);
  matrix.writeDigitNum(4, 8 , true);
  matrix.writeDisplay();

  digitalWrite(HC05_POWER_ON_LOW_2, LOW );  // now turn the HC05 on 

//  //  init LEDs     //////////////////////////
  for (int i=0; i<5; i++) {
    pinMode( panelLedArr[i], OUTPUT);
    setLED( i, 255); // on
    delay(500);
    setLED( i, 0);
    delay(500);
  }

  matrix.writeDigitRaw(0, 0 );
  matrix.writeDigitRaw(1, 0 );
  matrix.drawColon(false);
  matrix.writeDigitRaw(3, 0 );
  matrix.writeDigitRaw(4, 0 );
  matrix.writeDisplay();

 
  // init inputs and enable pullup
  pinMode(13              , INPUT_PULLUP); // 13 unused
  pinMode(SYSTEM_ENABLE   , INPUT_PULLUP); // 12 LOW is ENABLEd
  pinMode(SHOOT_BUTTON    , INPUT_PULLUP); // 8 LOW is SHOOT
  pinMode(INTAKE_BUTTON   , INPUT_PULLUP); // 7 LOW is INATKE
  pinMode(TEST_SWITCH     , INPUT_PULLUP); // 4 LOW is on
  pinMode(R_STICK_BUTTON  , INPUT_PULLUP); // 3 LOW is active   not used
  // other pins setup before this
  
  triggerTime = millis() + 3000;  // 3 seconds from now
  delay(2000);
  altser.println();
  altser.print("Universal Sparky Control Panel :Created ");
  altser.print( __DATE__ );
  altser.print(" ");
  altser.println( __TIME__ );
  if ( digitalRead(TEST_SWITCH) == HIGH ) { // LOW is on
    runTimeMonitorEnabled = false;
    altser.println("Runtime Monitor is disabled");
  }
}
/////////////////////  MAIN LOOP  /////////////////////////////
void loop(){
  static unsigned long wireTimer0,wireTimer1;
  
  // read our potentiomete rs and buttons and store raw data in data structure for transmission
  txdata.stickLy = ( ( ( analogRead(R_STICK_X)-512) *3)/7)+512; 
  txdata.stickLx = txdata.stickLy; 
  txdata.stickLbutton = LOW;   // no STICK BUTTONs attached
  txdata.stickRy = ( ( ( ( 1023-(analogRead(L_STICK_X) ) ) -512 )*3)/7)+512; 
  txdata.stickRx = txdata.stickRy;  
  txdata.stickRbutton = LOW;

  txdata.drivemode = 1;  //2nd generation control panels only do tank mode.
  txdata.enabled = !digitalRead(SYSTEM_ENABLE);
  
  txdata.shooterspeed = analogRead(SHOOTERSPEED);

  int buttonValue = 240;
  if( !digitalRead(INTAKE_BUTTON) ){
    txdata.intake = 1;
    buttonValue -= 80;
  }
  else {
    txdata.intake = 0;
  }
  if( !digitalRead(SHOOT_BUTTON) ){
    txdata.shoot = HIGH;
    buttonValue -= 80;
  }
  else {
    txdata.shoot = LOW;
  }
  // Check if the controller is enabled
  if(txdata.enabled){
    buttonValue -= 80;
    messageCounter += 1;
    txdata.counter = messageCounter;
  }
  //then we will go ahead and send that data out
  ETout.sendData();
  
 //there's a loop here so that we run the recieve function more often then the 
 //transmit function. This is important due to the slight differences in 
 //the clock speed of different Arduinos. If we didn't do this, messages 
 //would build up in the buffer and appear to cause a delay.
  for(int i=0; i<5; i++){
    //remember, you could use an if() here to check for new data, this time it's not needed.
    ETin.receiveData();
    delay(10);   // short delay between receive attempts
  }
  
  //delay for good measure
  delay(10);

//  *********  RUNTIME DISPLAY or DISPLAY TESTING *****************
  // check is display test on
  if ( digitalRead(TEST_SWITCH) == LOW ) { // LOW is on
    // ****  DISPLAY TESTING ******
    if ( !runTimeMonitorEnabled ) {
      altser.println("Monitor Activated");
    }
    runTimeMonitorEnabled = true;
    unsigned long testnow = millis();
    // once per second 
    if ( testnow >= triggerTime ) {
      triggerTime = testnow + 2000;
      int value1 = (analogRead(SHOOTERSPEED)+3)>>2;
      setLED( 3, value1 );
      setLED( 4, buttonValue);

      // ****  AltSerial TESTING ****
      if ( headingTime > testnow ) {
        altser.print( rxdata.supplyvoltagereading);
        altser.print( ", " );
        altser.print( rxdata.transmitpacketcount );
        altser.print( ", " );
        altser.print( rxdata.packetreceivedcount );
        altser.print( ", " );
        altser.print( rxdata.shooterspeedecho );
        altser.print( ", " );
        altser.print( txdata.enabled );
        altser.print( ", " );
        altser.print( rxdata.ballready );
        altser.print( " [" );
        altser.print( txdata.stickLx );
        altser.print( " " );
        altser.print( txdata.stickLy );
        altser.print( " " );
        altser.print( txdata.stickRx );
        altser.print( " " );
        altser.print( txdata.stickRy );  
        altser.print( " " );
        altser.print( wireTimer0 );  
        altser.print( " " );
        altser.println( rxdata.spare2 );
       
      } else {
        altser.println( "VOLT XMIT RECV SSECHO D12 BALL LX ly RX RL" );
        headingTime = testnow + 20000;
      }
    }
  }
//////////////////////   NORMAL DISPLAY   ////////////////////////
  else {  // test mode OFF, show main robot control signals
    static unsigned long updateDue;
    unsigned long now;
    static int phase;
    unsigned int phasefade;
    if ( runTimeMonitorEnabled ) {
      altser.println("Monitor Deactivated");
    }
    runTimeMonitorEnabled = false;
    now = millis();
    if ( now > updateDue ) {
      updateDue = now + 100; // 10 updates per second, max
      phase++; phase &= 7;
      if ( phase < 4 ) {
        phasefade = 210 + (phase * 10);
      } else {
        phasefade = 210 + ( 7-phase) * 10;
      }
      if ( txdata.enabled ) {
        if ( rxdata.ballready ) {
          setLED( INTAKEBUTTON_LED, 0 );
          setLED( SHOOTBUTTON_LED, 255 );
        } else {
          if ( txdata.intake ) {
            // intake button pushed blink LED fast
            if ( phase & 1 ) {
              setLED( INTAKEBUTTON_LED, 255 );
            } else {
              setLED( INTAKEBUTTON_LED, 0  );
            }
          } else {
            // intake button not pushed fade LED slow
            setLED( INTAKEBUTTON_LED, phasefade );
          }
          setLED( SHOOTBUTTON_LED, 0 );
        }  
      } else {
        setLED( INTAKEBUTTON_LED, 0 );
        setLED( SHOOTBUTTON_LED, 0 );
      }
      // use enable LED to show states
      static int lastRXcount;
      static int lastTXcount;
      //these should change in 100 ms
      int commOK = (rxdata.packetreceivedcount != lastRXcount) && (rxdata.transmitpacketcount != lastTXcount); 
      lastRXcount = rxdata.packetreceivedcount;
      lastTXcount = rxdata.transmitpacketcount;
      if ( commOK ) {
        if ( txdata.enabled ) {
          setLED( ENABLE_LED_3, 255 );  // on
        } else{
          if (phase) {
            setLED( ENABLE_LED_3, 0 ); // pulsing means not enabled
          } else {
            setLED( ENABLE_LED_3, 255 ); // pulsing means not enabled
          }
          //analogWrite(10,255-phase);
        }
      } else {
        // no comm - flashing
        if ( phase & 1 ) {
          setLED( ENABLE_LED_3, 128 );
        } else {
          setLED( ENABLE_LED_3, 0  );
        }
      }
    }  
  } // end of NORMAL DISPLAY

  ///////////////////////  TWI code for 7segment display interface ///////////
  if ( wireTimer1 < (millis() - 500) ) {
    int calctemp;
    static int speeddisplay;
    
    wireTimer1 = millis(); // reset
    wireTimer0 = micros();

    calctemp = ((rxdata.supplyvoltagereading*15)+17) / 142; //tenths of a volt resolution
    matrix.writeDigitNum(0, (calctemp/10) % 10 , true);
    matrix.writeDigitNum(1, calctemp % 10 , false);
    matrix.drawColon(true);
    calctemp = (txdata.shooterspeed * 15) / 152;
    if ( abs(calctemp-speeddisplay)>1 ) {
      speeddisplay = calctemp;
    }
    matrix.writeDigitNum(3, (speeddisplay / 10) , false);
    matrix.writeDigitNum(4, speeddisplay % 10, false);
 
    matrix.writeDisplay();

    wireTimer0 = micros() - wireTimer0;     // how long did this take?
  }
}

