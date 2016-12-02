//Sketch to exercise the Still Controller hardware
//  name:  still_control_test_11 (v1.1)
//  author:  Bob Glicksman;  date: 12/12/2011
// 
//  This is test code to exercise the Curbie open source still controller hardware, albeit some functions are intended
//  to be permanent "drivers", as described below.  This code uses a 74HC165 octal shift register
//  to read in the control pushbutton switches, with provision for 4 additional digital inputs.  A 74HC595 octal shift
//  register provides 8 digital outputs, which are presently connected to 8 LEDs for testing purposes.
//  The LEDs will be replaced/augmented by hardware drivers (e.g. SSRs) for the various alarms, solenoid valves,
//  motors, and heater control of the still.  Drivers and test code for three DS18B20 one-wire temperature
//  sensors and two types of I2C LCD display are included in this version.  Drivers and test code for an audiable 
//  alert device is not included at this time.
//
//  This version contains support for the B2QCshop 20x4 LCD display with integral I2C backpack, as well as the
//  Adafruit 20x4 LCD with the Adafruit I2C backpack kit.  It is only necessary to change the "#define Adafruit_LCD"
//  to "true" for the Adafruit display or to "false" for the B2QCshop display.  Note that the Adafruit LCD
//  requires installation of the patched "LiquidCrystal_TWI" library and the B2QCshop LCD requires installation
//  of the "LiquidCrystal_I2C" library.  Both libraries can be installed; they do not conflict.
//
//  This version supports both active_high and active_low output controls from the 74HC595 output shift register.  
//  This feature provides flexibility in driving high voltage and high current AC and DC loads, as some opto-isolator
//  modules and some relay drivers require an active_low control.  The global constant "INVERT_MASK" is used to 
//  invert designated control bits when shifting output control data to the 74HC595 shift register.  
//  Set the bit corresponding bit in INVERT_MASK to a 1 for an active-low control; otherwise set the corresponding 
//  bit to 0 for an active_high control.
//
//  A set of pushbutton switch codes and output register masks are defined as named constants up front of the sketch to make
//  functional coding simple and readable.
//
//  TESTING:  This software is used to test the still controller hardware. When everything is correctly connected
//    per the schematic, the software should display a top level screen as follows:
//    line 0:  "Top"  the temperature of sensor(0) as F or C, depending upon the global constant Centigrade,
//      then "WATER" and then spinner(0), stopped (X)
//    line 1:  "Mid"  the temperature of sensor(1) as F or C, depending upon the global constant Centigrade,
//      then "WATER" and then spinner(1), stopped (X)
//    line 2:  "Botm"  the temperature of sensor(2) as F or C, depending upon the global constant Centigrade
//      then "WATER" and then spinner(2), stopped (X)
//    line 3:  "Button:"  and then the name of a pushbutton switch [in square brackets] whenever that button is depressed
//      and then spinner(3), stopped (X), if the global constant "numberOfSpinners" equals 4 (it is 3 by default)
//
//  Depressing a pushbutton switch results in the following actions, when the hardware is working correctly:
//    "MENU": toggles the LEDs "HEATING_ELEMENT"  and "ALERT" on and off, and displays "MENU" in the square 
//      brackets on line 3
//    "CENTER": toggles the LEDs "WATER_FEED_PUMP" and "ALARM" on and off, displays "CENTER" in the square 
//      brackets on line 3, and toggles spinner(1) active, inactive
//    "UP": toggles the LEDs "BEER_FEED_PUMP" and "SPARE1" on and off, displays "UP" in the square 
//      brackets on line 3, and toggles spinner(0) active, inactive
//    "DOWN": toggles the LEDs "Y_VALVE" and "PRODUCT_VALVE" on and off, displays "DOWN" in the square 
//      brackets on line 3, and toggles spinner(2) active, inactive
//      
//  Known bugs and issues:
//  (1) If multiple pushbutton switches are activated simultaneously, the code for first switch tested for is returned.  
//    If a later switch code is repeatedly activated, the first switch code will be returned that number of times.  
//    For example, if the MENU switch is held down and UP is repeatedly pressed, the code for MENU will be 
//    returned each time UP is depressed.  This is a minor issue as only one switch function should be activated 
//    at any given time, as the switches are momentary pushbuttons and are not intended to be held depressed.

/********************************* GLOBAL CONSTANTS AND VARIABLES ******************************************/
#define DEBUG false  // debug mode sends debugging data to the serial monitor via the serial port.
#define Adafruit_LCD false  // true for Adafruit LCD, false for B2QCshop LCD

// includes for the DS18B20 temperature sensors
#include <OneWire.h>    // the one wire bus library
#include <DallasTemperature.h>  // the DS18B20 chip library -- uses the OneWire library

// includes for the LCD/i2c
#include <Wire.h>      // the i2c library:  clock on Analog pin 5; data on analog pin 4
#include <LiquidTWI.h>  // the high performance library (patched per Adafruit chat)
#include <LiquidCrystal_I2C.h> // the B2QC LCD library (also high performance)

// Define the data, clock and latch pins for the input and output shift registers
//  output shift register is 74HC595 (lights 8 LEDs)
const int out_data = 6;
const int out_clock = 5;
const int out_latch = 4;

// input shift register is 74HC165 (reads 5 way switch, menu switch, 2 spare inputs)
const int in_data = 7;
const int in_clock = 9;
const int in_latch = 8;

// constants for the control panel switches
//  codes below are returned when a switch is activated, or NO_ACTION.  Multiple switch
//  activations result in the first switch in the list being the returned value.
const int NO_ACTION =  0x00;
const int MENU =       0x01;
const int CENTER =     0x02;
const int UP =         0x03;
const int DOWN =       0x04;
//const int LEFT =       0x05;  // left and right are no longer used in the controller design
//const int RIGHT =      0x06;  // left and right are no longer used in the controller design
const int ERROR =      0xFF;  //just in case -- should never occur!
const unsigned long DEBOUNCE_TIME = 20L;  //20 milliseconds to debounce switches.

// constants for the digital outputs
//  codes below can be used in the setOutput(output, value) function.
//  Output codes:
const byte HEATING_ELEMENT =   B00000001;
const byte WATER_FEED_PUMP =   B00000010;
const byte BEER_FEED_PUMP =    B00000100;
const byte Y_VALVE =           B00001000;
const byte PRODUCT_VALVE =     B00010000;
const byte SPARE1 =            B00100000;
const byte ALARM =             B01000000;
const byte ALERT =             B10000000;

const byte INVERT_MASK =       B00000000;  // set a bit to 1 for active low, 0 for active high

// Value codes:
const boolean OFF = false;
const boolean ON = true;

// Constants for the DS18B20 temperature sensor switches
const int oneWireBusPin = 3;            // DIO pin #3 is the one wire bus for all DS18B20 sensors
const int temperatureResolution = 12;   // 12 bit resolution is the maximum for these devices
const int numberOfSensors = 3;          // the still uses three sensors -- change if the still design changes

// Constants for the LCD "spinners"
byte backSlash[8] = {B00000, B10000, B01000, B00100, B00010, B00001, B00000};  // make a backslash character
const char spinner[] = {'|', '/', '-', 0}; // sequence of symbols for the spinner, loc 0 is the backslash char
const byte numberOfSpinners = 3;
const long spinnerDelayTime = 35L;    // 35 milliseconds delay for spinner asthetics

// Global variables for pushbutton switches and output controls
int switchCount; //test the debouncing - count should only increase by 1 for each switch activation
volatile byte output; //variable to hold the contents for the output shift register

// Global variables for DS18B20 sensors
OneWire oneWire(oneWireBusPin);   // create an instance of the one wire bus
DallasTemperature sensors(&oneWire);  // create instance of DallasTemperature devices on the one wire bus
DeviceAddress reboilerTemperatureProbeAddress;  // array to hold the device ID code for the reboiler/bottom temperature probe chip
DeviceAddress midColumnTemperatureProbeAddress;  // array to hold the device ID code for the mid column temperature probe chip
DeviceAddress productTemperatureProbeAddress;  // array to hold the device ID code for the product/top temperature probe chip
volatile float reboilerTemperature = 0.0;    // the current value of the reboiler temperature probe reading
volatile float midColumnTemperature = 0.0;    // the current value of the mid column temperature probe reading
volatile float productTemperature = 0.0;    // the current value of the product temperature probe reading
boolean Centigrade = false;                  // false for Farenheight, true for Centigrade

// Global variables for LCD
LiquidTWI lcdA(0); // create an Adafruit LCD instance on the i2c bus, address 0
LiquidCrystal_I2C lcdB(0x27, 20, 4); // create a B2QCshop LCD instance on the i2c bus, address 0x27
#if Adafruit_LCD
  #define lcd lcdA
#else
  #define lcd lcdB
#endif
boolean spinnerState[] = { OFF, OFF, OFF, OFF };  // initialize 4 spinners to OFF

/********************************* END OF GLOBAL CONSTANTS AND VARIABLES ***********************************/

/********************************* BEGINNING OF setup() ****************************************************/
void setup()
{
  // define the Uno pins for the output shift register 74HC595
  pinMode (out_data, OUTPUT);
  pinMode (out_clock, OUTPUT);
  pinMode (out_latch, OUTPUT);
 
  // define the Uno pins for the input shift register 74HC165 
  pinMode (in_data, INPUT);
  pinMode (in_clock, OUTPUT);
  pinMode (in_latch, OUTPUT);
  
  //debug mode -- send switch activation code and count to the serial monitor
  if (DEBUG)
  {
    Serial.begin (9600);
    switchCount = 0; //test the debouncing
  }
  
  // clear all output controls at the start
  updateOutputShiftRegister(B00000000);
  
  // detect the DS18B20 temperature sensors and get their addresses
  sensors.begin();      // startup the temperature sensor instance
  if (DEBUG)
  {
    int numSensors = sensors.getDeviceCount();  // how many are found on the bus?
    if ( numSensors == numberOfSensors)
    {
      Serial.println ("Found all devices - OK!");
    }  else
    {
      Serial.print ("Found ");
      Serial.print ( numSensors );
      Serial.println (" sensors - ERROR!!");
    }
  }
  
  // Get the addresses for each temperature sensor -- assume they are in index order
  //  or the addresses can be manually set as an alternative
  sensors.getAddress ( reboilerTemperatureProbeAddress, 0 );
  sensors.getAddress ( midColumnTemperatureProbeAddress, 1 );
  sensors.getAddress ( productTemperatureProbeAddress, 2 );
  sensors.setResolution ( temperatureResolution ); // set the resolution of all temperature sensors to 12 bits
  sensors.setWaitForConversion ( false ); // set the sensors for non-blocking operation
  
 // Set up the LCD - always part of setup()
  if ( Adafruit_LCD )
  {
    lcdA.begin ( 20, 4 );  // startup the Adafruit lcd instance for 20x4 LCD
  }
  else
  {    
    lcdB.init();  // startup the B2QCshop lcd instance
  }
  
  lcd.setBacklight ( HIGH );
  lcd.createChar ( 0, backSlash ); // Write the backslash char into LCD char loc 0
  
// Display the main screen modified for hardware testing 
  lcd.clear();
  
  // first line
  lcd.setCursor ( 0, 0 ); 
  lcd.print ( "Top  xxx.x" );
  lcd.setCursor ( 10, 0 );
  if ( Centigrade )
  {
    lcd.write ( 'C' );
  } else
  {
    lcd.write ( 'F' );
  }
  lcd.setCursor ( 11, 0 );
  lcd.print ( "  WATER " );
  
  // second line
  lcd.setCursor ( 0, 1 ); 
  lcd.print ( "Mid  xxx.x" );
  lcd.setCursor ( 10, 1 );
  if ( Centigrade )
  {
    lcd.write ( 'C' );
  } else
  {
    lcd.write ( 'F' );
  }
  lcd.setCursor ( 11, 1 );
  lcd.print ( "  FEED  " ); 
 
  // third line
  lcd.setCursor ( 0, 2 ); 
  lcd.print ( "Botm xxx.x" );
  lcd.setCursor ( 10, 2 );
  if ( Centigrade )
  {
    lcd.write ( 'C' );
  } else
  {
    lcd.write ( 'F' );
  }
  lcd.setCursor ( 11, 2 );
  lcd.print ( "  HEAT  " ); 

  // fourth line for testing hardware
  lcd.setCursor ( 0, 3 );
  lcd.print ( "Button:" );
  
}
/********************************* END OF setup() **********************************************************/

/********************************* BEGINNING OF loop() *****************************************************/
// The main loop reads 8 inputs directly from the input shift register.  As each switch is tested, it toggles one or
//  more LEDs as a test of the hardware control functions.  All temperature sensors are also read, in a non-blocking manner.

void loop()
{ 
  int switchValue; //hold the pushbutton switch return code
  
  //test the readSwitches function with debouncing
  switchValue = readSwitches();
  
  //set the output LEDs according to the switch code returned
  switch (switchValue)
  {
    case NO_ACTION:
      break;
      
    case MENU:
      lcd.setCursor ( 8, 3);
      lcd.print ( "[MENU   ]" );
      if ( getOutputStatus(HEATING_ELEMENT) == OFF )
      {
        setOutput(HEATING_ELEMENT, ON);
        setOutput(ALERT, ON);        
      } else
      {
        setOutput(HEATING_ELEMENT, OFF);
        setOutput(ALERT, OFF);   
      }
      break;
      
    case CENTER:
      lcd.setCursor ( 8, 3);
      lcd.print ( "[CENTER ]" );    
      if ( getOutputStatus(WATER_FEED_PUMP) == OFF )
      {
        setOutput(WATER_FEED_PUMP, ON);
        setOutput(ALARM, ON); 
        setSpinner (1, ON);       
      } else
      {
        setOutput(WATER_FEED_PUMP, OFF);
        setOutput(ALARM, OFF); 
        setSpinner (1, OFF);
      }
      break;
      
    case UP:
      lcd.setCursor ( 8, 3);
      lcd.print ( "[UP     ]" );      
      if ( getOutputStatus(BEER_FEED_PUMP) == OFF )
      {
        setOutput(BEER_FEED_PUMP, ON);
        setOutput(SPARE1, ON);       
        setSpinner (0, ON);
      } else
      {
        setOutput(BEER_FEED_PUMP, OFF);
        setOutput(SPARE1, OFF); 
        setSpinner (0, OFF);
      }    
      break;
      
    case DOWN:
      lcd.setCursor ( 8, 3);
      lcd.print ( "[DOWN   ]" );  
      if ( getOutputStatus(Y_VALVE) == OFF )
      {
        setOutput(Y_VALVE, ON);
        setOutput(PRODUCT_VALVE, ON);        
        setSpinner (2, ON);
      } else
      {
        setOutput(Y_VALVE, OFF);
        setOutput(PRODUCT_VALVE, OFF);         
        setSpinner (2, OFF);
      }        
      break;     
      
    case ERROR:
      updateOutputShiftRegister(B11111111);
      break; 
 
    default:
      updateOutputShiftRegister(B10101010); 
  }   
      
  if (switchValue != NO_ACTION)
  {
    if (DEBUG)
    {
      switchCount++;
      Serial.print (switchValue, HEX);
      Serial.print (" ; ");
      Serial.println (switchCount);
    }
  }
 
  // Test the DS18B20 temperature sensor values   
  if ( readTemperatureSensors() == true )
  {
    lcd.setCursor ( 5, 0 );
    formattedPrint ( productTemperature );
    lcd.setCursor ( 5, 1 );
    formattedPrint ( midColumnTemperature );
    lcd.setCursor ( 5, 2 );
    formattedPrint ( reboilerTemperature );
        
    if (DEBUG)
    {
      Serial.print ( "--> reboilder temperature: ");
      Serial.println ( reboilerTemperature );
      Serial.print ( "--> mid column temperature: ");
      Serial.println ( midColumnTemperature );
      Serial.print ( "--> product temperature: ");
      Serial.println ( productTemperature );
      Serial.println ( "" );
    }
  }
  
  // To make the LCD spinners work, must call spin() once per loop 
  spin ();
  
}  //end of loop() function
/********************************* END OF loop() ***********************************************************/

/********************************* BEGINNING OF readSwitches() *********************************************/
// function to determine if a pushbutton switch has been activated.  If so, returns the switch code constant representing
//  the activated switch.  If no switch has been activated, NO_ACTION is returned.  If multiple switches are
//  activated (generally precluded by the mechanics of the switches, but possible nonetheless), the first switch activation
//  found is returned.
//
// The function reads the input shift register data and masks off the non-switch bits.  It stores the current state of the
//  switches in a static variable.  It also stores a static boolean representing a change in raw switch data, for debouncing.
//  When new switch data from the input shift register differs from the previously stored value, the boolean "debouncing" is set true and
//  the system time is stored.  When the function is entered with the boolean "debouncing" set true, the current time is compared to the
//  previously stored time to see if the debounce period has eneded.  If so, the input shift register data is again sampled and if
//  the state is the same as previously, a switch activation is declared and the swtich value is encoded and returned.
//
// This function is non-blocking.  It returns immediately and does not block on the debounce time.  Debouncing is determined dynamically
//  each time the function is called.

int readSwitches()
{
  static unsigned long changeDetectionTime;  // variable to hold the system time for debouncing
  static boolean debouncing = false; //set true if debouncing
  static byte switchState = 0; //hold the last read state of the input shift register for debouncing
  unsigned long newTime; // hold the current system time
  byte newSwitchState; // hold the current data from the input switch register
  unsigned long timeInterval;
  
  if (debouncing) //code if in the process of debouncing switches
  {
    newTime = millis(); //get the new time
    timeInterval = diff (newTime, changeDetectionTime);
    if ( timeInterval < DEBOUNCE_TIME ) //still waiting on debounce
    {
      return NO_ACTION;
    } else    //debounce time expired - recheck switches and return code
    {
      newSwitchState = getShiftRegisterData();
      newSwitchState = newSwitchState & 0x0F; //mask off extra 4 bits
      debouncing = false; //reset debounce flag
      if (newSwitchState == switchState) //confirmed switch data
      {
        if ( (switchState == 0) ) return NO_ACTION; // all switches released
        if ( (switchState & 0x01) != 0 ) return MENU; 
        if ( (switchState & 0x02) != 0 ) return CENTER;        
        if ( (switchState & 0x04) != 0 ) return UP;   
        if ( (switchState & 0x08) != 0 ) return DOWN;   
        return ERROR; // just in case something went wrong with the code!
      }  else  // switch action not confirmed -- just noise
      {
        return NO_ACTION; 
      }     
    }
  } 
 
  else  //code if not debouncing
  {
    newSwitchState = getShiftRegisterData();
    newSwitchState = newSwitchState & 0x3F; //mask off extra bits
    if (newSwitchState == switchState) //no change in the shift register data
    {
      return NO_ACTION;
    } else    //shift register data has changed -- debounce
    {
      switchState = newSwitchState;  //store the new switch state
      changeDetectionTime = millis(); //store time time for debouncing
      debouncing = true; //set debouncing flag
      return NO_ACTION; //no decision until after debouncing time and re-verification
    }
  }
}
/********************************* END OF readSwitches() ***************************************************/

/********************************* BEGINNING OF getShiftRegiesterData() ************************************/
// function to read in the data from the 74HC165 shift register.  One unsigned byte of data is returned.
//  arguments:  none.
//  return:  one byte of data representing:
//    bit 0: MENU button depressed
//    bit 1: pushbutton switch CENTER depressed
//    bit 2: pushbutton switch UP depressed
//    bit 3: pushbutton switch DOWN depressed
//    bit 4: spare (formally LEFT button depressed)
//    bit 5: spare (formally RIGHT button depressed)
//    bit 6: spare - not presently connected
//    bit 7: spare - not presently connected
//  It is possible to have multiple bits set in the returned byte.  The returned byte just contains the present status 
//  of all 8 74HC165 parallel inputs.

byte getShiftRegisterData()
{
  byte shiftData = 0;
  digitalWrite (in_clock, HIGH); //initialize
  
   //sample the switches
  digitalWrite (in_latch, LOW);
  digitalWrite (in_latch, HIGH);
  
  //read in the shift register data
  shiftData = shiftIn (in_data, in_clock, MSBFIRST);
  return shiftData;
}
/********************************* END OF getShiftRegiesterData() ******************************************/

/********************************* BEGINNING OF getOutputStatus() ******************************************/
// function to test the value of the output global variable the reflects the current status of the 
//  74HC595 output shift register contents.  
//  arguments:
//    control: the mask code for the output control to be tested.  See the defined constants.
//    return:  the result as ON or OFF based upon the defined constants.
boolean getOutputStatus(byte control)
{
  byte result;
  result = output & control;  //mask off all but the selected code
  if (result == 0)
  {
    return OFF;
  } else
  {
    return ON;
  }
}
/********************************* END OF getOutputStatus() ************************************************/

/********************************* BEGINNING OF setOutput() ************************************************/
// function to set/reset output controls individually.  The bit masks in the defined constants are used to set or clear
//  individual bits in the global variable "output", whose contents reflect the current state of the 74HC595 shift
//  register.  When the new value of output has been computed, updateOutputShiftRegister() is used to transfer
//  the values to the hardware.
//  arguments:
//    control: the mask code for the output control to be set/reset.  See the defined constants. 
//    state: ON or OFF value to set the control to.  See the defined constants.
void setOutput(byte control, boolean state)
{
  if (state == ON)
  {
    output = output | control;  //set the proper bit on leaving all others alone
  } else
  {
    output = output & (~control); //reset the proper bit off leaving all others alone
  }
  updateOutputShiftRegister(output);  //transfer the output byte to the output shift register
}
/********************************* END OF setOutput() ******************************************************/

/********************************* BEGINNING OF updateOutputShiftRegister() ********************************/
// updateOutputShiftRegister: use "shiftOut()" to transfer a byte 
//    to the 74HC595 shift register
//  arguments:  one byte of data representing (1 = ON, 0 = OFF):
//    bit 0: the HEATING_ELEMENT
//    bit 1: the WATER_FEED_PUMP
//    bit 2: the BEER_FEED_PUMP
//    bit 3: the Y_VALVE
//    bit 4: the PRODUCT_VALVE
//    bit 5: spare output - not presently used
//    bit 6: ALARM
//    bit 7: ALERT audiable indicator

void updateOutputShiftRegister(byte value)
{
  digitalWrite (out_latch, LOW);
  shiftOut (out_data, out_clock, MSBFIRST, value ^ INVERT_MASK);
  digitalWrite (out_latch, HIGH);
}
/********************************* END OF updateOutputShiftRegister() **************************************/

/********************************* BEGINNING OF readTemperatureSensors() ***********************************/
// Function to perform non-blocking reading of the DS18B20 Temerature sensors
//  Returns true if there is a new reading (750 ms from command)
//  also updates the global variables with the temperatures read

boolean readTemperatureSensors()
{
  const unsigned long readingTime = 750L;  //DS18B20 spec wait time for 12 bit reading
  
  static boolean readingInProgress = false; 
  static unsigned long lastTime;
  static unsigned long newTime;
  
  unsigned long timeInterval;
  
  if ( !readingInProgress ) // idle
  {
    sensors.requestTemperatures(); // get all sensors started reading
    readingInProgress = true;
    lastTime = millis();
    
  }  else // waiting on conversion completion
  {
     newTime = millis();
     timeInterval = diff (newTime, lastTime);
     if (timeInterval < readingTime)  // still reading 
     {
       return false;  // no new temps
     } else  // conversion complete -- get the values in degrees F
     {
       if (Centigrade)
       {
         reboilerTemperature = sensors.getTempC ( reboilerTemperatureProbeAddress );
         midColumnTemperature = sensors.getTempC ( midColumnTemperatureProbeAddress );
         productTemperature = sensors.getTempC ( productTemperatureProbeAddress );
       }  else
       {
         reboilerTemperature = sensors.getTempF ( reboilerTemperatureProbeAddress );
         midColumnTemperature = sensors.getTempF ( midColumnTemperatureProbeAddress );
         productTemperature = sensors.getTempF ( productTemperatureProbeAddress );
       }
       readingInProgress = false;
       return true; // new temps available 
     }

  }
}  
/********************************* END OF readTemperatureSensors() *****************************************/

/********************************* BEGINNING OF formattedPrint() *******************************************/
// Function to print temperatures in 3.1 format
//  arguments:
//    value: the floating point number to convert to a proper 3.1 string
//  also prints the resulting string to the LCD display at the current cursor position
void formattedPrint( float value)
{
  float temp;
  int integralPart;
  int decimalPart;
  String printString;
  
  temp = (value * 10.0F) + 0.5F;
  integralPart = (int)temp / 10;
  decimalPart = (int)temp % 10;
  if ( integralPart < 100 )  // add leading bland for 2 digit value
  {
    printString = " ";
  } else
  {
    printString = "";
  }
  printString = printString + String(integralPart);
  printString = printString + '.';
  printString = printString + String(decimalPart);
  lcd.print(printString);
}
/********************************* END OF formattedPrint() *************************************************/

/********************************* BEGINNING OF setSpinner() ***********************************************/
// Function to set the state of the specified spinners ON or OFF
//  arguments:
//    spinnerNumber - the number (0, 1, 2, or 3) of the spinner
//    state - boolean: ON or OFF
//  also sets the boolean in the global array "spinnerState" to ON or OFF
void setSpinner (int spinnerNumber, boolean state)
{
  spinnerState[spinnerNumber] = state;
}
/********************************* END OF setSpinner() *****************************************************/

/********************************* BEGINNING OF spin() *****************************************************/
// Spin each spinner that is on one step increment of the pattern
//  This function takes no arguments.  It uses the global array "spinnerState" to determine
//    whether to spin or not to spin a spinner.  Each time this function is called, it increments to
//    the next spinner, from spinner(0) to spinner(numberOfSpinners - 1).  If there are 4 spinners
//    total, it will take four calls to this function to advance each spinner one step in the 
//    dynamic spin display.  This keeps the spinner timing the same regardless of hoq many spinners
//    are activated and spinning.
void spin ()
{
  static int symbol[] = { 0, 0, 0, 0 }; // array to hold current spinner symbol for each spinner (max 4 spinners)
  static unsigned long lastTime = 0L;
  static byte spinnerNumber = 0;
  unsigned long newTime;
  
  newTime = millis(); 
  if ( diff (newTime, lastTime) > spinnerDelayTime)  // Time to update the next spinner
  {
    lcd.setCursor ( 19, spinnerNumber );
    if ( spinnerState[spinnerNumber] == ON )  // write the next spinner symbol
    {
      if ( symbol[spinnerNumber] > 2)  // determine the next spinner symbol
      {
        symbol[spinnerNumber] = 0;
      } else
      {
        symbol[spinnerNumber]++;
      }      
      lcd.write ( spinner[symbol[spinnerNumber]] );
    } else                        // write a "-"
    {
      lcd.write ( 'X' );  // stopped spinner is a "X"
    }    
    if (spinnerNumber < (numberOfSpinners - 1)) // set the next spinner number
    {
      spinnerNumber++;
    }  else
    {
      spinnerNumber = 0;
    }
    lastTime = newTime;
  }
}
/********************************* END OF spin() ***********************************************************/

/********************************* BEGINNING OF diff() *****************************************************/
// Function to subtract a new and old millis() reading, correcting for millis() overflow
//  arguments:
//    newTime - the current, updated time from millis()
//    oldTime - the previous time, from millis() for comparision for non-blocking delays
//  returns:  the time difference, correcting for millis() overflow which occurs one every 70 days
unsigned long diff ( unsigned long newTime, unsigned long oldTime )
{
  const unsigned long maxTimeValue = 0xFFFFFFFF;  // max value if millis() before overflow to 0x00000000
  long timeInterval;
 
  if ( newTime < oldTime )  // overflow of millis() has occurred -- fix
  {
    timeInterval = newTime + (maxTimeValue - oldTime) + 1L;
  }  else
  {
    timeInterval = newTime - oldTime;
  }
  return timeInterval;
}
/********************************* END OF diff() ***********************************************************/
