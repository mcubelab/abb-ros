// Hand Encoder Server
//
// Last Modified: 08/12/2011
//
// Author: Matt Bernstein, Robbie Paolini, Alberto Rodriguez
//
// Interface with the arduino over serial port, at baud rate
// BAUD_RATE. Arduino broadcasts encoder values. The format 
// is a series of "y@xxxx#" where y is an integer corresponding
// to the arduino pin where the encoder is connected and xxxx 
// is an integer value corresponding to the pulse width. If a 
// pulse is not found, NO_VALUE is returned.

/////////////////////////
// Constants
/////////////////////////

// First port that an encoder is connected to. Note that it is 
//  assumed encoders are connected to consecutive pins, eg:
//  FIRST_PIN, (FIRST_PIN+1), (FIRST_PIN+2) ... LAST_PIN
#define FIRST_PIN 5    
#define LAST_PIN 7  //Implicit number of fingers
#define NUM_FINGERS 3

// Number of microseconds to wait for a high pulse before giving up 
//  on a certain encoder
#define PIN_TIMEOUT 10000

// If we timeout, we return this value, so the computer knows 
//  when to ignore values coming from the arduino
#define NO_VALUE -9999

// Serial baud rate to communicate at
#define BAUD_RATE 115200

// Maximum pulse width
#define MAX_PULSE 4096

//////////////////////
// Global variables
//////////////////////
// Current Pin to read and broadcast
int currPin;

////////////////////////////////////////////
// Initialization (Called on Arduino Boot)
////////////////////////////////////////////

void setup()
{
  //Initialize input pins
  for (int i=0; i < NUM_FINGERS; i++)
    pinMode(FIRST_PIN - i, INPUT);
  
  //Initialize pin to read in
  currPin = FIRST_PIN;	
  
  //Initialize serial communication with computer
  Serial.begin(BAUD_RATE);    
  Serial.flush();
}

void loop() 
{
  // Read the high pulse length for currPin
  int high = pulseIn(currPin, HIGH, PIN_TIMEOUT);
  // Read the low pulse length for currPin
  int low = pulseIn(currPin,LOW, PIN_TIMEOUT);
    
  int enc = (int)((MAX_PULSE + 2)*((double)high/((double)(high + low)))) - 1;
  if(enc >= MAX_PULSE)
	enc = MAX_PULSE - 1;

  enc = enc + 1;

  // If 'pulseIn' returned 0, it couldn't find a pulse, 
  //  so return NO_VALUE
  if ((high == 0) || (low ==0))
    enc = NO_VALUE;

  // If we happen to be right at the boundary, make sure we
  //  don't return anything that's out of range
  //if (enc > MAX_PULSE)
  //  enc = MAX_PULSE;
        
  // Load this new value onto the serial port
  Serial.print(LAST_PIN - currPin);
  Serial.print('@');
  Serial.print(enc);
  Serial.print('#');

  currPin = currPin + 1;
  if(currPin> LAST_PIN)
    currPin = FIRST_PIN;
}
