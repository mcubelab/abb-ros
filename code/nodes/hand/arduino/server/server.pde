// Hand Encoder Server
//
// Last Modified: 08/12/2011
//
// Author: Matt Bernstein, Robbie Paolini
//
// Interface with the arduino over serial port, at baud rate
// BAUD_RATE. To get the value of a given encoder, sending
// "00n#" where n is the encoder number, starting at 1, will
// result in a response of "xxxx#" where xxxx is an integer
// value corresponding to the pulse width. If a pulse is not
// found, NO_VALUE is returned.

/////////////////////////
// Constants
/////////////////////////
// Number of fingers on the hand
#define NUM_FINGERS 3

// First port that an encoder is connected to. Note that it is 
//  assumed encoders are connected in decreasing order, eg:
//  FIRST_PIN, (FIRST_PIN-1), (FIRST_PIN-2) ...
#define FIRST_PIN 7    

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
// Serial buffer for communication with computer
char buffer[256];  
// Counter keeps track of current location in our serial buffer
int counter;
// Holds the strings we should compare for each finger request
char req_str[NUM_FINGERS][4];  

////////////////////////////////////////////
// Initialization (Called on Arduino Boot)
////////////////////////////////////////////

void setup()
{
  //Initialize input pins
  for (int i=0; i < NUM_FINGERS; i++)
  {
    pinMode(FIRST_PIN - i, INPUT);
    
    // Setup the string we expect to receive when
    //  a given encoder is requested.
    req_str[i][0] = '0';
    req_str[i][1] = '0';
    req_str[i][2] = char('1'+i);
    req_str[i][3] = '\0';
  }
  
  //Initialize serial communication with computer
  Serial.begin(BAUD_RATE);    
  Serial.flush();
  counter=0;
}

void loop() 
{
  // Only do anything if something is on the serial port
  if (Serial.available() > 0)
  {
    // Read the next character
    buffer[counter] = char(Serial.read());
    counter = counter + 1;
    if(buffer[counter-1] == '#') //ending command
    {
      // Since we got an end of a command, process this command
      buffer[counter-1]='\0';
      int enc = 0;
      
      // Figure out which encoder has been requested
      for (int i=0; i < NUM_FINGERS; i++)
      {
        if (!strcmp(buffer, req_str[i]))
        {
          // Now read the pulse width for this encoder
          enc = pulseIn(FIRST_PIN - i, HIGH, PIN_TIMEOUT);
          break;
        }        
      }
      
      // If 'pulseIn' returned 0, it couldn't find a pulse, 
      //  so return NO_VALUE
      if (enc == 0)
        enc = NO_VALUE;

      // If we happen to be right at the boundary, make sure we
      //  don't return anything that's out of range
      if (enc > MAX_PULSE)
        enc = MAX_PULSE;
        
      // Load this new value onto the serial port
      Serial.print(enc);
      Serial.print('#');
      
      // Now flush the rest of our input
      Serial.flush();
      counter=0;
    }
  }
}
