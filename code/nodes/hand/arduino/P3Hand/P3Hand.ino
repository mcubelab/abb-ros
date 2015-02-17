// P3HandRev1Test.ino : hardware test for the Rev 1 of the P3 Hand Controller Board 
// Copyright (c) 2012 Garth Zeglin

// ##BEGIN_LICENSE##
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
// 
// This is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this file; if not, write to the Free Software Foundation,
// Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
// ##END_LICENSE##
/****************************************************************/
/****************************************************************/
// The board uses an Arduino Nano v3.1, which has an ATmega328.

// This program assumes it is compiled with the Arduino IDE version 1.0 or
// later.  Earlier version do not have interrupt-driven serial output.

/****************************************************************/
// Import the SPI library, which is a very simple wrapper around the
// SPI registers.  Transfers block until complete (e.g. 8uSec @ 1MHz).
#include <SPI.h>

// Import the time measurement code for higher precision regulation of the run loop.
#include "ElapsedTimer.h"


/****************************************************************/
// Pin assignments.
// 			     D0/RX	RXD		serial data from USB	
// 			     D1/TX	TXD		serial data to USB	
const int modePin = 2;    // D2		PD2		motor driver control output	D2/MODE
const int phasePin = 3;   // D3		OC2B or PD3	motor driver PWM output		D3/PHASE
const int brakePin = 4;   // D4		PD4		motor driver control output	D4/!BRAKE!
const int enablePin = 5;  // D5		PD5 or OC0B	motor driver control or output	D5/!ENABLE!
const int quadCSPin = 6;  // D6		PD6		quadrature counter chip select	D6/!QUADSS!
const int adcCSPin = 7;   // D7		PD7		ADC chip select			D7/!ADCCS!
const int encInPin = 8;	  // D8		ICP1		Timer 1 Input Capture		D8/ICP1
//                           D9			        not used
const int dacCSPin = 10;  // D10	PB2		DAC chip select			D10/!DACCS!
const int spiDOPin = 11;  // D11/MOSI	MOSI		SPI data out			D11/MOSI
const int spiDIPin = 12;  // D12/MISO	MISO		SPI data in			D12/MISO
const int spiCLKPin = 13; // D13/SCK	SCK		SPI clock			D13/SCK

// analog pins
const int vmSenseChannel = 0;  // A0	ADC0	motor supply sensing	A0/VMSENSE
// A1 through A3, A6 and A7 are unused

// A4 and A5 are used as digital pins (not supported by Arduino library)
// A4	PC4	encoder multiplexer control output	A4/ENCSEL0
// A5	PC5	encoder multiplexer control output	A5/ENCSEL1
#define ENCSEL0_MASK 0x10
#define ENCSEL1_MASK 0x20
#define ENCSEL_SHIFT 4

/****************************************************************/
// Timer 1 support for finger encoder PWM signal measurement.

// Define constants for TCCR1B for the the two capture modes in use.

// The prescale is currently set to 8 so the counter resolution is 0.5 usec.  It
// would be better to use a prescale of 1, but with a 16MHz system clock, the
// 16-bit counter rolls over in 4.1 msec, slightly faster than at least one of
// the encoders.  The faster count rate should still be possible but will
// require making some assumptions about valid period lengths and careful
// analysis of the valid pulse widths.

const byte TCCR1B_value = 
  ( 0x00 |  // ICNC1 = 0; this might be 0x80 later to activate the ICP noice canceler.
    0x00 |  // ICES1 = 0 start with a falling edge sensitivity (this will be constantly changed).
    0x00 |  // WGM1[3:2] = 0. Timer is in normal mode 0, WGM1 = 0.
    0x02 ); // CS1[2:0] = 2 for a prescale of 8.

const byte TCCR1B_ICES = 0x40;  // ICES1 = 1 for rising edge sensitivity
const byte TIFR1_ICF = 0x20;    // mask for the input capture flag in the interrupt flag register


// Largest value we can set current limit to for motor
const int MAX_CURRENT_LIMIT = 4095;


void initFingerEncoderIO(void)
{
  // Configure encoder selector pins for digital output; these are not
  // normally digital I/O in the Arduino system.
  DDRC |= (ENCSEL0_MASK | ENCSEL1_MASK); 

  // Configure Timer1 for event capture.  It free-runs at a fast rate to
  // timestamp edges observed on the input capture pin ICP1 from the finger
  // encoders.  This will likely be upgraded in the future to also generate
  // interrupts, but for now, it is assumed that the capture registers can
  // simply be polled (with some limits on minimum measurable pulse width).

  TCCR1A = ( 0x00 |  // no outputs on OC1A or OC1B
	     0x00 ); // WGM1[1:0] = 0.  Timer is in normal mode 0, WGM1 = 0
  
  TCCR1B = TCCR1B_value;  // see above

  TIMSK1 = 0; // don't generate any interrupts
  
}
inline void selectFingerEncoder(int channel)
{
  // update the portC output bits controlling the encoder selector
  PORTC = (PORTC & ~(ENCSEL0_MASK | ENCSEL1_MASK)) | ((channel & 3) << ENCSEL_SHIFT );
}
inline void clearTimer1CaptureFlag(void) 
{
  TIFR1 = TIFR1_ICF;  // clear the capture flag by writing a 1 to the bit location
}
inline int timer1EdgeCaptured(void) 
{
  return (TIFR1 & TIFR1_ICF);  // return true after an event detected
}
inline void setTimer1EdgeSensitivity( int rising ) 
{
  TCCR1B = (rising) ? TCCR1B_value : (TCCR1B_value | TCCR1B_ICES );
}
/****************************************************************/
// State of an input capture state machine.
struct finger_encoder_t {
  unsigned int last_rising;
  unsigned int last_falling;
  unsigned int period;
  unsigned int width;
  int state;
  long timestamp;   // system millis clock timestamp for last full reading
  int channel;      // identifies encoder being read
};

// Latest count and timestamp of the encoders
int encs[3];
long Etimes[3];

// Latest value and timestamp of the force sensors
int forces[3];
long Ftimes[3];

// Updates record of force sensors
void pollForceSensors()
{
  for (int i=0; i<3; i++){
    forces[i] = readADC(i);
    Ftimes[i] = millis();
  }  
}

// State machine for encoder
void pollFingerEncoderCapture( struct finger_encoder_t *f )
{
  switch( f->state ) {

    // prepare to capture first rising edge
  case 0:
    setTimer1EdgeSensitivity( 1 );
    clearTimer1CaptureFlag();
    f->state++;
    break;

    // wait for rising edge to be captured
  case 1:
    if (timer1EdgeCaptured()) {
      f->last_rising = ICR1;          // read timestamp
      setTimer1EdgeSensitivity( 0 );  // configure for falling edge
      clearTimer1CaptureFlag();
      f->state++;
    }
    break;

    // wait for falling edge to be captured
  case 2:
    if (timer1EdgeCaptured()) {
      f->last_falling = ICR1;      // read timestamp
      Etimes[f->channel] = millis();
      setTimer1EdgeSensitivity( 1 );  // configure for rising edge
      clearTimer1CaptureFlag();
      f->state++;
    }
    break;

    // wait for second rising edge to be captured
  case 3:
    if (timer1EdgeCaptured()) {
      unsigned int rising = ICR1;     // read timestamp
      
      // a full cycle has been captured, so compute the full timing
      f->width = f->last_falling - f->last_rising;
      f->period = rising - f->last_rising;
      
      // an edge may sometimes be missed, resulting in longer period and width measurements by multiples of the actual period (usually only one)
      // three cases: no overrun, only overrun period, both width and period overrun
      f->period /= (f->period/8000);                    // divides the period by however many times it has overrun (period is around 8000)
      f->width -= (f->width/f->period)*f->period;       // subtracts multiples of the period from the width, if needed
      
      // Changes range of encoders to [0, 4095]
      encs[f->channel] =  ((float) f->width / f->period * 4095);

      // move on to next encoder
      f->channel = (f->channel+1)%3;
      selectFingerEncoder( f->channel );
      setTimer1EdgeSensitivity( 1 );
      clearTimer1CaptureFlag();
      f->state=1;

    }
    break;
  }
}

/****************************************************************/
// Sample and read one value from the MCP3204 12-bit 4-channel ADC.
// The DIN data is sampled on the rising edge of SCLK.
// The DOUT data changes on the falling edge of SCLK.
// The clock is assumed to idle low.
// This is SPI MODE0: CPOL = 0, CPHA = 0. 
int readADC(int channel)
{
  // See Figure 6-1 in the MPC3204 data sheet for the timing of the following transfers.
  byte command1  = ( 0x04 | 0x02 );     // Leading zeros, start bit, and single-ended mode.
  byte command2  = (channel & 3) << 6;  // Channel select bits D1 and D0.

  // Every SPI transfer is both a send and a receive.  The first value received
  // is ignored.  
  digitalWrite( adcCSPin, LOW );
  SPI.transfer( command1 );            // ignore don't care bits clocked in
  byte b1 = SPI.transfer( command2 );  // four garbage bits, four conversion bits clocked in
  byte b2 = SPI.transfer( 0 );         // remaining 8 bits conversion bits   
  digitalWrite( adcCSPin, HIGH );

  return (((b1 & 0x0f) << 8) | (b2));
}

/****************************************************************/
// Update the voltage output from the MCP4921 12-bit single-channel DAC.
void writeDAC(int value)
{
  byte command1 = ( 0x40 |  // BUF = 1 for buffered VREF input
		    0x20 |  // !GA = 1 for unity VREF gain
		    0x10 |  // !SHDN = 1 for active output
		    ((value >> 8) & 0x0f) ); // four most significant bits of value
  byte command2 = (value & 0xff);  // bottom eight bits of value
  
  digitalWrite( dacCSPin, LOW );
  SPI.transfer( command1 );
  SPI.transfer( command2 );
  digitalWrite( dacCSPin, HIGH);
}

/****************************************************************/
// Initialize the LS7366R quadrature counter.
// The general instruction format to the chip:
// operation codes:
enum { 
  QUAD_CLR = 0, QUAD_RD, QUAD_WR, QUAD_LOAD };
// register select values:
enum { 
  QUAD_NONE = 0,  QUAD_MDR0, QUAD_MDR1, QUAD_DTR, QUAD_CNTR, QUAD_OTR, QUAD_STR };
// these are combined:
#define QUAD_COMMAND(op, reg) (((op) << 6) | ((reg)<<3)) 

// e.g.
// WRITE_MDR0 is QUAD_COMMAND( QUAD_WR, QUAD_MDR0 ) == ( 2 << 6) | (1 << 3) == 0x88
// WRITE_MDR1 is QUAD_COMMAND( QUAD_WR, QUAD_MDR1 ) == ( 2 << 6) | (2 << 3) == 0x90
// READ_CNTR  is QUAD_COMMAND( QUAD_RD, QUAD_CNTR ) == ( 1 << 6) | (4 << 3) == 0x60
void initQuadrature(void)
{
  byte mdr0 = ( 0x03 | // B1, B0 set quadrature mode, four counts per cycle
  0 |    // B3, B2 are zero for free-running count
  0 |    // B5, B4 are zero to disable the index input
  0 |    // B6 is zero for asynchronous index
  0 );   // B7 is zero for a filter clock divisor of 1

  byte mdr1 = ( 0x02 | // B1, B1 set 2-byte counter mode (up to four bytes available)
  0x00 | // B2 is zero to enable counting, B3 is don't-care
  0x00 );// B7-B4 are zero to disable all external flag indications

  digitalWrite( quadCSPin, LOW );
  SPI.transfer( 0x88 ); // WRITE_MDR0 command
  SPI.transfer( mdr0 );
  digitalWrite( quadCSPin, HIGH );

  digitalWrite( quadCSPin, LOW );
  SPI.transfer( 0x90 ); // WRITE_MDR1 command
  SPI.transfer( mdr1 );
  digitalWrite( quadCSPin, HIGH );
}

/****************************************************************/
// Read back status values from the LS7366R quadrature counter.
byte readQuadratureByteRegister( int reg )
{
  digitalWrite( quadCSPin, LOW );
  SPI.transfer( QUAD_COMMAND( QUAD_RD, reg ) );
  byte value = SPI.transfer( 0 );
  digitalWrite( quadCSPin, HIGH );
  return value;
}

long readQuadratureStatus(void)
{
  byte mdr0 = readQuadratureByteRegister( QUAD_MDR0 );
  byte mdr1 = readQuadratureByteRegister( QUAD_MDR1 );
  byte str  = readQuadratureByteRegister( QUAD_STR );

  return (((long) mdr0) << 16) | (((long) mdr1) << 8) | (((long) str) << 0);
}

/****************************************************************/
// Read the quadrature counter with the relative motor angle.  N.B. this just
// reads 16 bits, if more resolution is required, the counter can operate with
// up to 32 bits, see the init function.
unsigned readQuadrature(void)
{
  digitalWrite( quadCSPin, LOW );
  SPI.transfer( 0x60); // READ_CNTR command
  byte b1 = SPI.transfer( 0 );
  byte b2 = SPI.transfer( 0 );
  digitalWrite( quadCSPin, HIGH );
  return ((((unsigned)b1)<<8) | ((unsigned)b2));
}


/***************************************************************/
// Resets quadrature count to 0. Useful for homing motor
void clearQuadrature(void)
{
  digitalWrite( quadCSPin, LOW );
  SPI.transfer( QUAD_COMMAND( QUAD_CLR, QUAD_CNTR ) );
  digitalWrite( quadCSPin, HIGH );
}


/****************************************************************/
// Return the motor supply voltage in millivolts.
int readVMOTOR(void)
{
  // VMOTOR is connected to an on-chip ADC channel via a 1:2 voltage divider.
  // The ADC is ten bits with a maximum range of 5V, so values from 0-1023 are
  // mapped to 0-10V.
  int raw = analogRead( vmSenseChannel );
  return map( raw, 0, 1024, 0, 10000 );
}

/****************************************************************/
// Configure TIMER2 to generate a motor PWM signal on OC2B for PHASE, suitable
// for locked anti-phase servo control.  This uses phase-correct PWM at 31kHz.

void initMotorPWM(void)
{

  TCCR2A = ( 0x00 |  // OC2A a normal digital I/O
  0x20 |  // OC2B mode 2 B: clear on up-count, set on down-count
  0x01 ); // WGM2[1:0] = 1 (WGM2 = 0b001) for full-count phase-correct PWM (Table 18-8)
  TCCR2B = ( 0x00 |  // no force output bits
  0x00 |  // WGM2[2] = 0
  0x01 ); // CS2[2:0] = 1 for prescale = 1 (Table 18-9)

  OCR2B = 128;  // set to a midrange value
}

/****************************************************************/
// Set eight-bit motor PWM drive value.  Values from -128 to 127 correspond to
// full reverse to full forward.  Zero is freewheeling (or could be braked).

void setMotorPWM( int speed )
{
  if (speed == 0) {
    // enable and mode both high sets sleep mode as long if braking is disabled
    digitalWrite( enablePin, HIGH );
    digitalWrite( modePin, HIGH );
    digitalWrite( brakePin, HIGH );
    OCR2B = 128;

  } 
  else {

    // else normally choose either forward or reverse fast current-decay mode
    digitalWrite( modePin, HIGH );  // high for fast current-decay mode
    digitalWrite( brakePin, HIGH ); // brake off
    // digitalWrite( phasePin, ( speed > 0 ) ? HIGH : LOW ); // high for forward, low for reverse

    speed = constrain( speed, -128, 127 );
    OCR2B = (speed + 128) & 0xff;

    digitalWrite( enablePin, LOW ); // driver enabled
  }
}

/****************************************************************/
/****************************************************************/
// Global variables for the test code.

// Declare a global timer to use in regulating the run loop.
ElapsedTimer loopTimer;

// Basic servo test state.
struct servo_t {
  int q;    // current position
  int q2;   // previous position
  int q_d;  // target position
  int u;    // output command
} 
c;

// State machine status for finger encoder sampling.
struct finger_encoder_t fingers;

// Global variables with initial values, controllable from the command line.
int floating = 0;
int mode = 0;
float current = 0.5;
float spd = 1.0;
float KP = 9.0, KI = 6.0, KD = 3.0;

// PID variables
float ITerm = 0;
int lastTime = 0;

// Very simple way to print <number> to <width> columns
// If <number> exceeds <width>, only the last <width> digits are printed
void printw(int number, int width){
  if (width>1) printw(number/10, width-1);
  Serial.print(number%10);
}

int print_count = 0;


/****************************************************************/

void setup(void) 
{

  // set up hardware
  pinMode( modePin, OUTPUT );
  pinMode( phasePin, OUTPUT );   // will be controlled by TIMER2 as OC2B later
  pinMode( brakePin, OUTPUT );   // active-low
  pinMode( enablePin, OUTPUT );  // active-low
  pinMode( quadCSPin, OUTPUT );  // active-low
  pinMode( adcCSPin, OUTPUT );   // active-low
  pinMode( dacCSPin, OUTPUT );   // active-low

  pinMode( encInPin, INPUT );    // will be used as Timer 1 input capture ICP1 later

  initFingerEncoderIO();         // the encoder selector uses Port C, which isn't supported by the Arduino library for digital I/O

    // set up some safe initial states
  digitalWrite( modePin, HIGH );
  digitalWrite( brakePin, HIGH );   // active-low
  digitalWrite( enablePin, HIGH );  // active-low

  digitalWrite( quadCSPin, HIGH );  // active-low
  digitalWrite( adcCSPin, HIGH );   // active-low
  digitalWrite( dacCSPin, HIGH );   // active-low

  // Configure the SPI port.  The SPI clock is set to 1MHz. (The ADC is rated at
  // a maximum of 2MHz, the DAC a maximum of 20MHz.)
  SPI.begin(); 
  SPI.setClockDivider( SPI_CLOCK_DIV16 );  
  SPI.setBitOrder( MSBFIRST );
  SPI.setDataMode( SPI_MODE0 );

  // The SPI pins D11, D12, and D13 are controlled by the SPI library.  N.B. The
  // SPI library also sets SS to an output, which is D10, currently dacCSPin.
  // This is harmless, we have already set it to output.  The library doesn't
  // appear to manipulate it otherwise.

  Serial.begin(115200);
  const char version_string[] = "P3HandRev1Test version " __DATE__ " " __TIME__;
  Serial.println( version_string );
  Serial.flush();

  // Finish initializing the peripherals.
  initQuadrature();
  initMotorPWM();
  
  // Make sure the motor encoder count is 0, and our target reflects that
  clearQuadrature();
  c.q_d = 0;

  // Start timing at zero.
  loopTimer.reset();
  
  writeDAC((int)(current * MAX_CURRENT_LIMIT));
  fingers.channel = 0;

  // Send start character, so we know we're ready
  Serial.println("!");
}

/****************************************************************/
// Polling loop with some test modes.

void loop( void )
{  
  // check for user commands
  poll_command();
  // check for encoder edge 
  // this will be called throughout loop to perhaps better catch encoder edges
  pollFingerEncoderCapture( &fingers );
  
  long usec = loopTimer.elapsedMicroseconds();

  // update once per millisecond
  if ( usec > 1000 && !floating) {
    
    // If time has overrun a lot, just reset the timer, otherwise subtract a constant offset.  This will
    // tend to average out jitter and ensure a constant average rate.
    if ( usec > 2000 ) loopTimer.reset();
    else loopTimer.subtractMicroseconds( 1000 );
    
    pollFingerEncoderCapture( &fingers );
      
    // For velocity estimator
    // Time passed between updates
    float timePassed = usec - lastTime + 1000;
    lastTime = usec;  
           
    // Treat the 16 bits of position count as a signed number from -32768 to 32767. 
    c.q2 = c.q;
    c.q = (int) readQuadrature();
    
    // Note that it's possible to have wrap around, so do long integer 
    //  subtraction to make sure we get the direction right.
    long err = (long)c.q_d - (long)c.q;    
    
    pollFingerEncoderCapture( &fingers );

    // No output if target reached
    if (err != 0){
      // Restricts and only uses integral term within 700 motor angles (half a revolution)
      if (err < 700){
        ITerm += (KI*err)/timePassed;
        ITerm = constrain (ITerm, -128, 127);
      }
      float veloc = (c.q-c.q2)*1000/timePassed;
      c.u = constrain( KP*err + ITerm - KD*veloc, -128*spd, 127*spd);   
    } else {
      c.u = 0;
      ITerm = 0;
    }

    setMotorPWM( c.u );
  }
  
  pollFingerEncoderCapture( &fingers );
  pollForceSensors();
   
//   MODES
//     1    Continuously prints motor position
//     2    Prints encoder count whenever captured (Goes in order 0, 1, 2)
//     3    Continuoulsy prints force sensors
//     4    Prints all hand data (motor, encoders, and force sensors) at the set interval
//    ALL   Motor movement whenever target != position
   
  switch(mode){
  default: break;
    
  // Streams motor position
  case 1:
    Serial.print("MTR       "); Serial.println( (int)readQuadrature() );
  break;
  
  // Streams encoders
  case 2:
  {
    static int curFinger = 0;
    if ( curFinger != fingers.channel ) {
      Serial.print("ENC "); Serial.print(curFinger); Serial.print("    "); Serial.print(encs[curFinger]); Serial.print("   @   "); Serial.println(Etimes[curFinger]);
      curFinger = (curFinger+1)%3;
    }
//    static long timing2 = 0;
//    if (millis() - timing2 > 1000){
//      timing2 = millis();
//      Serial.println("\n ---------------------------------------------");
//    }
  }  
  break;
  
  // Streams force sensors
  case 3:
  {
    for (int i=0; i<3; i++){
      Serial.print("FRC "); Serial.print(i); Serial.print("     "); Serial.print(forces[i]); Serial.print("  @  "); Serial.println(Ftimes[i]);  
    }
  }
  break;
  
  // Prints all hand data at the set time interval 
  case 4:
    static long timing = 0;
    if (millis() - timing > 30){    // Change value after the '>' sign to set interval (in milliseconds);
      timing = millis();
      Serial.print("#M"); Serial.print( (int)readQuadrature() ); Serial.println("&");
      Serial.print("#E"); printw(encs[0],4); printw(encs[1],4); printw(encs[2],4); Serial.println("&");
      Serial.print("#F"); printw(forces[0],4); printw(forces[1],4); printw(forces[2],4); Serial.println("&");
    }
//    static long timing2 = 0;
//    if (millis() - timing2 > 1000){
//      timing2 = millis();
//      Serial.println("\n ---------------------------------------------");
//    }
    break;
  
  }
  
}
/****************************************************************/
