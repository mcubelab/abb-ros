// CommandLine.pde : simple command processor for interactive debugging and control
// Copyright (c) 2011-2012 Garth Zeglin

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

// Process a command string which has been parsed into tokens.  This is not a
// very efficient interpreter, but simple to code and maintain.  This is called
// from the main event loop, so it may modify global state and trigger events
// but should not stall if it is to be called during normal operation.



//  Available Commands
// --------------------
// Function                      Command                     Notes 
// -----------                   --------------              -------------
// Change hand mode              mode <value>#                modes are within hand code
// Stop motor and hold           stop#                       sets current limit to 0, pauses, sets current position as target, and resets current limit
// Stop motor and float          float#                      motor will freely rotate

// Set home position             home#                       Sets quadrature counter to 0 for motor

// Open hand                     open#                       powers the hand until it stops moving for 10ms, releases, and holds position
// Close hand                    close#                      powers the hand until it stops moving for 10ms, releases, and holds position
//                                                           THESE COMMANDS CAN NOT BE INTERRUPTED

// Open hand a set angle         open <value>#               sets target position <value> motor angles towards the open direction
// Close hand a set angle        close <value#               sets target position <value> motor angles towards the close direction

// Get hand data                 stat#                       returns all hand data
// Get motor angle               motor#                      returns current motor angle
// Get encoder values            enc#                        returns latest encoder counts with timestamps
// Get encoder value             enc <value>#                returns latest encoder count with timestamp for encoder <value> (0, 1, 2)
// Get force values              force#                      returns latest sensor readings with timestamps
// Get force value               force <value>#              returns latest sensor reading with timesstamp for sensor <value> (0, 1, 2)

// Set motor target              tar <value>#                <value> between 0 and 65535   (unisigned)
// Set motor speed               speed <value>#              <value> between 0 and 100     (will be contrained)
// Set motor current limit       current <value>#            <value> between 0 and 4095    (will be contrained)
// Set PID tunings               tune <P> <I> <D>#           
// Set PID tuning                tune <letter> <value>#      sets the <letter> (p, i, d) gain to <value>
// Autoset PID tunings           autotune#                   automatically adjusts tunings to (somewhat) acceptable values


void execute_command( int argc, char **argv )
{
    if (argc > 0) {
      if (!strcmp(argv[0], "stop")){
        STOP();
      } 
      
      else if (!strcmp(argv[0], "enc")){
        if (argc < 2){
          for (int i=0; i<3; i++){
            Serial.print("ENC "); Serial.print(i); Serial.print("     "); Serial.print(encs[i]); Serial.print("  @  "); Serial.println(Etimes[i]);  
          }
        } else {
          int num = atoi(argv[1]);
          if (num>=0 && num <=2){
            Serial.print("ENC "); Serial.print(num); Serial.print("     "); Serial.print(encs[num]); Serial.print("  @  "); Serial.println(Etimes[num]);
          }
        }
      }
      
      else if (!strcmp(argv[0], "motor")){
        Serial.print("motor angle: "); Serial.println( readQuadrature() );
      }
      
      else if (!strcmp(argv[0], "force")){
        if (argc < 2){
          for (int i=0; i<3; i++){
            Serial.print("FRC "); Serial.print(i); Serial.print("     "); Serial.print(forces[i]); Serial.print("  @  "); Serial.println(Ftimes[i]);  
          }
        } else {
          int num = atoi(argv[1]);
          if (num>=0 && num <=2){
            Serial.print("FRC "); Serial.print(num); Serial.print("     "); Serial.print(forces[num]); Serial.print("  @  "); Serial.println(Ftimes[num]);
          }
        }
      }      
      
      else if (!strcmp(argv[0], "float")){
        floating = 1;
        setMotorPWM( 0 );
      }
      
      else if (!strcmp(argv[0], "open")){
        if (argc == 2)
          c.q_d = c.q - atoi(argv[1]);
        else {
          floating = 1;
          int lastPosition = readQuadrature()+1;
          while ( lastPosition != readQuadrature() ){
            setMotorPWM(-128);
            lastPosition = readQuadrature();
            delay(10);
          }
          setMotorPWM(0);
          STOP();
        }
      }
      
      else if (!strcmp(argv[0], "close")){
        if (argc == 2)
          c.q_d = c.q + atoi(argv[1]);
        else {
          floating = 1;
          int lastPosition = readQuadrature()-1;
          while ( lastPosition != readQuadrature() ){
            setMotorPWM(127);
            lastPosition = readQuadrature();
            delay(10);
          }
          setMotorPWM(0);
          STOP();
        }
      }
      
      else if (!strcmp(argv[0], "tar")){
        if (argc < 2) Serial.print("current target is : ");
        else {
          floating = 0;
          c.q_d = atoi(argv[1]);
	  Serial.print("new position target : ");
        }
        Serial.println( c.q_d );
      }
      
      
      else if (!strcmp(argv[0], "stat")) {
        Serial.print("\n       uptime = "); Serial.print( millis() ); Serial.println(" milliseconds.");
        Serial.print("    test mode = "); Serial.println( mode );
        Serial.print(" motor supply = "); Serial.print( readVMOTOR() ); Serial.println(" millivolts.");\
        Serial.print("motor current = "); Serial.println( readADC( 3 ) );
        
        Serial.print("current limit = "); Serial.println( current );
        Serial.print("        speed = "); Serial.println( spd );
        Serial.print("      tunings = "); Serial.print( KP ); Serial.print("  "); Serial.print( KI ); Serial.print("  "); Serial.println( KD );
        
        Serial.print("    encoder 0 = "); Serial.println(encs[0]);
        Serial.print("    encoder 1 = "); Serial.println(encs[1]);
        Serial.print("    encoder 2 = "); Serial.println(encs[2]);
        Serial.print("   f sensor 0 = "); Serial.println(forces[0]);
        Serial.print("   f sensor 1 = "); Serial.println(forces[1]);
        Serial.print("   f sensor 2 = "); Serial.println(forces[2]);
        Serial.print("  motor angle = "); Serial.println( readQuadrature() );
        Serial.print(" motor target = "); Serial.println( c.q_d );
        Serial.print("       output = "); Serial.println(c.u);
        Serial.println();
      }  
      
      else if (!strcmp(argv[0], "mode" )) {
        if (argc < 2) {
          Serial.print("current mode is: "); Serial.println( mode );
        } else {
	  mode = atoi(argv[1]);
	  Serial.print("set mode to : "); Serial.print( mode ); Serial.println("$");
        }
      }

      else if (!strcmp(argv[0], "current")){
        if (argc < 2) {
          Serial.print("current limit is: "); Serial.println( current );
        } else {
          current = constrain(atoi(argv[1]) / 100.0 , 0.0, 1.0);
	  writeDAC( (int)(current *MAX_CURRENT_LIMIT) );
          //Serial.print("new current : "); Serial.println( current );
        }
      }
      
      else if (!strcmp(argv[0], "speed")){
        if (argc < 2) {
          Serial.print("current speed is: "); Serial.println( spd );
        } else {
          spd = constrain(atoi(argv[1]) / 100.0 , 0.0, 1.0);
          //Serial.print("new speed : "); Serial.println( spd );
        }
      }
       
      else if (!strcmp(argv[0], "tune")){
        if (argc == 3) {
          if (!strcmp(argv[1], "p"))      KP = atof(argv[2]);
          else if (!strcmp(argv[1], "i")) KI = atof(argv[2]);
          else if (!strcmp(argv[1], "d")) KD = atof(argv[2]);
        } else if (argc == 4) {
          KP = atof(argv[1]); KI = atof(argv[2]); KD = atof(argv[3]); 
          }
        Serial.print("tunings at: "); Serial.print( KP ); Serial.print("  "); Serial.print( KI ); Serial.print("  "); Serial.println( KD );
      }
      
      else if (!strcmp(argv[0], "autotune")){
        float prop = current / 1000.0;
        KP = 3.0 * prop;
        KI = 2.0 * prop;
        KD = 1.0 * prop;
        Serial.print("tunings at: "); Serial.print( KP ); Serial.print("  "); Serial.print( KI ); Serial.print("  "); Serial.println( KD );
      }
      
      else if (!strcmp(argv[0], "home")){
        clearQuadrature();
        c.q_d = 0;
      }
    }
}

void STOP(){
  writeDAC( 0 );
  delay(350);                    // delay to allow the motor to settle down to a safe position
  c.q_d = readQuadrature();
  writeDAC( (int)(current * MAX_CURRENT_LIMIT) );
  floating = 0;
  Serial.println("STOPPED");
}

/****************************************************************/
int parse_arg_line(char *string, char **argv, int argmax)
{
  int argc = 0;
  char *cp = string;
  while (*cp && argc < argmax) {
    while (*cp && isspace(*cp)) cp++;     /* scan past white space */
    
    if (*cp) argv[argc++] = cp;           /* save pointer to text */  
    else break;
    
    while (*cp && !isspace(*cp)) cp++;    /* scan past text */
    if (*cp) *cp++ = 0;
  }
  return argc;
}

/****************************************************************/
// Define a simple global buffer for accumulating a command string.
const int max_command_length = 80;
const int max_command_terms  =  5;
static char command_buffer[max_command_length+1];
static int next_command_char = 0;

// Polling function to be called from the main event loop.
void poll_command(void)
{
  while (Serial.available()) {
    int next_char = Serial.read();

    // once a line terminator is received, process the available command string
    if (next_char == '#') {
      command_buffer[next_command_char] = 0; // terminate the string

      // parse the string into pieces and execute them
      char *argv[max_command_terms];
      int argc = parse_arg_line( command_buffer, argv, max_command_terms);
      execute_command(argc, argv);

      // then reset the command buffer
      next_command_char = 0;
    }

    // else store the characters until a full line is received, ignoring overrun
    else if (next_command_char < max_command_length) {
      command_buffer[next_command_char++] = next_char;
    }
  }
}  
