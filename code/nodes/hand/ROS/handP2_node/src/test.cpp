#include <serialPort/serialPort.h>
#include "P2Interpreter.h"

int main()
{
  P2Interpreter hand;
  hand.connectArduino("/dev/tty.usbserial-A600aih2",115200);
  hand.connectMotor("/dev/tty.usbserial",38400);
  
  if (hand.connectedMotor && hand.connectedArduino)
    {
      hand.setMotorSpeed(100);
      hand.setMotorIntensity(600);
      int encInitial = hand.encMotor;
       for (int speed = 50; speed<=250; speed+=50)
	{
	  hand.setMotorSpeed(speed);
	  hand.setMotor(encInitial - 7500);
	  //nanosleep(&req, (timespec *)NULL);
	  hand.waitRest();
	  hand.setMotorSpeed(speed+25);
	  hand.setMotor(encInitial);
	  //nanosleep(&req, (timespec *)NULL);
	  hand.waitRest();
	  }
     
      for (int i = 0; i<100; i++)
	{
	  printf("%d --", hand.encMotor);
    for (int j=0; j<NUM_FINGERS; j++)
      printf(" %d", hand.encFingers[j]);
    printf("\n");

	  struct timespec req = {0};
	  req.tv_sec = 0.0;
	  req.tv_nsec = 50000000.0;
	  nanosleep(&req, (timespec *)NULL);
	}
      hand.disconnectMotor();
      hand.disconnectArduino();
    }
  else
    printf("Connection problem.\n");
  return 0;
}
