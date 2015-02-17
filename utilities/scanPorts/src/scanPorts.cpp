#include <sys/types.h>
#include <dirent.h>
#include "../include/serialPort.h"

#define MAX_BUFFER 256


int main(int argc, char *argv[])
{
  char buffer[MAX_BUFFER];
 
 bool ok=true;
  int option;
  char portName[MAX_BUFFER];
 
 
  int arduinoBaudRate = 115200;
  int motorControllerBaudRate = 38400;


  //Check arguments
  if(argc!=2)
    ok = false;
  else
    {
      if(!sscanf(argv[1],"%d",&option))
	ok = false;
      else
	{
	  switch(option)
	    {
	    case 1:
	      {
		printf("PORT_SCANNER: Scanning for Arduino...\n");
		break;
	      }
	    case 2:
	      {
		printf("PORT_SCANNER: Scanning ports for Motor Controller...\n");
		break;
	      }
	    default:
	      {
		ok = false;
		break;
	      }
	    }
	}
    }
  if(!ok)
    {
      printf("PORT_SCANNER: Wrong argument. Specify argument 1 for Arduino and 2 for motor Controller\n");
      return false;
    }


  //Scan ports
  struct dirent *de=NULL;
  DIR *d=NULL;
  
  d=opendir("/dev");
  if(d == NULL)
    {
      printf("PORT_SCANNER: Couldn't open device directory.\n");
      return false;
    }
  
  // Loop through all the devices
  printf("PORT_SCANNER: Looping through the usb serial ports ...\n");
  while(de = readdir(d))
    {
	//printf("%s\n",de->d_name);
      //The name must begin with tty
      if(strstr (de->d_name,"ttyUSB"))
      	{
	  sprintf(portName,"/dev/%s",de->d_name);
	  printf("Scanning %s\n",portName);
	  if(option==1)//Check for Arduino
	    {
		SerialPort spArduino = SerialPort(portName, arduinoBaudRate);
		sleep(2.0);
		spArduino.setBlocking(false);
	        sprintf(buffer,"001#");
		spArduino.flushPort();
		spArduino.writeData(buffer);
		if(spArduino.readData(buffer,MAX_BUFFER,'#',250)>0)
		  {
		    int aux;
		    if(sscanf(buffer,"%d",&aux))
			{
			  printf("PORT_SCANNER: Arduino found at port %s with baud rate %d.\n",portName, arduinoBaudRate);
			  return true;
			}
		    }
		
	        spArduino.closePort();
	    }
	  else if(option==2) //Check for motor controller
	    {
	        SerialPort spMotor = SerialPort(portName, motorControllerBaudRate);
		sleep(2.0);
		spMotor.setBlocking(false);
		
                sprintf(buffer,"\nANSW0\n");
      		spMotor.writeData(buffer);
		sprintf(buffer,"\nPOS\n");
		spMotor.flushPort();
		spMotor.writeData(buffer);
		if(spMotor.readData(buffer,MAX_BUFFER,'\n',200)>0)
		  {
		    int aux;
		    if(sscanf(buffer,"%d",&aux))
			{
			  printf("PORT_SCANNER: Motor Controller found at port %s with baud rate %d.\n",portName, motorControllerBaudRate);
			  return true;
			}
		    }		      
		
	        spMotor.closePort();
	    }
	}
    }
  closedir(d);
  if(option ==1)
    printf("PORT_SCANNER: Arduino not found\n");
 else
   printf("PORT_SCANNER: Motor Controller not found\n");
 
  return 0;
}
