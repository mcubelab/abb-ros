#include "serialPort.h"

SerialPort::SerialPort()
{
  connected = false;
}

SerialPort::SerialPort(const char * serialAddress)
{
  connected = false;
  openPort(serialAddress,9600); //Default baud rate
}

SerialPort::SerialPort(const char * serialAddress, const int baudRate)
{
  connected = false;
  openPort(serialAddress,baudRate);
}

SerialPort & SerialPort::operator = (const SerialPort &original)
{
  if(this != &original)
    {
      serialId = original.serialId;
      connected = original.connected;
      block = original.block; 
 
#ifdef WIN32
      GetCommState(original.serialId,&parameters);
#else
      tcgetattr(original.serialId,&parameters);
#endif
    }
  return *this;
}

int SerialPort::openPort(const char * serialAddress, const int baudRate)
{
  if(connected)
    closePort();
  //Open the serial port
#ifdef WIN32
  WCHAR    portAddress[256];
  MultiByteToWideChar(CP_UTF8,0, serialAddress, -1, portAddress, 256);
  serialId = CreateFile(portAddress,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			NULL);
  if(serialId != INVALID_HANDLE_VALUE)
    connected = true; 
  else
    connected = false;
#else
  serialId = open(serialAddress, O_RDWR | O_NOCTTY | O_NDELAY);

  if(serialId)
    connected = true;
  else
    connected = false;	

#endif
  
  if(connected)
    {
      //Set blocking status (Non blocking by default)
      setBlocking(false);
      
      //Capture serial port parameters
#ifdef WIN32
      GetCommState(serialId,&parameters);
#else
      tcgetattr(serialId,&parameters);
#endif
      
      //Set baud rate
      switch(baudRate)
	{
	case 110:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_110;
#else
	    cfsetispeed(&parameters,B110);
	    cfsetospeed(&parameters,B110);
#endif
	    break;
	  }
	case 300:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_300;
#else
	    cfsetispeed(&parameters,B300);
	    cfsetospeed(&parameters,B300);
#endif
	    break;
	  }
	case 600:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_600;
#else
	    cfsetispeed(&parameters,B600);
	    cfsetospeed(&parameters,B600);
#endif
	    break;
	  }
	case 1200:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_1200;
#else
	    cfsetispeed(&parameters,B1200);
	    cfsetospeed(&parameters,B1200);
#endif
	    break;
	  }
	case 2400:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_2400;
#else
	    cfsetispeed(&parameters,B2400);
	    cfsetospeed(&parameters,B2400);
#endif
	    break;
	  }
	case 4800:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_4800;
#else
	    cfsetispeed(&parameters,B4800);
	    cfsetospeed(&parameters,B4800);
#endif
	    break;
	  }
	case 9600:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_9600;
#else
	    cfsetispeed(&parameters,B9600);
	    cfsetospeed(&parameters,B9600);
#endif
	    break;
	  }
	case 19200:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_19200;
#else
	    cfsetispeed(&parameters,B19200);
	    cfsetospeed(&parameters,B19200);
#endif
	    break;
	  }
	case 38400:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_38400;
#else
	    cfsetispeed(&parameters,B38400);
	    cfsetospeed(&parameters,B38400);
#endif
	    break;
	  }
	case 57600:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_57600;
#else
	    cfsetispeed(&parameters,B57600);
	    cfsetospeed(&parameters,B57600);
#endif
	    break;
	  }
	case 115200:
	  {
#ifdef WIN32
	    parameters.BaudRate=CBR_115200;
#else
	    cfsetispeed(&parameters,B115200);
	    cfsetospeed(&parameters,B115200);
#endif
	    break;
	  }
	default:
	  {
	    printf("Warning: No standard Baud rate selected.\n");
	    printf("(110, 300, 600, 1200, 2400, 4800, 9600, 19200, 38400, 57600, or 115200).\n");
	    printf("Connceted at default 9600 bauds.\n");
#ifdef WIN32
	    parameters.BaudRate=CBR_9600;
#else
	    cfsetispeed(&parameters,B9600);
	    cfsetospeed(&parameters,B9600);
#endif
	    break;
	  }
	}
      //Rest of communication parameters
      //N81 options (8 bits, No parity check, 1 stop bit) 
#ifdef WIN32
      parameters.ByteSize=8;
      parameters.StopBits=ONESTOPBIT;
      parameters.Parity=NOPARITY;
      SetCommState(serialId, &parameters);
#else
      parameters.c_cflag |= (CLOCAL|CREAD);
      parameters.c_cflag &= ~PARENB;
      parameters.c_cflag &= ~CSTOPB;
      parameters.c_cflag &= ~CSIZE;
      parameters.c_cflag |=CS8;
      tcsetattr(serialId,TCSAFLUSH,&parameters);
#endif

    }
  return connected;
}

int SerialPort::flushPort()
{
  if(connected)
    {
      char buffer[256];
      int numBytes = 1;
      while(numBytes > 0)
	numBytes = readData(buffer,256);
      return 1;
    }
  return 0;
}

int SerialPort::closePort()
{
  if(connected)
    {
#ifdef WIN32
      CloseHandle(serialId);
#else
      close(serialId);
#endif
      connected=false;
    }
  return !connected;
}

bool SerialPort::isConnected()
{
  return connected;
}

bool SerialPort::setBlocking(const bool blocking)
{
  if(connected)
    {
      if(blocking)
	{
	  //#if WIN32
	  //COMMTIMEOUTS timeouts = {0};
	  //timeouts.ReadIntervalTimeout = 50;
	  //timeouts.ReadTotalTimeoutConstant = MAXDWORD;
	  //timeouts.ReadTotalTimeoutMultiplier = 0;
	  //timeouts.WriteTotalTimeoutConstant = MAXDWORD;
	  //timeouts.WriteTotalTimeoutMultiplier = 0;
	  //SetCommTimeouts(serialId,&timeouts);
	  //#else
	  if(fcntl(serialId, F_SETFL,0)!= -1)
	    {
	      block = true;
	      return true;
	    }
	  //#endif
	}
      else
	{
	  //#if WIN32
	  //COMMTIMEOUTS timeouts = {0};
	  //timeouts.ReadIntervalTimeout = MAXDWORD;
	  //timeouts.ReadTotalTimeoutConstant = 0;
	  //timeouts.ReadTotalTimeoutMultiplier = 0;
	  //timeouts.WriteTotalTimeoutConstant = 0;
	  //timeouts.WriteTotalTimeoutMultiplier = 0;
	  //SetCommTimeouts(serialId,&timeouts);
	  //#else
	  //fcntl(serialId, F_SETFL,FNDELAY);
	  int flags;
	  flags = fcntl(serialId,F_GETFL,0);
	  if(flags != -1)
	    {
	      if(fcntl(serialId, F_SETFL, flags | O_NONBLOCK) != -1)
		{
		  block = false;
		  return false;
		}
	    }
	}
    }
  return false;
}

int SerialPort::writeData(char const * buffer)
{
  int n;
  if(connected)
    {
#ifdef WIN32
      DWORD num;
      WriteFile(serialId, buffer, strlen(buffer),&num,NULL);
      n = (int)num;
#else
      n = write(serialId, buffer, strlen(buffer));
#endif
      /*
      struct timespec req = {0,0};
      req.tv_sec = 0;
      req.tv_nsec = (int)(1000000L);
      nanosleep(&req, (timespec *)NULL);
      */
      return n;
    }
  else
    return -1;
}

int SerialPort::readData(char * buffer, const int bufferLenght)
{
  int n;
  if(connected)
    {
#ifdef WIN32
      DWORD num;
      ReadFile(serialId, buffer, bufferLenght, &num, NULL);
      n = (int)num;
#else
      if (!block)
      {
        // If we are non-blocking, wait until the socket is ready before reading
        FD_ZERO(&fds);
        FD_SET(serialId, &fds);
        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000;
        int s = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);

        // If the socket is still not ready, don't read
        if (s > 0)
          n = read(serialId,buffer,bufferLenght);
        else
          n = s;
      }
      else
      {
        n = read(serialId,buffer,bufferLenght);
      }

#endif
      return n;
    }
  else
    return -1;
}

int SerialPort::readData(char * buffer, const int bufferLenght, const char* termStr, int timeout)
{
  timeval tim;
  gettimeofday(&tim, NULL);
  double iniT=tim.tv_sec+(tim.tv_usec/1000000.0);
  //clock_t iniTime = clock();
  char bufferAux[256];
  int nChar;
  int remChar = bufferLenght-1;
  bool end = false;
  strcpy (buffer,"");

  while(!end)
  {
    gettimeofday(&tim, NULL);
    double partialT=tim.tv_sec+(tim.tv_usec/1000000.0);

    if ((partialT - iniT) > (double)timeout/1000.0)
      return -1;

    nChar = readData(bufferAux,255);
    if(nChar>0)
    {
      bufferAux[nChar] = '\0';
      if (strstr(bufferAux, termStr) != NULL)
        end = true;
      remChar-=strlen(bufferAux);
      if(remChar>=0)
        strcat(buffer,bufferAux);		      
      else
      {
        strncat(buffer,bufferAux,strlen(bufferAux) + remChar);
        end = true;
      }
    }
#ifdef WIN32
    Sleep (1);
#endif
  }
  return 1;
}

SerialPort::~SerialPort(void)
{
}
