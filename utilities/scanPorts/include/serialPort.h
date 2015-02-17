#if !defined(SERIALPORT_INCLUDED)
#define SERIALPORT_INCLUDED

#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#endif

#include <stdio.h>
#include <string.h>
#include <time.h>

class SerialPort
{
private:
  #ifdef WIN32
    HANDLE serialId;            //Serial Port Id
    DCB parameters;             //Serial Port parameters
  #else
    int serialId;               //Serial Port Id
    struct termios parameters;     //Serial Port parameters
  #endif
  bool connected;       //Connection status
  bool block;        //Blocking state when reading 

public:
  //Constructors
  SerialPort();
  SerialPort(const char * serialAddress);
  SerialPort(const char * serialAddress, const int baudRate);
  SerialPort & operator = (const SerialPort &original);	//assignment
	
  //Destructor
  ~SerialPort();
  
  //Configuration
  int openPort(const char * serialAddress, const int baudRate);
  int closePort();
  int flushPort();
  bool isConnected();
  bool setBlocking(bool blocking);
 
  //Communication
  int writeData(char const *buffer);
  int readData(char *buffer, const int bufferLenght);
  int readData(char *buffer, const int bufferLenght, const char termChar, int timeout=100000);
};

#endif	// !defined(SERIALPORT_INCLUDED)
