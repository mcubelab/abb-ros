#include <time.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include "../include/serialPort.h"

#define MAX_BUFFER 256
#define SAMPLING_TIME 0.025 //Sampling the hand at 40Hz
#define NSAMPLES 20
#define MAX_MOTOR_POS 30000 
#define MIN_MOTOR_POS -30000

//Logger threads
void* motorLoggerMain(void *args);
void* fingersLoggerMain(void *args);

class P2Interpreter
{
 private:
  pthread_t motorLoggerThread;
  pthread_t fingersLoggerThread;
  pthread_attr_t motorLoggerThreadAttr;  
  pthread_attr_t fingersLoggerThreadAttr;  

 public:
  SerialPort spMotor;
  SerialPort spArduino;

  bool connectedMotor;
  bool connectedArduino;
  int motorSpeed;
  int motorIntensity;
  int encMotor;
  int encFingers[4];

  //Constructors
  P2Interpreter();

  //Destructor
  ~P2Interpreter();

  //Connection
  bool connectArduino(const char * serialAddressArduino, const int baudRate);
  bool disconnectArduino();
  bool connectMotor(const char * serialAddressMotor, const int baudRate);
  bool disconnectMotor();

  //Configuration
  bool setMotorSpeed(const int mSpeed);
  bool setMotorIntensity(const int mIntensity);
 
  //Functionality
  bool setMotor(int motorPos);
  bool getMotor(int *encoder);
  bool getFinger(int *encoder, int fingerId);
  bool setRest();
  bool setHome();
  void waitRest(double delay=0);
};

