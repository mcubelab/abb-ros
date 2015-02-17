#include <time.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <serialPort/serialPort.h>
#include <handP2_comm/handP2_comm.h>

#define MAX_BUFFER 256
#define SAMPLING_TIME 0.025 //Sampling the hand at 40Hz
#define NSAMPLES 20
#define NUM_SAMPS 10
#define MAX_MOTOR_POS 30000 
#define MIN_MOTOR_POS -30000

#define MIN_VARIANCE 30.0
#define VAR_FACT 1.0

#define NO_ENC_VAL -9999

// Pulse widths can be from 1 to 4096
#define MAX_PULSE 4096

// The maximum we expect an encoder count to vary from 1 time step to the next
#define MAX_ENC_DIFF 1000

// Windows near edges (1 or MAX_PULSE) where we 
// should start worrying about cross overs
#define WINDOW_WIDTH 500

#define MIN_WINDOW WINDOW_WIDTH
#define MAX_WINDOW (MAX_PULSE - WINDOW_WIDTH)

const int calibration_reference[NUM_FINGERS] = 
{
  2000,
  3000,
  3000,
};

const int enc_limits[NUM_FINGERS][2] = 
{
  {900, 2500},
  {1500, 3000},
  {1900, 3500}
};

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
  int encFingers[NUM_FINGERS];
  int motBuf[NUM_SAMPS];
  bool moving;
  bool bufFull;
  int bufCnt;
  bool calibrating;

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
  bool isMoving(bool *mov);
  void waitRest(double delay=0);

  void checkMoving();
};

