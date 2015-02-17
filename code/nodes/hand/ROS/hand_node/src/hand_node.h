//
// Name: Robbie Paolini
//
// File Name: hand_node.hpp
//
// Last Modified: 8/31/2012
// Last Modified: 10/03/2013 Nikhil: Added new service hand.DADA
//
// Header file for the hand node, which controls P3, a simple hand 
// designed in the Manipulation Lab. P3 is controlled by an arduino. This
// program interacts with the arduino over a USB serial port. It records
// and publishes finger, motor, and palm sensor data. It allows the user
// to change the speed and current limit of the motor and then command it
// to move to a certain pose. Since the motor encoder is relative, it also
// implements a calibration routine so we can get consistent motor poses 
// across runs. Please look at the arduino code on P3 for more details with
// communication

#ifndef HAND_NODE_H
#define HAND_NODE_H

#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

//ROS specific
#include <ros/ros.h>

#include <serialPort/serialPort.h>
#include <hand_comm/hand_comm.h>

#define MOT_BUF 10

#define MAX_BUFFER 2048

// Define this if using P3.5, otherwise, comment it out for P3
#define P3_5

// Commandable encoder positons for the motor
#define MAX_MOTOR_POS 32767 
#define MIN_MOTOR_POS -32768

// Minimum loop time for WaitRest loop
#define MIN_DELAY 0.0001

// Number of digits expected in an encoder and force data message
#define ENC_DATA_WIDTH 4
#define FORCE_DATA_WIDTH 4

// Max that an encoder can drift from one calibration to
// another without throwing an error
#define MAX_CAL_DIFF 30

// Rate at which arduino is publishing data (seconds)
#define DATA_RATE   0.03
// Rate at which we are reading from the serial port (seconds)
#define LOGGER_THREAD_RATE 0.025

// Constants in Y = Mx+B for computing the number of counts/sec,
//  where x is a motor speed between 0 and 1
#define CNTS_PER_SEC_M 6230.0
#define CNTS_PER_SEC_B -431.5

// Constant to determine how close together motor positions have to
//  be to consider the hand to not be moving
#define MOVING_FACTOR   1.0

// Speed and force to move the hand at when moving fast during calibration
//#define CAL_FAST_SPEED  0.8
//#define CAL_FAST_FORCE  0.5
#define CAL_FAST_SPEED  0.9
#define CAL_FAST_FORCE  0.95

// Speed and force to move the hand at when moving slowly to calibrate
//#define CAL_SLOW_SPEED  0.3
//#define CAL_SLOW_FORCE  0.5
#define CAL_SLOW_SPEED  0.65
#define CAL_SLOW_FORCE  0.85

// Distance from expected motor zero pose to move to when fast calibrating
//#define CAL_FAST_OFFSET 1000
#ifdef P3_5
#define CAL_FAST_OFFSET -1000  //sign is flipped for P-3.5 as 8:1 motor rotates in the opposite direction as that of 22:1 motor used for P-3.
#else
#define CAL_FAST_OFFSET 1000
#endif
// Name of file where we save the current calibration data to
#define CAL_FILE_NAME  "P2Calibration.txt"

// Matrix to convert raw forces to calibrated forces
const double force_cal[NUM_HAND_FORCES][NUM_RAW_HAND_FORCES] = 
{
  {1, 0, 0},
  {0, 1, 0},
  {0, 0, 1}
};

typedef struct non_block_t
{
  double delay1, delay2, angle1, angle2;
} non_block_t;

typedef enum
{
  HNB_DADA = 0,
  HNB_NO_ACTION,
  HNB_NUM_ACTIONS
} non_block_action_t;

// Mutexes used to prevent terrible things from happening when accessing
// data across multiple threads
pthread_mutex_t encMotorMutex;    // When accessing motor encoder data
pthread_mutex_t encFingersMutex;  // When accessing finger encoder data
pthread_mutex_t rawForcesMutex;   // When accessing raw palm data
pthread_mutex_t serialOutMutex;   // When sending data over the serial port


// Class definition
class HandController
{
 public:
  // Constructors and destructors
  HandController();
  HandController(ros::NodeHandle * n);
  virtual ~HandController();
  
  // Services
  bool hand_Ping(hand_comm::hand_Ping::Request& req, 
      hand_comm::hand_Ping::Response& res);
  bool hand_Calibrate(hand_comm::hand_Calibrate::Request& req, 
      hand_comm::hand_Calibrate::Response& res);
  bool hand_GetEncoders(hand_comm::hand_GetEncoders::Request& req, 
      hand_comm::hand_GetEncoders::Response& res);
  bool hand_GetRawForces(hand_comm::hand_GetRawForces::Request& req, 
      hand_comm::hand_GetRawForces::Response& res);
  bool hand_GetForces(hand_comm::hand_GetForces::Request& req, 
      hand_comm::hand_GetForces::Response& res);
  bool hand_GetAngles(hand_comm::hand_GetAngles::Request& req, 
      hand_comm::hand_GetAngles::Response& res);
  bool hand_WaitRest(hand_comm::hand_WaitRest::Request& req, 
      hand_comm::hand_WaitRest::Response& res);
  bool hand_SetSpeed(hand_comm::hand_SetSpeed::Request& req, 
      hand_comm::hand_SetSpeed::Response& res);
  bool hand_SetForce(hand_comm::hand_SetForce::Request& req, 
      hand_comm::hand_SetForce::Response& res);
  bool hand_SetEncoder(hand_comm::hand_SetEncoder::Request& req, 
      hand_comm::hand_SetEncoder::Response& res);
  bool hand_SetAngle(hand_comm::hand_SetAngle::Request& req, 
      hand_comm::hand_SetAngle::Response& res);
  bool hand_SetRest(hand_comm::hand_SetRest::Request& req, 
      hand_comm::hand_SetRest::Response& res);
  bool hand_IsMoving(hand_comm::hand_IsMoving::Request& req, 
      hand_comm::hand_IsMoving::Response& res);
  bool hand_DADA(hand_comm::hand_DADA::Request& req, 
      hand_comm::hand_DADA::Response& res);

  // This function reads in serial data from the arduino
  // It is called at a certain rate by a timer event
  void logCallback(const ros::TimerEvent& event);

  // Set the default hand configuration
  bool defaultHandConfiguration();

  // Advertise topics and services
  void advertiseServices();
  void advertiseTopics();

  // Connect and disconnect from the arduino
  bool connectArduino();
  bool disconnectArduino();

  // Load calibration from file
  bool loadCalibration();

  // Pointer to ros node
  ros::NodeHandle *node;

  // Zero encoder count
  int encMotorAtZeroAngle; //encMotorZero;
  int encFingersAtZeroAngle[NUM_FINGERS]; //encFingersZero[NUM_FINGERS];

  // Whether or not we have calibrated the hand
  bool calibrated;
  
  non_block_t non_block_data;
  
  non_block_action_t non_block_action;

  // Delay, angle, delay, angle
  bool dada(double delay1, double angle1, double delay2, double angle2);


 private:

  // Set the speed of the hand
  bool setMotorSpeed(double mSpeed);
  // Set the current limit fo the hand
  bool setMotorIntensity(double mIntensity);
  // Move the motor to a certain encoder count
  bool setMotorPos(int mPos);
  // Reset the encoder count of the motor to 0
  bool setHome();
  // Stop the motor
  bool setRest();
  // Block until motor stops moving
  bool waitRest(double delay = 0);
  

  // Update "moving" variable
  void checkMoving();

  // Compute forces and angles based on raw forces and 
  // encoder counts, respectively
  void getForces(double forces[NUM_HAND_FORCES]);
  void getAngles(double angles[NUM_FINGERS]);

  
  // Conversion factor between counts and angles
  double stepsPerDegreeMotor;
  double stepsPerDegreeFingers;

  // Serial port connection for the arduino, and whether it's connected
  SerialPort spArduino;
  bool connectedArduino;

  // Current motor speed and current limit
  double motorSpeed;
  double motorIntensity;

  // Current sensor data
  int encMotor;                   // Current position of motor
  int encFingers[NUM_FINGERS];    // Current position of fingers
  int rawForces[NUM_RAW_HAND_FORCES];  // Current raw values from palm sensors

  // Keeps track of read in serial characters that are not yet used
  char serialBuffer[MAX_BUFFER];
  int curSerial;

  // Ring buffer for keeping track of whether the hand is moving
  int motBuf[MOT_BUF];
  bool bufFull;
  int bufCnt;

  // Whether or not the hand is moving
  bool moving;

  //handles to ROS stuff
  ros::Publisher handle_hand_AnglesLog;
  ros::Publisher handle_hand_EncodersLog;
  ros::Publisher handle_hand_RawForcesLog;
  ros::Publisher handle_hand_ForcesLog;
  ros::ServiceServer handle_hand_Ping;
  ros::ServiceServer handle_hand_Calibrate;
  ros::ServiceServer handle_hand_GetEncoders;
  ros::ServiceServer handle_hand_GetRawForces;
  ros::ServiceServer handle_hand_GetForces;
  ros::ServiceServer handle_hand_GetAngles;
  ros::ServiceServer handle_hand_IsMoving;
  ros::ServiceServer handle_hand_WaitRest;
  ros::ServiceServer handle_hand_SetSpeed;
  ros::ServiceServer handle_hand_SetForce; 
  ros::ServiceServer handle_hand_SetEncoder;
  ros::ServiceServer handle_hand_SetAngle;
  ros::ServiceServer handle_hand_SetRest;
  ros::ServiceServer handle_hand_DADA;
};

#endif // HAND_NODE_H
