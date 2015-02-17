#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "P2Interpreter.h"

//ROS specific
#include <ros/ros.h>

#include <handP2_comm/handP2_comm.h>

#define MOT_BUF 10

// Max that an encoder can drift from one calibration to
// another without throwing an error
#define MAX_CAL_DIFF 30

//This are now ROS parameters
//#define ZERO_MOTOR_POS           -5800
//#define STEPS_PER_DEGREE_MOTOR   -171.27
//#define STEPS_PER_DEGREE_FINGERS  11.38
//#define DEFAULT_MOTOR_SPEED       0.5
//#define DEFAULT_MOTOR_FORCE       0.5
//#define MAX_MOTOR_INTENSITY       800
//#define MIN_MOTOR_INTENSITY       200
//#define MAX_MOTOR_SPEED           250
//#define MIN_MOTOR_SPEED             0
//#define ARDUINO_SERIAL_ID        "/dev/tty.usbserial-A600aih2"  
//#define ARDUINO_SERIAL_BAUD      115200
//#define MOTOR_SERIAL_ID          "/dev/tty.usbserial"
//#define MOTOR_SERIAL_BAUD        38400

class HandController
{
 public:
  ros::NodeHandle *node;
  P2Interpreter hand;
  bool calibrated;
  int encMotorZero;
  int encFingersZero[NUM_FINGERS];
  double stepsPerDegreeMotor;
  double stepsPerDegreeFingers;

  HandController();
  HandController(ros::NodeHandle * n);
  virtual ~HandController();
  
  bool defaultHandConfiguration();

  void advertiseServices();
  void advertiseTopics();
  void logCallback(const ros::TimerEvent& event);

  bool hand_Ping(handP2_comm::hand_Ping::Request& req, handP2_comm::hand_Ping::Response& res);
  bool hand_Calibrate(handP2_comm::hand_Calibrate::Request& req, handP2_comm::hand_Calibrate::Response& res);
  bool hand_GetEncoders(handP2_comm::hand_GetEncoders::Request& req, handP2_comm::hand_GetEncoders::Response& res);
  bool hand_GetAngles(handP2_comm::hand_GetAngles::Request& req, handP2_comm::hand_GetAngles::Response& res);
  bool hand_WaitRest(handP2_comm::hand_WaitRest::Request& req, handP2_comm::hand_WaitRest::Response& res);
  bool hand_SetSpeed(handP2_comm::hand_SetSpeed::Request& req, handP2_comm::hand_SetSpeed::Response& res);
  bool hand_SetForce(handP2_comm::hand_SetForce::Request& req, handP2_comm::hand_SetForce::Response& res);
  bool hand_SetEncoder(handP2_comm::hand_SetEncoder::Request& req, handP2_comm::hand_SetEncoder::Response& res);
  bool hand_SetAngle(handP2_comm::hand_SetAngle::Request& req, handP2_comm::hand_SetAngle::Response& res);
  bool hand_SetRest(handP2_comm::hand_SetRest::Request& req, handP2_comm::hand_SetRest::Response& res);
  bool hand_IsMoving(handP2_comm::hand_IsMoving::Request& req, handP2_comm::hand_IsMoving::Response& res);

 private:

  //handles to ROS stuff
  ros::Publisher handle_hand_AnglesLog;
  ros::Publisher handle_hand_EncodersLog;
  ros::ServiceServer handle_hand_Ping;
  ros::ServiceServer handle_hand_Calibrate;
  ros::ServiceServer handle_hand_GetEncoders;
  ros::ServiceServer handle_hand_GetAngles;
  ros::ServiceServer handle_hand_IsMoving;
  ros::ServiceServer handle_hand_WaitRest;
  ros::ServiceServer handle_hand_SetSpeed;
  ros::ServiceServer handle_hand_SetForce; 
  ros::ServiceServer handle_hand_SetEncoder;
  ros::ServiceServer handle_hand_SetAngle;
  ros::ServiceServer handle_hand_SetRest;
};
