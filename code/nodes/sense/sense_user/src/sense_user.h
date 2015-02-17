#include <stdio.h>
#include <limits>
#include <iostream>
#include <string>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
//#include <robot_comm/robot_comm.h>
#include <logger_comm/logger_comm.h>
#include <util_comm/util_comm.h>
#include <sense_comm/sense_comm.h>
// Speed to move to home position at
#define SM_USER_SPEED_TCP 100.0
#define SM_USER_SPEED_ORI 50.0

typedef enum SM_State
{
  SM_INIT,
  SM_GOTO_CAMERA,
  SM_ASK_USER,
  SM_LOG,
  SM_END,
  SM_ERROR,
  SM_NUM_STATES
} SM_State;

//RobotComm robot;
LoggerComm logger;
UtilComm util;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_sense_User;

int nMarkers;
double x;
double y;
double theta;
double alpha;
double dist;

double cameraJ1,cameraJ2,cameraJ3,cameraJ4,cameraJ5,cameraJ6;

int stepStateMachine(int currState, SM_InputParams senseParams);
bool sense_User(sense_comm::sense_User::Request& req, sense_comm::sense_User::Response& res);
