#include <stdio.h>
#include <limits>
#include <iostream>
#include <string>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
#include <util_comm/util_comm.h>
#include <logger_comm/logger_comm.h>
#include <vision_comm/vision_comm.h>
#include <sense_comm/sense_comm.h>
#include <hand_comm/hand_comm.h>

typedef enum SM_State
{
  SM_INIT,
  SM_GOTO_CAMERA,
  SM_TAKE_PIC,
  SM_LOG,
  SM_END,
  SM_ERROR,
  SM_NUM_STATES
} SM_State;

//RobotComm robot;
UtilComm util;
LoggerComm logger;
VisionComm vision;
HandComm hand;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_sense_Vision;

int nMarkers;
double x;
double y;
double theta;
double alpha;
double confidence;
double dist;
std::string picFileName;
int motor_enc;
int finger_enc[NUM_FINGERS];
int raw_forces[NUM_RAW_HAND_FORCES];

double cameraJ1,cameraJ2,cameraJ3,cameraJ4,cameraJ5,cameraJ6;

int stepStateMachine(int currState, SM_InputParams senseParams);
bool sense_Vision(sense_comm::sense_Vision::Request& req, 
    sense_comm::sense_Vision::Response& res);
