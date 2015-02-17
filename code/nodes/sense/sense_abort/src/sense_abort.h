#include <stdio.h>
#include <limits>
#include <iostream>
#include <string>

//ROS specific
#include <ros/ros.h>
#include <matVec/matVec.h>
#include <logger_comm/logger_comm.h>
#include <brain_comm/brain_comm.h>
#include <sense_comm/sense_comm.h>

typedef enum SM_State
{
  SM_INIT,
  SM_LOAD_MODEL,
  SM_QUERY_MODEL,
  SM_LOG,
  SM_END,
  SM_ERROR,
  SM_NUM_STATES
} SM_State;

LoggerComm logger;
BrainComm brain;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_sense_Abort;

bool singulated;
int nMarkers;
double x;
double y;
double theta;
double dist;

int label;
std::string currentModel;

int stepStateMachine(int currState, SM_InputParams senseParams);
bool sense_Abort(sense_comm::sense_Abort::Request& req, sense_comm::sense_Abort::Response& res);
