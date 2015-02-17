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

#define CERT_SCALE 0.125
#define TH_DIST_SCALE 100

typedef enum SM_State
{
  SM_INIT,
  SM_LOAD_MODELS,
  SM_QUERY_MODEL,
  SM_LOG,
  SM_END,
  SM_ERROR,
  SM_NUM_STATES
} SM_State;

LoggerComm logger;
BrainComm brain;

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_sense_Data;

int nMarkers;
double x;
double y;
double theta;
double dist;

double theta_s;
double dist_s;

double certainty;

int label;
std::string currentSingulationModel;
std::string currentPoseEstimationModel;

int stepStateMachine(int currState, SM_InputParams senseParams);
bool sense_Data(sense_comm::sense_Data::Request& req, sense_comm::sense_Data::Response& res);
