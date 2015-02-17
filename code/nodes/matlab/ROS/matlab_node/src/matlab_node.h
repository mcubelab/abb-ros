#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <engine.h>
//#include <matrix.h>

//ROS specific
#include <ros/ros.h>
#include <matlab_comm/matlab_comm.h>

#define MAX_BUFFER 1024
#define MAX_MATLAB_BUFFER 8192

bool ready; //Set to true when Matlab node is ready to serve commands

bool matlab_Ping(matlab_comm::matlab_Ping::Request& req, matlab_comm::matlab_Ping::Response& res);
bool matlab_SendCommand(matlab_comm::matlab_SendCommand::Request& req, matlab_comm::matlab_SendCommand::Response& res);
bool matlab_PutArray(matlab_comm::matlab_PutArray::Request& req, matlab_comm::matlab_PutArray::Response& res);
bool matlab_GetArray(matlab_comm::matlab_GetArray::Request& req, matlab_comm::matlab_GetArray::Response& res);
bool matlab_PutString(matlab_comm::matlab_PutString::Request& req, matlab_comm::matlab_PutString::Response& res);
bool matlab_GetString(matlab_comm::matlab_GetString::Request& req, matlab_comm::matlab_GetString::Response& res);
bool matlab_AddPath(matlab_comm::matlab_AddPath::Request& req, matlab_comm::matlab_AddPath::Response& res);

bool evalString(char *buffer, bool echo);

ros::NodeHandle *nodePtr;
ros::ServiceServer handle_matlab_Ping;
ros::ServiceServer handle_matlab_SendCommand;
ros::ServiceServer handle_matlab_PutArray;
ros::ServiceServer handle_matlab_GetArray;
ros::ServiceServer handle_matlab_PutString;
ros::ServiceServer handle_matlab_GetString;
ros::ServiceServer handle_matlab_AddPath;

//Matlab Engine
Engine *matEng;

// Buffer for storing matlab echo output
char echo_buffer[MAX_MATLAB_BUFFER+1];
