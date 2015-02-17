#ifndef MATLAB_COMM_H
#define MATLAB_COMM_H

#define MAX_BUFFER 1024

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <matlab_comm/matlab_Ping.h>
#include <matlab_comm/matlab_SendCommand.h>
#include <matlab_comm/matlab_PutArray.h>
#include <matlab_comm/matlab_GetArray.h>
#include <matlab_comm/matlab_PutString.h>
#include <matlab_comm/matlab_GetString.h>
#include <matlab_comm/matlab_AddPath.h>

class MatlabComm
{
  public:
    MatlabComm();
    MatlabComm(ros::NodeHandle* np);
    ~MatlabComm();

    // Subscription
    void subscribe(ros::NodeHandle* np);

    // Call this before program exits so we don't have double freeing issues
    void shutdown();

    //Client functions to simplify calling Matlab ROS services
    bool Ping();

    //send
    bool sendCommand(const char *command);
    bool sendVec(const char *name, int n, double* data);
    bool sendVec(const char *name, const Vec &v);
    bool sendMat(const char *name, int n, int m, double* data);
    bool sendMat(const char *name, const Mat &m);
    bool sendValue(const char *name, double v);
    bool sendString(const char *name, char *str);
    bool addPath(const char *folder);

    //receive
    bool getMat(const char *name, Mat &m);
    Mat getMat(const char *name);
    bool getVec(const char *name, Vec &v);
    Vec getVec(const char *name);
    Quaternion getQuaternion(const char *name);
    double getValue(const char *name);
    bool getValue(const char *name, double *v);
    bool getString(const char *name, char *str, int strLength);
    std::string getString(const char *name);

  private:
    // ROS Service Clients
    ros::ServiceClient handle_matlab_Ping;
    ros::ServiceClient handle_matlab_SendCommand;
    ros::ServiceClient handle_matlab_PutArray;
    ros::ServiceClient handle_matlab_GetArray;
    ros::ServiceClient handle_matlab_PutString;
    ros::ServiceClient handle_matlab_GetString;
    ros::ServiceClient handle_matlab_AddPath;

    // ROS services
    matlab_comm::matlab_Ping matlab_Ping_srv;
    matlab_comm::matlab_SendCommand matlab_SendCommand_srv;
    matlab_comm::matlab_PutArray matlab_PutArray_srv;
    matlab_comm::matlab_GetArray matlab_GetArray_srv;
    matlab_comm::matlab_PutString matlab_PutString_srv;
    matlab_comm::matlab_GetString matlab_GetString_srv;
    matlab_comm::matlab_AddPath matlab_AddPath_srv;
};

#endif //MATLAB_COMM_H
