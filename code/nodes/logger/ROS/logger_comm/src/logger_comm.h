#ifndef LOGGER_COMM_H
#define LOGGER_COMM_H

#include <ros/ros.h>

#include <logger_comm/logger_Start.h>
#include <logger_comm/logger_Stop.h>
#include <logger_comm/logger_Append.h>
#include <logger_comm/logger_Create.h>
#include <logger_comm/logger_Copy.h>

#include <logger_comm/logger_SystemLog.h>

#include <string>
using namespace std;

class LoggerComm
{
  public:
    LoggerComm();
    LoggerComm(ros::NodeHandle* np);
    ~LoggerComm();

    // Subscribe Function
    void subscribe(ros::NodeHandle* np);

    // Subscribe to Topics
    void subscribeSystem(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const logger_comm::logger_SystemLogConstPtr&));

    // Shutdown service clients
    void shutdown();

    bool Start(string &filename, string folder, int id=0);
    bool Start(string &filename, int id=0);
    bool StartGrasp(string &filename, int id=0);
    bool StartPlace(string &filename, int id=0);
    bool Start(int id=0);
    bool StartGrasp(int id=0);
    bool StartPlace(int id=0);
    bool Stop();
    bool Append(string filename, string info);
    bool Create(string &filename);
    bool Copy(string filename, string folder);

  private:
    // Subscribers
    ros::Subscriber logger_system_sub;

    // ROS Service Clients
    ros::ServiceClient handle_logger_Start;
    ros::ServiceClient handle_logger_Stop;
    ros::ServiceClient handle_logger_Append;
    ros::ServiceClient handle_logger_Create;
    ros::ServiceClient handle_logger_Copy;
 
    // ROS service messages
    logger_comm::logger_Start logger_Start_srv;
    logger_comm::logger_Stop logger_Stop_srv;
    logger_comm::logger_Append logger_Append_srv;
    logger_comm::logger_Create logger_Create_srv;
    logger_comm::logger_Copy logger_Copy_srv;
};
#endif //LOGGER_COMM_H
