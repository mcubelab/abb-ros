#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>

//ROS specific
#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <logger_comm/logger_SystemLog.h>
#include <logger_comm/logger_Start.h>
#include <logger_comm/logger_Stop.h>
#include <logger_comm/logger_Append.h>
#include <logger_comm/logger_Create.h>
#include <logger_comm/logger_Copy.h>

#define COMMON_FOLDER "log"
#define LOG_BUFFER 1024

pthread_mutex_t cartesianLogMutex;
pthread_mutex_t jointsLogMutex;
pthread_mutex_t forceLogMutex;
pthread_mutex_t encodersLogMutex;
pthread_mutex_t anglesLogMutex;
pthread_mutex_t handRawForcesLogMutex;
pthread_mutex_t handForcesLogMutex;


ros::NodeHandle* nodePtr;
ros::Timer loggerTimer;
ros::Timer scannerTimer;
ros::Time logStartTime;

ros::Subscriber handle_robot_CartesianLog;
ros::Subscriber handle_robot_JointsLog;
ros::Subscriber handle_robot_ForceLog;
ros::Subscriber handle_hand_EncodersLog;
ros::Subscriber handle_hand_AnglesLog;
ros::Subscriber handle_hand_RawForcesLog;
ros::Subscriber handle_hand_ForcesLog;
ros::Publisher handle_logger_SystemLog;
ros::ServiceServer handle_logger_Start;
ros::ServiceServer handle_logger_Stop;
ros::ServiceServer handle_logger_Append;
ros::ServiceServer handle_logger_Create;
ros::ServiceServer handle_logger_Copy;
//kinect
ros::Subscriber handle_hand_kinect;

FILE *fLog;
int id;

char currentFileName[LOG_BUFFER];

bool logging;
bool robot_CartesianLog_present, robot_CartesianLog_logging;
bool robot_JointsLog_present, robot_JointsLog_logging;
bool robot_ForceLog_present, robot_ForceLog_logging;
bool hand_EncodersLog_present, hand_EncodersLog_logging;
bool hand_AnglesLog_present, hand_AnglesLog_logging;
bool hand_RawForcesLog_present, hand_RawForcesLog_logging;
bool hand_ForcesLog_present, hand_ForcesLog_logging;

double robot_CartesianLog_values[7];
double robot_JointsLog_values[NUM_JOINTS];
double robot_ForceLog_values[NUM_FORCES];
int hand_EncodersLog_values[NUM_FINGERS+1];
double hand_AnglesLog_values[NUM_FINGERS+1];
int hand_RawForcesLog_values[NUM_RAW_HAND_FORCES];
double hand_ForcesLog_values[NUM_HAND_FORCES];
//kinect
int kinect_height;


void scanAndSubscribeTopics();
void advertiseServices();
void advertiseTopics();


//Services
bool logger_Start(logger_comm::logger_Start::Request& req, logger_comm::logger_Start::Response& res);
bool logger_Stop(logger_comm::logger_Stop::Request& req, logger_comm::logger_Stop::Response& res);
bool logger_Append(logger_comm::logger_Append::Request& req, logger_comm::logger_Append::Response& res);
bool logger_Create(logger_comm::logger_Create::Request& req, logger_comm::logger_Create::Response& res);
bool logger_Copy(logger_comm::logger_Copy::Request& req, logger_comm::logger_Copy::Response& res);

//Callbacks
void logCallback(const ros::TimerEvent& event);
void scannerCallback(const ros::TimerEvent& event);
void robot_CartesianLog_Callback(const robot_comm::robot_CartesianLog& msg);
void robot_JointsLog_Callback(const robot_comm::robot_JointsLog& msg);
void robot_ForceLog_Callback(const robot_comm::robot_ForceLog& msg);
void hand_EncodersLog_Callback(const hand_comm::hand_EncodersLog& msg);
void hand_AnglesLog_Callback(const hand_comm::hand_AnglesLog& msg);
void hand_RawForcesLog_Callback(const hand_comm::hand_RawForcesLog& msg);
void hand_ForcesLog_Callback(const hand_comm::hand_ForcesLog& msg);

// Helper funtion to stop logs
bool stopLog();
