#ifndef GRASP_COMM_H
#define GRASP_COMM_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <grasp_comm/grasp_User.h>
#include <grasp_comm/grasp_OpenLoop.h>
#include <grasp_comm/grasp_OpenLoop_EarlyAbort.h>
#include <grasp_comm/grasp_Abort.h>
#include <grasp_comm/grasp_OpenLoop_Detect.h>

using namespace std;

#define MAX_BUFFER 1024

// Default Work object location
// Shared between all grasp routines
#define GM_WORK_X 808.5//500.0
#define GM_WORK_Y -612.86//0.0
#define GM_WORK_Z 0.59//250.0
#define GM_WORK_Q0 0.7084//0.0
#define GM_WORK_QX 0.0003882//0.0
#define GM_WORK_QY -0.0003882//1.0
#define GM_WORK_QZ 0.7058//0.0

// Default Work object location
// Shared between by all grasp routines
#define GM_TOOL_X  0.0
#define GM_TOOL_Y  0.0
#define GM_TOOL_Z  105.0
#define GM_TOOL_Q0 1.0
#define GM_TOOL_QX 0.0
#define GM_TOOL_QY 0.0
#define GM_TOOL_QZ 0.0

// State Machine frequency
#define GM_STATE_MACHINE_RATE 25.0 //Hz

//Grasp nodes names
#define GM_USER_NAME       "grasp_user"
#define GM_OPENLOOP_NAME   "grasp_openloop"
#define GM_OPENLOOP_EARLYABORT_NAME   "grasp_openloop_earlyabort"
#define GM_GRASP_ABORT_NAME	 "grasp_abort"
#define GM_OPENLOOP_DETECT_NAME "grasp_openloop_detect"

typedef enum GM_ErrorType
{
  GM_ERROR_NONE,
  GM_ERROR_COLLISION,
  GM_ERROR_BAD_PARAMS,
  GM_ERROR_UNINITIALIZED,
  GM_ERROR_UNKNOWN,
  GM_NUM_ERRORS
} GM_ErrorType;

typedef enum GM_GraspType
{
  GM_TYPE_NONE,
  GM_TYPE_USER,
  GM_TYPE_OPENLOOP,
  GM_TYPE_OPENLOOP_EARLYABORT,
  GM_TYPE_ABORT,
	GM_TYPE_OPENLOOP_DETECT,
  GM_NUM_TYPES
} GM_GraspType;

//Struct with the input parameters for grasp procedures
typedef struct
{
  //Common
  double x;                   // X coordinate of the center of the grasp procedue (50mm)
  double y;                   // Y coordinate of the center of the grasp procedure (-50mm)
  int id;                     // Experiment id. Integer to be written at the beginning of the log file.
  GM_GraspType type;          // Grasp Type (user, open loop, ...)  

  //For openLoopgGrasp
  double z_lim;               // Absolute z limit that the motion should not pass (150mm)
  double up;                  // How much to go up from the detected surface. (32mm)
  double down;                // How much to go down in the final grasp. (41mm)
  int nFlips;                 // Number of oscillation during the grasp. (4)
  double oscillationAmplitude;// Initial oscillation amplitude. (PI/3rad)
  double handSpeed;           // Speed that hand should close at while grasping (0.45)
}GM_InputParams;

//Struct with the output of grasp procedures
typedef struct
{
  GM_GraspType type;           // Grasp Type (user, open loop, ...)  
  string logFileName;
  GM_ErrorType error;
}GM_OutputParams;


class GraspComm
{
  public:
    // Public access to the ROS node
    ros::NodeHandle *nodePtr;
    
    // Constructors
    GraspComm();
    GraspComm(ros::NodeHandle *n); 
    ~GraspComm();

    void subscribe(ros::NodeHandle *n);

    void shutdown();

    // User functions
    GM_OutputParams grasp(GM_InputParams inparams);

    // Made public in case we want to preload the system before calling grasp 
    bool enable(GM_GraspType graspType);
    bool disable();
  
  private:
    RobotComm robot;
    HandComm hand;
    GM_GraspType currentGraspType;
    vector<string> openNodes;
   
    // Required initialization for selecting a grasp mode
    bool openNode(string nodeName);
    bool closeNodes();

    // ROS Service Clients
    ros::ServiceClient handle_grasp_User;
    ros::ServiceClient handle_grasp_OpenLoop;
    ros::ServiceClient handle_grasp_OpenLoop_EarlyAbort;
    ros::ServiceClient handle_grasp_Abort;
		ros::ServiceClient handle_grasp_OpenLoop_Detect;
  
    // ROS services
    grasp_comm::grasp_User grasp_User_srv;
    grasp_comm::grasp_OpenLoop grasp_OpenLoop_srv;
    grasp_comm::grasp_OpenLoop grasp_OpenLoop_EarlyAbort_srv;
    grasp_comm::grasp_Abort grasp_Abort_srv;
		grasp_comm::grasp_OpenLoop_Detect grasp_OpenLoop_Detect_srv;
};

#endif //GRASP_COMM_H
