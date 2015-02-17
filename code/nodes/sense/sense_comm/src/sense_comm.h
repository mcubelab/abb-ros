#ifndef SENSE_COMM_H
#define SENSE_COMM_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <sense_comm/sense_User.h>
#include <sense_comm/sense_Vision.h>
#include <sense_comm/sense_Data.h>
#include <sense_comm/sense_Abort.h>

using namespace std;

#define MAX_BUFFER 1024

// Default Work object location
// Shared between all sense routines
#define SM_WORK_X 500.0
#define SM_WORK_Y 0.0
#define SM_WORK_Z 250.0
#define SM_WORK_Q0 0.0
#define SM_WORK_QX 0.0
#define SM_WORK_QY 1.0
#define SM_WORK_QZ 0.0

// Default Work object location
// Shared between by all sense routines
#define SM_TOOL_X  0.0
#define SM_TOOL_Y  0.0
#define SM_TOOL_Z  105.0
#define SM_TOOL_Q0 1.0
#define SM_TOOL_QX 0.0
#define SM_TOOL_QY 0.0
#define SM_TOOL_QZ 0.0

// Speed to move to home position at
#define SM_SPEED_TCP 200.0
#define SM_SPEED_ORI 150.0

// State Machine frequency
#define SM_STATE_MACHINE_RATE 25.0 //Hz

//Grasp nodes names
#define SM_USER_NAME       "sense_user"
#define SM_VISION_NAME     "sense_vision"
#define SM_DATA_NAME       "sense_data"
#define SM_ABORT_NAME      "sense_abort"

typedef enum SM_SenseType
{
  SM_TYPE_NONE,
  SM_TYPE_USER,
  SM_TYPE_VISION,
  SM_TYPE_DATA,
  SM_TYPE_ABORT,
  SM_TYPE_ERROR,
  SM_NUM_TYPES
} SM_SenseType;

//Struct with the input parameters for grasp procedures
typedef struct
{
  //Common
  SM_SenseType type;          // Sense Type (user, vision, brain ...)  
  string logFileName;     // Name of the log file before processing.

  // Only for sense_abort
  string earlyAbortModelName;       // Name of the model to compare against.

  // Only for sense_Data
  string singulationModelName;
  string poseEstimationModelName;

}SM_InputParams;

//Struct with the output of grasp procedures
typedef struct
{
  SM_SenseType type;           // Grasp Type (user, open loop, ...)  
  string logFileName;     // Name of the log file after appending the output from sensing the state of the hand.
  
  bool singulated;             // 1-Object singulated / 0-Object not singulated
  int nMarkers;                // Number of markers grasped.
  double confidence;           // Level of confidence on the output.
  double x;                    // X-coordinate of the grasped object (in tool coordinates).
  double y;                    // Y-coordinate of the grasped object (in tool coordinates).
  double distance;             // Distance from the center of the marker to the center of the palm (origin).
  double theta;                // Orientation of the grasped object .
  double alpha;                // angle of line perpendicular to marker axis 

  // Only for sense_Vision
  string picFileName;     // Name of the picture taken by the camera.
}SM_OutputParams;


class SenseComm
{
  public:
    // Public access to the ROS node
    ros::NodeHandle *nodePtr;
    
    // Constructors
    SenseComm();
    SenseComm(ros::NodeHandle *n); 
    ~SenseComm();

    // User functions
    SM_OutputParams sense(SM_InputParams inparams);

    void subscribe(ros::NodeHandle *n);
    void shutdown();

    // Made public in case we want to preload the system before calling grasp 
    bool enable(SM_SenseType senseType);
    bool disable();
  
  private:
    SM_SenseType currentSenseType;
    vector<string> openNodes;

    // Required initialization for selecting a grasp mode
    bool openNode(string nodeName);
    bool closeNodes();

    // ROS Service Clients
    ros::ServiceClient handle_sense_User;
    ros::ServiceClient handle_sense_Vision;
    ros::ServiceClient handle_sense_Data;
    ros::ServiceClient handle_sense_Abort;
  
    // ROS services
    sense_comm::sense_User sense_User_srv;
    sense_comm::sense_Vision sense_Vision_srv;
    sense_comm::sense_Data sense_Data_srv;
    sense_comm::sense_Abort sense_Abort_srv;
};

#endif //SENSE_COMM_H
