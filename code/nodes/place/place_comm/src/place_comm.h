#ifndef PLACE_COMM_H
#define PLACE_COMM_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <place_comm/place_OpenLoop.h>
#include <place_comm/drop_OpenLoop.h>
#include <place_comm/insert_OpenLoop.h>

using namespace std;

#define MAX_BUFFER 1024

// Default Work object location
// Shared between all place routines
#define PM_WORK_X 808.5//500.0
#define PM_WORK_Y -612.86//0.0
#define PM_WORK_Z 0.59//250.0
#define PM_WORK_Q0 0.7084//0.0
#define PM_WORK_QX 0.0003882//0.0
#define PM_WORK_QY -0.0003882//1.0
#define PM_WORK_QZ 0.7058//0.0

// Default Tool object location
// Shared between by all place routines
#define PM_TOOL_X  0.0
#define PM_TOOL_Y  0.0
#define PM_TOOL_Z  105.0
#define PM_TOOL_Q0 1.0
#define PM_TOOL_QX 0.0
#define PM_TOOL_QY 0.0
#define PM_TOOL_QZ 0.0

// State Machine frequency
#define PM_STATE_MACHINE_RATE 25.0 //Hz

//Place nodes names
#define PM_PLACE_OPENLOOP_NAME   "place_openloop"
#define PM_DROP_OPENLOOP_NAME   "drop_openloop"
#define PM_INSERT_OPENLOOP_NAME   "insert_openloop"

typedef enum PM_PlaceType
{
  PM_TYPE_NONE,
  PM_TYPE_OPENLOOP,
  PM_TYPE_DROP,
  PM_TYPE_INSERT,
  PM_TYPE_ERROR,
  PM_NUM_TYPES
} PM_PlaceType;

//Struct with the input parameters for place procedures
typedef struct
{
  //Common
  PM_PlaceType type;  // Place Type (open loop, ...)  

  //For openLoopPlace
  double angle;     // Angle to rotate hand by initially (rad)
  double h_dist;    // Distance to move hand horizontally (mm)
  double v1_dist;   // Distance to move hand vertically the first time (mm)
  double v2_dist;   // Dist to move have vertically the second time (mm) 

  bool test;        // True if we should do this action on the test side of the robot,
                    //  false if we should do this on the training side

}PM_InputParams;

//Struct with the output of place procedures
typedef struct
{
  PM_PlaceType type;          // Place Type (open loop, ...)  
  std::string logFileName;    // File name of log taken during placing
  std::string picFileName;    // File name of picture taken during placing
  bool success;               // Whether or not the place was successful
  bool abort;                 // Whether or not the run was aborted
}PM_OutputParams;


class PlaceComm
{
  public:
    // Public access to the ROS node
    ros::NodeHandle *nodePtr;
    
    // Constructors
    PlaceComm();
    PlaceComm(ros::NodeHandle *n); 
    ~PlaceComm();

    // User functions
    PM_OutputParams place(PM_InputParams inparams);

    void subscribe(ros::NodeHandle *n);
    void shutdown();

    // Made public in case we want to preload the system before calling place 
    bool enable(PM_PlaceType placeType);
    bool disable();
  
  private:
    PM_PlaceType currentPlaceType;
    vector<string> openNodes;

    // Required initialization for selecting a place mode
    bool openNode(string nodeName);
    bool closeNodes();

    // ROS Service Clients
    ros::ServiceClient handle_place_OpenLoop;
    ros::ServiceClient handle_drop_OpenLoop;
    ros::ServiceClient handle_insert_OpenLoop;
  
    // ROS services
    place_comm::place_OpenLoop place_OpenLoop_srv;
    place_comm::drop_OpenLoop drop_OpenLoop_srv;
    place_comm::insert_OpenLoop insert_OpenLoop_srv;
};

#endif //PLACE_COMM_H
