#include <errno.h>
#include <fcntl.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <matVec/matVec.h>
#include <math.h>

#include "ABBInterpreter.h"

#include "PracticalSocket/PracticalSocket.h" // For UDPSocket and SocketException
#include "tinyxml.h"

//ROS specific
#include <ros/ros.h>
#include <robot_comm/robot_comm.h>
#include <sensor_msgs/JointState.h>

//#define MAX_BUFFER 256
#define MAX_BUFFER 10000
#define ID_CODE_MAX 999

#define SERVER_BAD_MSG 0
#define SERVER_OK 1
#define SERVER_COLLISION 2
#define SERVER_BAD_IK 3
#define SERVER_BAD_FK 4

#define MAX_TRANS_STEP 2.0
#define MAX_ROT_STEP (0.5 * DEG2RAD)
#define MAX_J_STEP 0.5

#define NB_FREQ 200.0
#define STOP_CHECK_FREQ 25.0
#define DIST_CHECK_FREQ 100.0

#define SAFETY_FACTOR 0.90
#define MINIMUM_TRACK_DIST_TRANS 1.0 //mm
#define MAXIMUM_TRACK_DIST_TRANS 20.0 //mm
#define MINIMUM_TRACK_DIST_ORI 0.333  //deg
#define MAXIMUM_TRACK_DIST_ORI 6.66 //deg
#define INFINITY_TRACK_DIST_TRANS 1000.0 ///mm
#define INFINITY_TRACK_DIST_ORI 333.0 //deg

#define MINIMUM_NB_SPEED_TCP 1.0 //mm/s
#define MINIMUM_NB_SPEED_ORI 0.333 //deg/s

// Mutex used for threads
pthread_mutex_t nonBlockMutex;
pthread_mutex_t jointUpdateMutex;
pthread_mutex_t cartUpdateMutex;
pthread_mutex_t forceUpdateMutex;
pthread_mutex_t sendRecvMutex;

class RobotController
{
 public:
  RobotController(ros::NodeHandle * n);
  virtual ~RobotController();

  // Initialize the robot
  bool init(std::string id = "");
  string robotname;
  string robotname_sl;

  // Service Callbacks
  bool robot_Ping(
      robot_comm::robot_Ping::Request& req, 
      robot_comm::robot_Ping::Response& res);
  bool robot_SetDefaults(
      robot_comm::robot_SetDefaults::Request& req, 
      robot_comm::robot_SetDefaults::Response& res);
  bool robot_SetCartesianJ(
      robot_comm::robot_SetCartesianJ::Request& req, 
      robot_comm::robot_SetCartesianJ::Response& res);
  bool robot_SetCartesian(
      robot_comm::robot_SetCartesian::Request& req, 
      robot_comm::robot_SetCartesian::Response& res);
  bool robot_GetCartesian(
      robot_comm::robot_GetCartesian::Request& req, 
      robot_comm::robot_GetCartesian::Response& res);
  bool robot_SetJoints(
      robot_comm::robot_SetJoints::Request& req, 
      robot_comm::robot_SetJoints::Response& res);
  bool robot_GetJoints(
      robot_comm::robot_GetJoints::Request& req, 
      robot_comm::robot_GetJoints::Response& res);
  bool robot_GetIK(
      robot_comm::robot_GetIK::Request& req, 
      robot_comm::robot_GetIK::Response& res);
  bool robot_GetFK(
      robot_comm::robot_GetFK::Request& req, 
      robot_comm::robot_GetFK::Response& res);
  bool robot_Stop(
      robot_comm::robot_Stop::Request& req, 
      robot_comm::robot_Stop::Response& res);
  bool robot_SetTool(
      robot_comm::robot_SetTool::Request& req, 
      robot_comm::robot_SetTool::Response& res);
  bool robot_SetInertia(
      robot_comm::robot_SetInertia::Request& req, 
      robot_comm::robot_SetInertia::Response& res);
  bool robot_SetWorkObject(
      robot_comm::robot_SetWorkObject::Request& req, 
      robot_comm::robot_SetWorkObject::Response& res);
  bool robot_SetComm(
      robot_comm::robot_SetComm::Request& req, 
      robot_comm::robot_SetComm::Response& res);
  bool robot_SpecialCommand(
      robot_comm::robot_SpecialCommand::Request& req,
      robot_comm::robot_SpecialCommand::Response& res);
  bool robot_SetVacuum(
      robot_comm::robot_SetVacuum::Request& req, 
      robot_comm::robot_SetVacuum::Response& res);
  bool robot_GetState(
      robot_comm::robot_GetState::Request& req, 
      robot_comm::robot_GetState::Response& res);
  bool robot_SetSpeed(
      robot_comm::robot_SetSpeed::Request& req, 
      robot_comm::robot_SetSpeed::Response& res);
  bool robot_SetAcc(
      robot_comm::robot_SetAcc::Request& req, 
      robot_comm::robot_SetAcc::Response& res);
  bool robot_SetZone(
      robot_comm::robot_SetZone::Request& req, 
      robot_comm::robot_SetZone::Response& res);
  bool robot_SetTrackDist(
      robot_comm::robot_SetTrackDist::Request& req, 
      robot_comm::robot_SetTrackDist::Response& res);
  bool robot_IsMoving(
      robot_comm::robot_IsMoving::Request& req, 
      robot_comm::robot_IsMoving::Response& res);
  bool robot_Approach(
      robot_comm::robot_Approach::Request& req,
      robot_comm::robot_Approach::Response& res);
  // Buffer (joints) Comm declerations:
  bool robot_AddJointPosBuffer(
	  robot_comm::robot_AddJointPosBuffer::Request& req,
	  robot_comm::robot_AddJointPosBuffer::Response& res);
  bool robot_ExecuteJointPosBuffer(
	  robot_comm::robot_ExecuteJointPosBuffer::Request& req,
	  robot_comm::robot_ExecuteJointPosBuffer::Response& res);
  bool robot_ClearJointPosBuffer(
	  robot_comm::robot_ClearJointPosBuffer::Request& req,
	  robot_comm::robot_ClearJointPosBuffer::Response& res);
    // Buffer (TCP)
  bool robot_AddBuffer(
	  robot_comm::robot_AddBuffer::Request& req,
	  robot_comm::robot_AddBuffer::Response& res);
  bool robot_ExecuteBuffer(
	  robot_comm::robot_ExecuteBuffer::Request& req,
	  robot_comm::robot_ExecuteBuffer::Response& res);
  bool robot_ClearBuffer(
	  robot_comm::robot_ClearBuffer::Request& req,
	  robot_comm::robot_ClearBuffer::Response& res);
	  

  // Advertise Services and Topics
  void advertiseServices();
  void advertiseTopics();

  // Call back function for the logging which will be called by a timer event
  void logCallback(const ros::TimerEvent&);
  
  // Call back function for the RRI which will be called by a timer event
  void rriCallback(const ros::TimerEvent&);
  
  // Public access to the ROS node
  ros::NodeHandle *node;

  // Non-Blocking move variables
  bool non_blocking;  // Whether we are in non-blocking mode
  bool do_nb_move;    // Whether we are currently moving in non-blocking mode
  bool targetChanged; // Whether a new target was specified
  bool stopRequest;   // Set to true when we are trying to stop the robot
  bool stopConfirm;   // Set to true when the thread is sure it's stopped
  bool cart_move;     // True if we're doing a cartesian move, false if joint
  bool cart_move_j;     // True if we're doing a cartesian move, and we want the robot to use a joint move to do it, false if just a regular cartesian move

  // Variables dealing with changing non-blocking speed and step sizes
  bool changing_nb_speed; // Overrides setSpeed safety
  double curCartStep;     // Largest cartesian stepsize during non-blocking
  double curOrientStep;   // Largest orientation step size during non-blocking
  double curJointStep;    // Largest joint step size during non-blocking
  double curDist[3];      // Max allowable tracking error (pos, ang, joint)

  // Most recent goal position, and the final target position
  Vec curGoalP;
  Quaternion curGoalQ;
  Vec curTargP;
  Quaternion curTargQ;
  double curGoalJ[NUM_JOINTS];
  double curTargJ[NUM_JOINTS];

  // Error Handling
  int errorId;
  char errorReply[MAX_BUFFER];
  
  // Move commands are public so that the non-blocking thread can use it
  bool setCartesianJ(double x, double y, double z, 
    double q0, double qx, double qy, double qz);
  bool setCartesian(double x, double y, double z, 
    double q0, double qx, double qy, double qz);
  bool setJoints(double j1, double j2, double j3, 
      double j4, double j5, double j6);
      
  // Buffer Commands for joint positions
  bool addJointPosBuffer(double j1, double j2, double j3, double j4, double j5, double j6);
  bool executeJointPosBuffer();
  bool clearJointPosBuffer();
  
  // Buffer Commands for joint positions
  bool addBuffer(double x, double y, double z, double q0, double qx, double qy, double qz);
  bool executeBuffer();
  bool clearBuffer();

  // Functions that compute our distance from the current position to the goal
  double posDistFromGoal();
  double orientDistFromGoal();
  double jointDistFromGoal();

 private:
  // Socket Variables
  bool motionConnected;
  bool loggerConnected;
  bool RRIConnected;
  int robotMotionSocket;
  int robotLoggerSocket;
  UDPSocket* RRIsock;

  // Connect to servers on the robot
  bool connectMotionServer(const char* ip, int port);
  bool connectLoggerServer(const char* ip, int port);
  bool establishRRI(int port);
  
  // Sets up the default robot configuration
  bool defaultRobotConfiguration();

  //handles to ROS stuff
  
  ros::Publisher handle_robot_RosJointState;
  ros::Publisher handle_robot_CartesianLog;
  ros::Publisher handle_robot_JointsLog;
  ros::Publisher handle_robot_ForceLog;
  ros::ServiceServer handle_robot_Ping;
  ros::ServiceServer handle_robot_SetCartesian;
  ros::ServiceServer handle_robot_SetCartesianJ;
  ros::ServiceServer handle_robot_GetCartesian;
  ros::ServiceServer handle_robot_SetJoints;
  ros::ServiceServer handle_robot_GetJoints;
  ros::ServiceServer handle_robot_GetIK;
  ros::ServiceServer handle_robot_GetFK;
  ros::ServiceServer handle_robot_Stop;
  ros::ServiceServer handle_robot_SetTool;
  ros::ServiceServer handle_robot_SetInertia;
  ros::ServiceServer handle_robot_SetWorkObject;
  ros::ServiceServer handle_robot_SetSpeed;
  ros::ServiceServer handle_robot_SetAcc;
  ros::ServiceServer handle_robot_GetState;
  ros::ServiceServer handle_robot_SetZone;
  ros::ServiceServer handle_robot_SetTrackDist;
  ros::ServiceServer handle_robot_SpecialCommand;
  ros::ServiceServer handle_robot_SetComm;
  ros::ServiceServer handle_robot_SetVacuum;
  ros::ServiceServer handle_robot_IsMoving;
  ros::ServiceServer handle_robot_SetDefaults;
  ros::ServiceServer handle_robot_Approach;
  ros::ServiceServer handle_robot_AddJointPosBuffer;
  ros::ServiceServer handle_robot_ExecuteJointPosBuffer;
  ros::ServiceServer handle_robot_ClearJointPosBuffer;
  ros::ServiceServer handle_robot_AddBuffer;
  ros::ServiceServer handle_robot_ExecuteBuffer;
  ros::ServiceServer handle_robot_ClearBuffer;
 
  // Helper function for communicating with robot server
  bool sendAndReceive(char *message, int messageLength, 
      char*reply, int idCode=-1);

  // Internal functions that communicate with the robot
  bool ping();
  bool getCartesian(double &x, double &y, double &z, 
      double &q0, double &qx, double &qy, double &qz);
  bool getJoints(double &j1, double &j2, double &j3,
      double &j4, double &j5, double &j6);
  bool setTool(double x, double y, double z, 
    double q0, double qx, double qy, double qz);
  bool setInertia(double m, double cgx, double cgy, 
    double cgz, double ix, double iy, double iz);
  bool setWorkObject(double x, double y, double z, 
    double q0, double qx, double qy, double qz);
  bool specialCommand(int command, double param1=0, double param2=0, double param3=0, double param4=0, double param5=0);
  bool setVacuum(int v);
  bool setSpeed(double tcp, double ori);
  bool setAcc(double acc, double deacc);
  bool setZone(int z);
  bool stop_nb();
  bool setComm(int mode);
  
  // Check if robot is currently moving or not
  bool is_moving();

  // Functions to handle setting up non-blocking step sizes
  bool setTrackDist(double pos_dist, double ang_dist);
  bool setNonBlockSpeed(double tcp, double ori);

  // Robot State
  double curSpd[2];
  int curVacuum;
  int curZone;
  Vec curToolP;
  Quaternion curToolQ;
  int curToolM;
  Vec curToolCG;
  Vec curToolI;
  Vec curWorkP;
  Quaternion curWorkQ;

  // Robot Position and Force Information
  Vec curP;
  Quaternion curQ;
  double curJ[NUM_JOINTS];
  double curForce[NUM_FORCES];
};
