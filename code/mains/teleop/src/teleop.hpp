#include <ros/ros.h>
#include <matVec/matVec.h>
#include <ncurses.h>
#include <string.h>
#include <cstdio>

#include <robot_comm/robot_comm.h>
#include <logger_comm/logger_comm.h>
#include <hand_comm/hand_comm.h>

#include <string>
using namespace std;

#include "teleop_console.hpp"

// Step size for cartesian moves (mm)
#define CART_STEP 50.0
// Step size for cartesian rotations (deg -> rad)
#define R_STEP (1.0 * DEG2RAD)
// Step size for joint moves (deg)
#define J_STEP 10.0
// Step size for hand (deg)
#define H_STEP 1.0
// Step size for TCP and ORI speed
#define TCP_STEP 1.0
#define ORI_STEP 1.0

// Initial Defaults for Work Object, Tool, Home, and Speed
//Work object
#define DEF_WORK_X 500.0
#define DEF_WORK_Y 0.0
#define DEF_WORK_Z 250.0
#define DEF_WORK_Q0 0.0
#define DEF_WORK_QX 0.0
#define DEF_WORK_QY 1.0
#define DEF_WORK_QZ 0.0

//Center of the palm of the hand
#define DEF_TOOL_X  0.0
#define DEF_TOOL_Y  0.0
#define DEF_TOOL_Z  105.0
#define DEF_TOOL_Q0 1.0
#define DEF_TOOL_QX 0.0
#define DEF_TOOL_QY 0.0
#define DEF_TOOL_QZ 0.0

//Home Joints
#define DEF_HOME_J1 0.0
#define DEF_HOME_J2 24.89
#define DEF_HOME_J3 17.98
#define DEF_HOME_J4 0.0
#define DEF_HOME_J5 47.12
#define DEF_HOME_J6 0.0

// Initial Speeds to use
#define DEF_TCP 50.0
#define DEF_ORI 10.0

// Zone to use
#define T_ZONE 0

// Rate to run the state machine at (Hz)
#define TELEOP_RATE 25.0

#define TELEOP_ID 3

const string teleop_log_str = "teleop_log";

string log_file;

bool initalize();
void updateCartData(const robot_comm::robot_CartesianLogConstPtr& msg);
void updateJointData(const robot_comm::robot_JointsLogConstPtr& msg);
void updateHandData(const hand_comm::hand_AnglesLogConstPtr& msg);
void updateData();
void getCommand();

RobotComm robot;
LoggerComm logger;
HandComm hand;

ros::NodeHandle *nodePtr;

double homeJ[6];
double workObj[7];
double toolObj[7];
double curSpd[2];

double curC[7];
double curJ[6];
double curH[5];


double newC[7];
double newJ[6];
double newH[5];

bool updateC, updateJ, updateH;
bool readingC, readingJ, readingH;

KEY_CMDS last_key;

Quaternion x_pos_rot;
Quaternion x_neg_rot;
Quaternion y_pos_rot;
Quaternion y_neg_rot;
Quaternion z_pos_rot;
Quaternion z_neg_rot;

bool enabled;

ros::Subscriber robot_cartesian_sub;
ros::Subscriber robot_joints_sub;
ros::Subscriber hand_angles_sub;

#define NUM_ALIGN_DIRS 6
const Vec align_vecs[NUM_ALIGN_DIRS] = 
{
  Vec("1    0   0", 3),
  Vec("0    1   0", 3),
  Vec("0    0   1", 3),
  Vec("-1   0   0", 3),
  Vec("0    -1  0", 3),
  Vec("0    0   -1",3)
};

const Quaternion align_quats[NUM_ALIGN_DIRS] =
{
  Quaternion("0.7071  0       0.7071  0"),
  Quaternion("0.7071 -0.7071  0       0"),
  Quaternion("1       0       0       0"),
  Quaternion("0.7071  0       -0.7071 0"),
  Quaternion("0.7071  0.7071  0       0"),
  Quaternion("-1      0       0       0")
};
