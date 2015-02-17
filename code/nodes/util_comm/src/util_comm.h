#ifndef UTIL_COMM_H
#define UTIL_COMM_H

#include <ros/ros.h>
#include <vision_comm/vision_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <hand_comm/hand_comm.h>
#include <robot_comm/robot_comm.h>
#include <logger_comm/logger_comm.h>
#include <matVec/matVec.h>

using namespace std;

// Work object location
#define U_WORK_X 808.5//500.0
#define U_WORK_Y -612.86//0.0
#define U_WORK_Z 0.59//250.0
#define U_WORK_Q0 0.7084//0.0
#define U_WORK_QX 0.0003882//0.0
#define U_WORK_QY -0.0003882//1.0
#define U_WORK_QZ 0.7058//0.0

// Tool object location
#define U_TOOL_X  0.0
#define U_TOOL_Y  0.0
#define U_TOOL_Z  105.0
#define U_TOOL_Q0 1.0
#define U_TOOL_QX 0.0
#define U_TOOL_QY 0.0
#define U_TOOL_QZ 0.0

// Move Speed and Zone to use when going to home position
#define U_TCP 200.0
#define U_ORI 150.0
#define U_ZONE 0

#define U_SLOW_TCP 100.0
#define U_SLOW_ORI 20.0
#define U_INTERP_ZONE ZONE_5
#define VIS_INTERP_LEVEL 0.95

// Open hand to this absolute location when calibrating vision
#define M_HAND_VIS 70.0

// Default speed and force to move hand at
#define M_HAND_SPEED 0.7
#define M_HAND_FORCE 0.5

// Open hand to this angle when dropping marker back into bin
#define HAND_DROP 40.0

#define VIS_CAL_SETTLE_TIME 200000


#define NUM_OBJREC_VIEWS 4

// Calibration file to use for vision system
const string vis_grasp_cal_file = "vis_cal.png";
const string vis_place_cal_file = "place_cal.png";

static const HomogTransf handBoxOffset(
    Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 100.0",3));
static const double boxWidths[3] = {0.15, 0.15, 0.15};

// Transform from TCP to finger axis
// rotation about the z axis with x pointing straight up and y pointing towards the palm center
// for the other 2 fingers: first rotate back 120 deg, then rotate y by 90 
// rot2quat(quat2rot([cos(-60) 0 0 sin(-60)])*quat2rot([sqrt(2)/2 0 -sqrt(2)/2 0]))
// rot2quat(quat2rot([cos(60) 0 0 sin(60)])*quat2rot([sqrt(2)/2 0 -sqrt(2)/2 0]))
// finger1 >> y=-45
// finger2 >> x=45cos(30)=0.0389 y=45sin(30)=0.0225 
// finger3 >> x=-45cos(30)=-0.0389 y=45sin(30)=0.0225
// z offset from palm to finger axis = -15.36
static const int numFingers = 3;
static const int numBoxesPerFinger = 1; //2; TODO
static const HomogTransf fingerAxes[numFingers] = {
  HomogTransf(Quaternion("0.3536 0.6124 -0.3536 0.6124").getRotMat(),Vec("38.9 22.5 -15.36", 3)),
  HomogTransf(Quaternion("0.7071 0.0 -0.7071 0.0").getRotMat(),Vec("0.0 -45.0 -15.36", 3)),
  HomogTransf(Quaternion("-0.3536 0.6124 0.3536 0.6124").getRotMat(),Vec("-38.9 22.5 -15.36", 3))
};

// Translate to finger (center of box) pose relative to axis (should be the same for each finger)
// x=15.36mm + half_height= -8.89+(83/2)=32.61
// y=-11.37-(34.5/2)=-28.62
static const HomogTransf fingerBoxOffset[numBoxesPerFinger] = {
HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("32.44 -28.61 0.0", 3))
  //HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("40.0 -20.0 0.0", 3))
//  HomogTransf(Quaternion("-0.898 0.0 0.0 0.439").getRotMat(),Vec("40.0 -20.0 0.0", 3)) //TODO
};
static const HomogTransf palmBoxOffset[1] = {
  HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 -25.0",3))};

// half the box dims
double lfingerx = 0.055;//0.025; //0.05;
double lfingery = 0.025;//0.025;
double lfingerz = 0.02;//0.02;
double tfingerx = 0.025; //TODO
double tfingery = 0.025; //TODO
double tfingerz = 0.02; //TODO
static const double fingerWidths[3*numBoxesPerFinger*numFingers] = {lfingerx,lfingery,lfingerz,
						       lfingerx,lfingery,lfingerz,
						       lfingerx,lfingery,lfingerz};


//static const double fingerWidths[numBoxesPerFinger] = {lfingerx,lfingery,lfingerz,lfingerx,lfingery,lfingerz,lfingerx,lfingery,lfingerz,tfingerx,tfingery,tfingerz,tfingerx,tfingery,tfingerz,tfingerx,tfingery,tfingerz};
static const double palmWidths[3] = {0.15,0.15,0.03};

//static const double fingerWidths[3] = {0.01, 0.01, 0.02};

static const HomogTransf objRecPoses[NUM_OBJREC_VIEWS] =
{
  HomogTransf(Quaternion("0.0 -0.2588 0.9659 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.0 0.9659 -0.2588 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
  HomogTransf(Quaternion("0.5 -0.5 -0.5 0.5").getRotMat(), Vec("530.0 300.0 460.0", 3))
};

static const double objRecFinalJ[NUM_JOINTS] = {
  9.74,
  22.93,
  12.85,
  -84.09,
  98.02,
  36.2
};


class UtilComm
{
  public:



    UtilComm();
    UtilComm(ros::NodeHandle* np);
    ~UtilComm();

    void shutdown();

    void subscribe(ros::NodeHandle *np);

    bool random_drop();
    bool drop(double x, double y);
    bool drop();


    bool get_random_bin_pos(double &x, double &y);

    bool calibrate_vision();

    bool calibrate_hand(bool fast=0);

    bool go_home();

    bool go_test_home();

    bool go_test_action();

    bool go_home_from_test();

    bool go_home_from_vision();

    bool go_safe_vision();

    bool go_vision();

    bool go_place();

    bool getObjectInHand(int &obj, double trans[3], double quat[4]);
    bool getObjectInHand(int &obj, Vec trans, Quaternion quat);
    bool getObjectInHand(int &obj, HomogTransf &handPose);

    bool getObjectFromCamera(int &obj, HomogTransf &objPose);

    // Get the location of the object in the hand given multiple views.
    // Moves the robot to different views, saves point clouds, sets up 
    //  bounds for where the hand is in the image, and then calls a 
    //  service to calculate the object pose.
    bool getObjMulti(int &obj, double trans[3], double quat[4]);
    bool getObjMulti(int &obj, Vec trans, Quaternion quat);
    bool getObjMulti(int &obj, HomogTransf &handPose);

    bool getObjMulti(int &obj, HomogTransf &handPose, const HomogTransf poses[NUM_OBJREC_VIEWS]);

    bool getObjMulti(int &obj, HomogTransf &handPose, std::string filenames[NUM_OBJREC_VIEWS]);

    bool getObjMulti(int &obj, HomogTransf &handPose, std::string filenames[NUM_OBJREC_VIEWS], const HomogTransf poses[NUM_OBJREC_VIEWS]);

    bool getObjMultiFiles(int &obj, HomogTransf &handPose, const std::string filenames[NUM_OBJREC_VIEWS]);

    bool getObjMultiFiles(int &obj, HomogTransf &handPose, const std::string filenames[NUM_OBJREC_VIEWS], const HomogTransf poses[NUM_OBJREC_VIEWS]);

    // Set guess of object position for object recognition
    bool setGuessInHand(const HomogTransf handPose);
    bool setGuessInHand(const HomogTransf handPose, const HomogTransf robotPose);

    // Do everything getObjMulti does, but also put a 
    //  guess on where the object is in the hand.
    bool getObjMultiWithGuess(int &obj, HomogTransf &handPose, const HomogTransf handGuess);

    // Get Object on table

  private:
    bool set_defaults();

    RobotComm robot;
    HandComm hand;
    VisionComm vision;
    LoggerComm logger;
    ObjRecComm objRec;
    ros::NodeHandle* node;

    double bin_x_min, bin_x_max, bin_x_range;
    double bin_y_min, bin_y_max, bin_y_range;
    double placeJ[NUM_JOINTS];
    double visionJ[NUM_JOINTS];
    double homeJ[NUM_JOINTS];
    double homeX, homeY, homeZ;
    double homeQ0, homeQX, homeQY, homeQZ;
    double safeCameraX, safeCameraY, safeCameraZ;
    double safeCameraQ0, safeCameraQX, safeCameraQY, safeCameraQZ;
    double testHomeC[7];
    double testHomeJ[NUM_JOINTS];
    double testActionC[7];
    double testActionJ[NUM_JOINTS];

    double objRecInitJ[NUM_JOINTS];
    HomogTransf objRecInitC;

    HomogTransf cameraFrame;
};

#endif //UTIL_COMM_H
