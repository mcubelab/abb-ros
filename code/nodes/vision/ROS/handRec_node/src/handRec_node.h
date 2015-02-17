#ifndef HANDREC_NODE_H
#define HANDREC_NODE_H

#include <ros/ros.h>
#include <handRec_comm/handRec_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>
#include <string>


/*
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
    static const double lfingerx = 0.055;//0.025; //0.05;
    static const double lfingery = 0.025;//0.025;
    static const double lfingerz = 0.02;//0.02;
    static const double tfingerx = 0.025; //TODO
    static const double tfingery = 0.025; //TODO
    static const double tfingerz = 0.02; //TODO
    static const double fingerWidths[3*numBoxesPerFinger*numFingers] = {lfingerx,lfingery,lfingerz,
      lfingerx,lfingery,lfingerz,
      lfingerx,lfingery,lfingerz};
    //static const double fingerWidths[numBoxesPerFinger] = {lfingerx,lfingery,lfingerz,lfingerx,lfingery,lfingerz,lfingerx,lfingery,lfingerz,tfingerx,tfingery,tfingerz,tfingerx,tfingery,tfingerz,tfingerx,tfingery,tfingerz};
    static const double palmWidths[3] = {0.15,0.15,0.03};

    //static const double fingerWidths[3] = {0.01, 0.01, 0.02};

    static const HomogTransf objRecPoses[NUM_HANDREC_VIEWS] =
    {
      HomogTransf(Quaternion("0.0 -0.2588 0.9659 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
      HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
      HomogTransf(Quaternion("0.0 0.9659 -0.2588 0.0").getRotMat(), Vec("500.0 300.0 460.0", 3)),
      HomogTransf(Quaternion("0.5 -0.5 -0.5 0.5").getRotMat(), Vec("530.0 300.0 460.0", 3))
    };

    static const std::vector<HomogTransf> handRecPoses(objRecPoses, objRecPoses + NUM_HANDREC_VIEWS);

    static const double objRecFinalJ[NUM_JOINTS] = {
      9.74,
      22.93,
      12.85,
      -84.09,
      98.02,
      36.2
    };



*/

static const double objRecFinalJ[NUM_JOINTS] = {
  9.74,
  22.93,
  12.85,
  -84.09,
  98.02,
  36.2
};

class HandRecognition
{
  public:
    static const double REC_TCP = 100.0;
    static const double REC_ORI = 40.0;
  public:
    HandRecognition();
    HandRecognition(ros::NodeHandle *n);
    virtual ~HandRecognition();

    bool init(ros::NodeHandle *n);

    bool handRec_GetPose(handRec_comm::handRec_GetPose::Request& req, 
        handRec_comm::handRec_GetPose::Response& res);
    bool handRec_GetPoseSpec(handRec_comm::handRec_GetPoseSpec::Request& req, 
        handRec_comm::handRec_GetPoseSpec::Response& res);
    bool handRec_SetPrefOrient(handRec_comm::handRec_SetPrefOrient::Request& req, 
        handRec_comm::handRec_SetPrefOrient::Response& res);
    bool handRec_SetGuess(handRec_comm::handRec_SetGuess::Request& req, 
        handRec_comm::handRec_SetGuess::Response& res);
    bool handRec_SetObject(handRec_comm::handRec_SetObject::Request& req, 
        handRec_comm::handRec_SetObject::Response& res);

  private:

    //////////////////////////////////////////////////////////////////////
    // Private Functions

    // This function takes in a desired set of robot poses, sets up
    // boundaries and ignore regions for the hand, moves the robot to those
    // positions and takes depth images, and then calls a service to
    // analyze the images to determine the pose of an object
    bool getPose(const std::vector<HomogTransf> &poses, int &obj, HomogTransf &handPose, std::vector<std::string> &filenames);

    //////////////////////////////////////////////////////
    // Private Variables

    // Pointer to this ROS node
    ros::NodeHandle *node;    

    // Communication objects for object recognition, 
    //  robot, and hand nodes, which we need
    ObjRecComm objRec;
    RobotComm robot;
    HandComm hand;

    // Preferred orientation of object with respect to the hand
    Quaternion pref_orient;
    bool use_pref_orient;

    // Guess of where we think the object is in the hand
    HomogTransf guessPose;
    bool use_guess;


    // DEFAULT CONSTANTS, READ IN FROM PARAMETERS

    // Offset from hand TCP to center of boundary box
    HomogTransf handBoxOffset;
    double handBoxWidths[3];

    // Offset from hand TCP to center of palm ignore box
    HomogTransf palmBoxOffset;
    double palmBoxWidths[3];

    // Offset to each finger axis
    std::vector<HomogTransf> fingerAxes;
    // Offset from finger axis to center of finger ignore box(es)
    std::vector<HomogTransf> fingerBoxOffsets;
    std::vector<std::vector<double> > fingerBoxWidths;

    // Poses to move the robot to when taking images with the kinect
    std::vector<HomogTransf> defaultRecPoses;

    ros::ServiceServer handle_handRec_GetPose;
    ros::ServiceServer handle_handRec_GetPoseSpec;
    ros::ServiceServer handle_handRec_SetPrefOrient;
    ros::ServiceServer handle_handRec_SetGuess;
    ros::ServiceServer handle_handRec_SetObject;
};

#endif // HANDREC_NODE_H
