#ifndef TABLEVISION_NODE_H
#define TABLEVISION_NODE_H

#include <ros/ros.h>
#include <tableVision_comm/tableVision_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <vector>
#include <string>

#include <SerialStream.h>
#include <iostream>

using namespace LibSerial;

class TableVision
{
  public:
    static const double REC_TCP = 100.0;
    static const double REC_ORI = 40.0;
  
  public:
    TableVision();
    TableVision(ros::NodeHandle *n);
    virtual ~TableVision();

    bool init(ros::NodeHandle *n);

    bool tableVision_GetPose(tableVision_comm::tableVision_GetPose::Request& req, 
        tableVision_comm::tableVision_GetPose::Response& res);
    bool tableVision_GetObjAndPose(tableVision_comm::tableVision_GetObjAndPose::Request& req, 
        tableVision_comm::tableVision_GetObjAndPose::Response& res);
    bool tableVision_SavePoints(tableVision_comm::tableVision_SavePoints::Request& req, 
        tableVision_comm::tableVision_SavePoints::Response& res);
    bool tableVision_GetHandPoseSpec(tableVision_comm::tableVision_GetHandPoseSpec::Request& req, 
        tableVision_comm::tableVision_GetHandPoseSpec::Response& res);
    bool tableVision_GetHandPose(tableVision_comm::tableVision_GetHandPose::Request& req, 
        tableVision_comm::tableVision_GetHandPose::Response& res);



  private:

    bool getTablePose(int &obj, HomogTransf &objPose, std::string &filename);
    bool getHandPose(int &obj, HomogTransf &objPose, std::string &filename, const HomogTransf &handPose);

    // Private Variables

    // Pointer to this ROS node
    ros::NodeHandle *node;    

    // Communication object for moving the robot
    RobotComm robot;

    // Communication object for getting finger angles
    HandComm hand;

    // Communication object for object recognition
    ObjRecComm objRec;

    // Communication with servos blocking kinect projectors
    SerialStream serialPort;


    // DEFAULT CONSTANTS, READ IN FROM PARAMETERS

    // Expected serial port address
    std::string serialPortAddress;

    // Names of kinects to use when looking at the table
    int numCameras;
    std::vector<std::string> cameraNames;

    int numTableCameras;
    std::vector<std::string> tableCameraNames;

    int numHandCameras;
    std::vector<std::string> handCameraNames;


    // The servo number corresponding to each camera
    std::vector<int> servoNums;

    // Boundary box of table
    HomogTransf tableBoxOffset;
    double tableBoxWidths[3];

    // Any areas we wish to ignore within the boundary box
    std::vector<HomogTransf> ignoreOffsets;
    std::vector<double> ignoreWidths;



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

    HomogTransf tableIgnoreOffset;
    double tableIgnoreWidths[3];

    // Pose to move the robot to when taking images with the kinects
    HomogTransf defaultHandPose;

    ros::ServiceServer handle_tableVision_GetPose;
    ros::ServiceServer handle_tableVision_GetHandPose;
    ros::ServiceServer handle_tableVision_GetHandPoseSpec;
    ros::ServiceServer handle_tableVision_GetObjAndPose;
    ros::ServiceServer handle_tableVision_SavePoints;
};

#endif // HANDREC_NODE_H
