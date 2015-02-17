#ifndef TABLEVISION_COMM_H
#define TABLEVISION_COMM_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <cstring>

#include <matVec/matVec.h>
#include <geometry_msgs/PoseStamped.h>
#include <objRec_comm/objRec_comm.h> // Included so RecObj::Type is included in the library

#include <tableVision_comm/tableVision_GetPose.h>
#include <tableVision_comm/tableVision_GetHandPose.h>
#include <tableVision_comm/tableVision_GetHandPoseSpec.h>
#include <tableVision_comm/tableVision_GetObjAndPose.h>
#include <tableVision_comm/tableVision_SavePoints.h>


class TableVisionComm
{
  public:
    TableVisionComm();
    TableVisionComm(ros::NodeHandle* np);
    ~TableVisionComm();

    void shutdown();

    void subscribe(ros::NodeHandle *np);

    // Returns: true if object was found, false otherwise
    // Variables that are passed by reference and assigned:
    // obj: The object that was found
    // pose: The pose of the object
    bool GetPose(const int obj, HomogTransf &pose);
    bool GetObjAndPose(int &obj, HomogTransf &pose);
    bool GetHandPose(const int obj, HomogTransf &pose);
    bool GetHandPoseSpec(const int obj, const HomogTransf handPose, HomogTransf &pose);

    // The rest of these functions are different ways of accessing the same
    // information as above
    bool GetPose(const int obj, HomogTransf &pose, std::string &filename);
    bool GetPose(const int obj, double trans[3], double quat[4]);
    bool GetPose(const int obj, Vec &trans, Quaternion &quat);
    bool GetPose(const int obj, double pose[7]);
    bool GetPose(const int obj, double pose[7], std::string &filename);

    bool GetObjAndPose(int &obj, HomogTransf &pose, std::string &filename);
    bool GetObjAndPose(int &obj, double trans[3], double quat[4]);
    bool GetObjAndPose(int &obj, Vec &trans, Quaternion &quat);
    bool GetObjAndPose(int &obj, double pose[7]);
    bool GetObjAndPose(int &obj, double pose[7], std::string &filename);

    // Save an RGB-D image from the kinect. Specify whether or not you want
    // the point cloud to be binary, and the function returns the name and
    // location of the file the node saved the point cloud file to. When 
    // calling this service from tableVision, it will also move the servos 
    // so that the kinects are covered or not
    bool SavePoints(std::string &filename, const bool use_filter=false, 
        const bool is_binary=true, const std::string cameraName="");

  private:
    // ROS Shorthand
    ros::ServiceClient handle_tableVision_GetPose;
    ros::ServiceClient handle_tableVision_GetHandPose;
    ros::ServiceClient handle_tableVision_GetHandPoseSpec;
    ros::ServiceClient handle_tableVision_GetObjAndPose;
    ros::ServiceClient handle_tableVision_SavePoints;

    tableVision_comm::tableVision_GetPose tableVision_GetPose_srv;
    tableVision_comm::tableVision_GetHandPose tableVision_GetHandPose_srv;
    tableVision_comm::tableVision_GetHandPoseSpec tableVision_GetHandPoseSpec_srv;
    tableVision_comm::tableVision_GetObjAndPose tableVision_GetObjAndPose_srv;
    tableVision_comm::tableVision_SavePoints tableVision_SavePoints_srv;
};



#endif // TABLEVISION_COMM_H
