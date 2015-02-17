#ifndef HANDREC_COMM_H
#define HANDREC_COMM_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <cstring>

#include <matVec/matVec.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <objRec_comm/objRec_comm.h> // Included so RecObj::Type is included in the library

#include <handRec_comm/handRec_GetPose.h>
#include <handRec_comm/handRec_GetPoseSpec.h>
#include <handRec_comm/handRec_SetPrefOrient.h>
#include <handRec_comm/handRec_SetGuess.h>
#include <handRec_comm/handRec_SetObject.h>

class HandRecComm
{
  public:
    HandRecComm();
    HandRecComm(ros::NodeHandle* np);
    ~HandRecComm();

    void shutdown();

    void subscribe(ros::NodeHandle *np);

    // Get object pose. Moves the robot to different views, saves
    // point clouds, sets up bounds for where the hand is in the images,
    // and then calls a service to calculate the object pose. 
    // 
    // Returns: true if object was found, false otherwise
    // Variables that are passed by reference and assigned:
    // obj: The object that was found
    // handPose: The pose of the object with respect to the hand
    bool GetPose(int &obj, HomogTransf &handPose);

    // The rest of these functions are different ways of accessing the same
    // information as above
    bool GetPose(int &obj, HomogTransf &handPose, std::vector<std::string> &filenames);
    bool GetPose(int &obj, double trans[3], double quat[4]);
    bool GetPose(int &obj, Vec &trans, Quaternion &quat);
    bool GetPose(int &obj, double pose[7]);
    bool GetPose(int &obj, double pose[7], std::vector<std::string> &filenames);

    // In this function, we specify where to move the robot to. Useful for
    // accuracy evaluation
    bool GetPoseSpec(int &obj, HomogTransf &handPose, const std::vector<HomogTransf> poses, std::vector<std::string> &filenames);

    // Set the preferred orientation of the object. This is useful when the
    // object has symmetries, and we want to get consistent results
    bool SetPrefOrient(const Quaternion quat);
    bool ClearPrefOrient();

    // Set a guess of where we think the object is in our hand. Useful for
    // speeding things up if we are expecting the same pose over and over
    // again
    bool SetGuess(const HomogTransf handPose);
    bool ClearGuess();

    // Set an object preference
    bool SetObject(const int obj);
    bool ClearObject();

  private:
    // ROS Shorthand
    ros::ServiceClient handle_handRec_GetPose;
    ros::ServiceClient handle_handRec_GetPoseSpec;
    ros::ServiceClient handle_handRec_SetPrefOrient;
    ros::ServiceClient handle_handRec_SetGuess;
    ros::ServiceClient handle_handRec_SetObject;

    handRec_comm::handRec_GetPose handRec_GetPose_srv;
    handRec_comm::handRec_GetPoseSpec handRec_GetPoseSpec_srv;
    handRec_comm::handRec_SetPrefOrient handRec_SetPrefOrient_srv;
    handRec_comm::handRec_SetGuess handRec_SetGuess_srv;
    handRec_comm::handRec_SetObject handRec_SetObject_srv;
};



#endif // HANDREC_COMM_H
