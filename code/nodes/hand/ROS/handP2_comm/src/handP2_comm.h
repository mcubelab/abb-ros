#ifndef HAND_COMM_H
#define HAND_COMM_H

#include <ros/ros.h>

#include <handP2_comm/hand_Ping.h>
#include <handP2_comm/hand_Calibrate.h>
#include <handP2_comm/hand_GetAngles.h>
#include <handP2_comm/hand_IsMoving.h>
#include <handP2_comm/hand_SetEncoder.h>
#include <handP2_comm/hand_SetRest.h>
#include <handP2_comm/hand_WaitRest.h>
#include <handP2_comm/hand_GetEncoders.h>
#include <handP2_comm/hand_SetAngle.h>
#include <handP2_comm/hand_SetForce.h>
#include <handP2_comm/hand_SetSpeed.h>

#include <handP2_comm/hand_EncodersLog.h>
#include <handP2_comm/hand_AnglesLog.h>

#include <matVec/matVec.h>

#define NUM_FINGERS 3

class HandComm
{
  public:
    HandComm();
    HandComm(ros::NodeHandle* np);
    ~HandComm();

    // Subscribe Function
    void subscribe(ros::NodeHandle* np);

    // Subscribe to Topics
    void subscribeAngles(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const handP2_comm::hand_AnglesLogConstPtr&));
    void subscribeEncoders(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const handP2_comm::hand_EncodersLogConstPtr&));

    // Shutdown service clients
    void shutdown();

    bool Ping();
    bool Calibrate(bool fast=0);
    bool GetAngles(double &mot_ang, double ang[NUM_FINGERS]);
    bool GetAngles(double &mot_ang, Vec &ang);
    bool IsMoving(bool &moving);
    bool SetEncoder(int enc);
    bool SetRest();
    bool WaitRest(int delay);
    bool GetEncoders(int &mot_enc, int enc[NUM_FINGERS]);
    bool GetEncoders(int &mot_enc, Vec &enc);
    bool SetAngle(double angle);
    bool SetForce(double force);
    bool SetSpeed(double speed);

  private:
    // Subscribers
    ros::Subscriber hand_angles_sub;
    ros::Subscriber hand_encoders_sub;

    // ROS Service Clients
    ros::ServiceClient handle_hand_Ping;
    ros::ServiceClient handle_hand_Calibrate;
    ros::ServiceClient handle_hand_GetAngles;
    ros::ServiceClient handle_hand_IsMoving;
    ros::ServiceClient handle_hand_SetEncoder;
    ros::ServiceClient handle_hand_SetRest;
    ros::ServiceClient handle_hand_WaitRest;
    ros::ServiceClient handle_hand_GetEncoders;
    ros::ServiceClient handle_hand_SetAngle;
    ros::ServiceClient handle_hand_SetForce;
    ros::ServiceClient handle_hand_SetSpeed;

    // ROS services
    handP2_comm::hand_Ping hand_Ping_srv;
    handP2_comm::hand_Calibrate hand_Calibrate_srv;
    handP2_comm::hand_GetAngles hand_GetAngles_srv;
    handP2_comm::hand_IsMoving hand_IsMoving_srv;
    handP2_comm::hand_SetEncoder hand_SetEncoder_srv;
    handP2_comm::hand_SetRest hand_SetRest_srv;
    handP2_comm::hand_WaitRest hand_WaitRest_srv;
    handP2_comm::hand_GetEncoders hand_GetEncoders_srv;
    handP2_comm::hand_SetAngle hand_SetAngle_srv;
    handP2_comm::hand_SetForce hand_SetForce_srv;
    handP2_comm::hand_SetSpeed hand_SetSpeed_srv;
};
#endif //HAND_COMM_H
