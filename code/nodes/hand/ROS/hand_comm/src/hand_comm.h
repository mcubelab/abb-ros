#ifndef HAND_COMM_H
#define HAND_COMM_H

#include <ros/ros.h>

#include <hand_comm/hand_Ping.h>
#include <hand_comm/hand_Calibrate.h>
#include <hand_comm/hand_GetAngles.h>
#include <hand_comm/hand_IsMoving.h>
#include <hand_comm/hand_SetEncoder.h>
#include <hand_comm/hand_SetRest.h>
#include <hand_comm/hand_WaitRest.h>
#include <hand_comm/hand_GetEncoders.h>
#include <hand_comm/hand_GetRawForces.h>
#include <hand_comm/hand_GetForces.h>
#include <hand_comm/hand_SetAngle.h>
#include <hand_comm/hand_SetForce.h>
#include <hand_comm/hand_SetSpeed.h>

#include <hand_comm/hand_EncodersLog.h>
#include <hand_comm/hand_AnglesLog.h>
#include <hand_comm/hand_RawForcesLog.h>
#include <hand_comm/hand_ForcesLog.h>

#include <hand_comm/hand_DADA.h>

#include <matVec/matVec.h>

#include <geometry_msgs/Pose.h>

#define NUM_FINGERS 3
#define NUM_RAW_HAND_FORCES 3
#define NUM_HAND_FORCES 3

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
        void (*funcPtr)(const hand_comm::hand_AnglesLogConstPtr&));
    void subscribeEncoders(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const hand_comm::hand_EncodersLogConstPtr&));
    void subscribeRawForces(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const hand_comm::hand_RawForcesLogConstPtr&));
    void subscribeForces(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const hand_comm::hand_ForcesLogConstPtr&));

    // Shutdown service clients
    void shutdown();

    bool Ping();
    bool Calibrate(bool fast=0);
    bool GetAngles(double &mot_ang, double ang[NUM_FINGERS]);
    bool GetAngles(double &mot_ang, Vec &ang);
    bool IsMoving(bool &moving);
    bool SetEncoder(int enc);
    bool SetRest();
    bool WaitRest(double delay);
    bool GetEncoders(int &mot_enc, int enc[NUM_FINGERS]);
    bool GetEncoders(int &mot_enc, Vec &enc);
    bool GetRawForces(int forces[NUM_RAW_HAND_FORCES]);
    bool GetRawForces(Vec &forces);
    bool GetForces(double forces[NUM_HAND_FORCES]);
    bool GetForces(Vec &forces);
    bool SetAngle(double angle);
    bool SetForce(double force);
    bool SetSpeed(double speed);
    bool DADA(double delay1, double angle1, double delay2, double angle2);

    bool closeHand();
    bool openHand();
    bool setup(double speed, double force);

  private:
    // Subscribers
    ros::Subscriber hand_angles_sub;
    ros::Subscriber hand_encoders_sub;
    ros::Subscriber hand_raw_forces_sub;
    ros::Subscriber hand_forces_sub;

    // ROS Service Clients
    ros::ServiceClient handle_hand_Ping;
    ros::ServiceClient handle_hand_Calibrate;
    ros::ServiceClient handle_hand_GetAngles;
    ros::ServiceClient handle_hand_IsMoving;
    ros::ServiceClient handle_hand_SetEncoder;
    ros::ServiceClient handle_hand_SetRest;
    ros::ServiceClient handle_hand_WaitRest;
    ros::ServiceClient handle_hand_GetEncoders;
    ros::ServiceClient handle_hand_GetRawForces;
    ros::ServiceClient handle_hand_GetForces;
    ros::ServiceClient handle_hand_SetAngle;
    ros::ServiceClient handle_hand_SetForce;
    ros::ServiceClient handle_hand_SetSpeed;
    ros::ServiceClient handle_hand_DADA;

    // ROS services
    hand_comm::hand_Ping hand_Ping_srv;
    hand_comm::hand_Calibrate hand_Calibrate_srv;
    hand_comm::hand_GetAngles hand_GetAngles_srv;
    hand_comm::hand_IsMoving hand_IsMoving_srv;
    hand_comm::hand_SetEncoder hand_SetEncoder_srv;
    hand_comm::hand_SetRest hand_SetRest_srv;
    hand_comm::hand_WaitRest hand_WaitRest_srv;
    hand_comm::hand_GetEncoders hand_GetEncoders_srv;
    hand_comm::hand_GetRawForces hand_GetRawForces_srv;
    hand_comm::hand_GetForces hand_GetForces_srv;
    hand_comm::hand_SetAngle hand_SetAngle_srv;
    hand_comm::hand_SetForce hand_SetForce_srv;
    hand_comm::hand_SetSpeed hand_SetSpeed_srv;
    hand_comm::hand_DADA hand_DADA_srv;
};
#endif //HAND_COMM_H
