#ifndef REGRASP_COMM_H
#define REGRASP_COMM_H

#include <ros/ros.h>


#include <regrasp_comm/regrasp_Ping.h>
#include <regrasp_comm/regrasp_Initialize.h>
#include <regrasp_comm/regrasp_Rotate.h>
#include <regrasp_comm/regrasp_FreeDrop.h>
#include <regrasp_comm/regrasp_FramePick.h>
#include <regrasp_comm/regrasp_FrameDrop.h>
#include <regrasp_comm/regrasp_StandtoLie.h>
#include <regrasp_comm/regrasp_LietoStand.h>
#include <regrasp_comm/regrasp_ThrowCatch.h>


#include <matVec/matVec.h>

#define NUM_FINGERS 3

class RegraspComm
{
  public:
    RegraspComm();
    RegraspComm(ros::NodeHandle* np);
    //~RegraspComm();

// Subscribe Function
    void subscribe(ros::NodeHandle* np);

    // Shutdown service clients
    //void shutdown();

    bool Ping();
    bool Initialize();
    bool Rotate(double rotateangle);
    bool FreeDrop();
    bool FramePick();
    bool FrameDrop();
    bool StandtoLie(double x, double y, double z, double rotateangle);
    bool LietoStand(double x, double y, double z, double rotateangle);
    bool ThrowCatch(int SC_number, double Z1, double Z2, double A1, double A2, double A3);

  private:
    // ROS Service Clients
    ros::ServiceClient handle_regrasp_Ping;
    ros::ServiceClient handle_regrasp_Initialize;
    ros::ServiceClient handle_regrasp_Rotate;
    ros::ServiceClient handle_regrasp_FreeDrop;
    ros::ServiceClient handle_regrasp_FramePick;
    ros::ServiceClient handle_regrasp_FrameDrop;
    ros::ServiceClient handle_regrasp_StandtoLie;
    ros::ServiceClient handle_regrasp_LietoStand;
    ros::ServiceClient handle_regrasp_ThrowCatch;

    // ROS services
    regrasp_comm::regrasp_Ping regrasp_Ping_srv;
    regrasp_comm::regrasp_Initialize regrasp_Initialize_srv;
    regrasp_comm::regrasp_Rotate regrasp_Rotate_srv;
    regrasp_comm::regrasp_FreeDrop regrasp_FreeDrop_srv;
    regrasp_comm::regrasp_FramePick regrasp_FramePick_srv;
    regrasp_comm::regrasp_FrameDrop regrasp_FrameDrop_srv;
    regrasp_comm::regrasp_StandtoLie regrasp_StandtoLie_srv;
    regrasp_comm::regrasp_LietoStand regrasp_LietoStand_srv;
    regrasp_comm::regrasp_ThrowCatch regrasp_ThrowCatch_srv;
};
#endif //REGRASP_COMM_H
