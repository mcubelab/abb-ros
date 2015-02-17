#include "regrasp_comm.h"

RegraspComm::RegraspComm()
{
}

RegraspComm::RegraspComm(ros::NodeHandle* np)
{
  subscribe(np);
}
/*
RegraspComm::~RegraspComm()
{
  shutdown();
}

void HandComm::shutdown()
{
  handle_regrasp_Ping.shutdown();
  handle_regrasp_Initialize.shutdown();
  handle_regrasp_Rotate.shutdown();
  handle_regrasp_FreeDrop.shutdown();
  handle_regrasp_FramePick.shutdown();
  handle_regrasp_FrameDrop.shutdown();
}
*/

void RegraspComm::subscribe(ros::NodeHandle* np)
{
  handle_regrasp_Ping = 
    np->serviceClient<regrasp_comm::regrasp_Ping>("regrasp_Ping");
  handle_regrasp_Initialize = 
    np->serviceClient<regrasp_comm::regrasp_Initialize>("regrasp_Initialize");
  handle_regrasp_Rotate = 
    np->serviceClient<regrasp_comm::regrasp_Rotate>("regrasp_Rotate");
  handle_regrasp_FreeDrop = 
    np->serviceClient<regrasp_comm::regrasp_FreeDrop>("regrasp_FreeDrop");
  handle_regrasp_FramePick = 
    np->serviceClient<regrasp_comm::regrasp_FramePick>("regrasp_FramePick");
  handle_regrasp_FrameDrop = 
    np->serviceClient<regrasp_comm::regrasp_FrameDrop>("regrasp_FrameDrop");
  handle_regrasp_StandtoLie = 
    np->serviceClient<regrasp_comm::regrasp_StandtoLie>("regrasp_StandtoLie");
  handle_regrasp_LietoStand = 
    np->serviceClient<regrasp_comm::regrasp_LietoStand>("regrasp_LietoStand");
  handle_regrasp_ThrowCatch = 
    np->serviceClient<regrasp_comm::regrasp_ThrowCatch>("regrasp_ThrowCatch");
}

bool RegraspComm::Ping()
{
    return handle_regrasp_Ping.call(regrasp_Ping_srv);
}

bool RegraspComm::Initialize()
{
    return handle_regrasp_Initialize.call(regrasp_Initialize_srv);
}

bool RegraspComm::Rotate(double rotateangle)
{
    regrasp_Rotate_srv.request.rotateangle = rotateangle;
    return handle_regrasp_Rotate.call(regrasp_Rotate_srv);
}

bool RegraspComm::FreeDrop()
{
    return handle_regrasp_FreeDrop.call(regrasp_FreeDrop_srv);
}

bool RegraspComm::FramePick()
{
    return handle_regrasp_FramePick.call(regrasp_FramePick_srv);
}

bool RegraspComm::FrameDrop()
{
    return handle_regrasp_FrameDrop.call(regrasp_FrameDrop_srv);
}

bool RegraspComm::StandtoLie(double x, double y, double z, double rotateangle)
{
    regrasp_StandtoLie_srv.request.rotateangle = rotateangle;
    regrasp_StandtoLie_srv.request.x = x;
    regrasp_StandtoLie_srv.request.y = y;
    regrasp_StandtoLie_srv.request.z = z;
    return handle_regrasp_StandtoLie.call(regrasp_StandtoLie_srv);
}

bool RegraspComm::LietoStand(double x, double y, double z, double rotateangle)
{
    regrasp_LietoStand_srv.request.rotateangle = rotateangle;
    regrasp_LietoStand_srv.request.x = x;
    regrasp_LietoStand_srv.request.y = y;
    regrasp_LietoStand_srv.request.z = z;
    return handle_regrasp_LietoStand.call(regrasp_LietoStand_srv);
}

bool RegraspComm::ThrowCatch(int SC_number, double Z1, double Z2, double A1, double A2, double A3)
{
    regrasp_ThrowCatch_srv.request.SC_number = SC_number;
    regrasp_ThrowCatch_srv.request.Z1 = Z1;
    regrasp_ThrowCatch_srv.request.Z2 = Z2;
    regrasp_ThrowCatch_srv.request.A1 = A1;
    regrasp_ThrowCatch_srv.request.A2 = A2;
    regrasp_ThrowCatch_srv.request.A3 = A3;
    return handle_regrasp_ThrowCatch.call(regrasp_ThrowCatch_srv);
}
