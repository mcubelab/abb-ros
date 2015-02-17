#include "hand_comm.h"

HandComm::HandComm()
{
}

HandComm::HandComm(ros::NodeHandle* np)
{
  subscribe(np);
}

HandComm::~HandComm()
{
  shutdown();
}

void HandComm::subscribe(ros::NodeHandle* np)
{
  handle_hand_Ping = 
    np->serviceClient<hand_comm::hand_Ping>("hand_Ping");
  handle_hand_Calibrate = 
    np->serviceClient<hand_comm::hand_Calibrate>("hand_Calibrate");
  handle_hand_GetAngles = 
    np->serviceClient<hand_comm::hand_GetAngles>("hand_GetAngles");
  handle_hand_IsMoving = 
    np->serviceClient<hand_comm::hand_IsMoving>("hand_IsMoving");
  handle_hand_SetEncoder = 
    np->serviceClient<hand_comm::hand_SetEncoder>("hand_SetEncoder");
  handle_hand_SetRest = 
    np->serviceClient<hand_comm::hand_SetRest>("hand_SetRest");
  handle_hand_WaitRest = 
    np->serviceClient<hand_comm::hand_WaitRest>("hand_WaitRest");
  handle_hand_GetEncoders = 
    np->serviceClient<hand_comm::hand_GetEncoders>("hand_GetEncoders");
  handle_hand_GetRawForces = 
    np->serviceClient<hand_comm::hand_GetRawForces>("hand_GetRawForces");
  handle_hand_GetForces = 
    np->serviceClient<hand_comm::hand_GetForces>("hand_GetForces");
  handle_hand_SetAngle = 
    np->serviceClient<hand_comm::hand_SetAngle>("hand_SetAngle");
  handle_hand_SetForce = 
    np->serviceClient<hand_comm::hand_SetForce>("hand_SetForce");
  handle_hand_SetSpeed = 
    np->serviceClient<hand_comm::hand_SetSpeed>("hand_SetSpeed");
  handle_hand_DADA = 
    np->serviceClient<hand_comm::hand_DADA>("hand_DADA");
}


void HandComm::subscribeAngles(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const hand_comm::hand_AnglesLogConstPtr&))
{
  hand_angles_sub = np->subscribe("hand_AnglesLog", q_len, funcPtr);
}
void HandComm::subscribeEncoders(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const hand_comm::hand_EncodersLogConstPtr&))
{
  hand_encoders_sub = np->subscribe("hand_EncodersLog", q_len, funcPtr);
}
void HandComm::subscribeRawForces(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const hand_comm::hand_RawForcesLogConstPtr&))
{
  hand_raw_forces_sub = np->subscribe("hand_RawForcesLog", q_len, funcPtr);
}
void HandComm::subscribeForces(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const hand_comm::hand_ForcesLogConstPtr&))
{
  hand_forces_sub = np->subscribe("hand_ForcesLog", q_len, funcPtr);
}


void HandComm::shutdown()
{
  handle_hand_Ping.shutdown();
  handle_hand_Calibrate.shutdown();
  handle_hand_GetAngles.shutdown();
  handle_hand_IsMoving.shutdown();
  handle_hand_SetEncoder.shutdown();
  handle_hand_SetRest.shutdown();
  handle_hand_WaitRest.shutdown();
  handle_hand_GetEncoders.shutdown();
  handle_hand_GetRawForces.shutdown();
  handle_hand_GetForces.shutdown();
  handle_hand_SetAngle.shutdown();
  handle_hand_SetForce.shutdown();
  handle_hand_SetSpeed.shutdown();
  handle_hand_DADA.shutdown();
}

bool HandComm::Ping()
{
  return handle_hand_Ping.call(hand_Ping_srv);
}

bool HandComm::Calibrate(bool fast)
{
  hand_Calibrate_srv.request.fast = fast;
  return handle_hand_Calibrate.call(hand_Calibrate_srv);
}

bool HandComm::GetAngles(double &mot_ang, double ang[NUM_FINGERS])
{
  if (handle_hand_GetAngles.call(hand_GetAngles_srv))
  {
    mot_ang = hand_GetAngles_srv.response.angleMotor;
    for (int i=0; i < NUM_FINGERS; i++)
      ang[i] = hand_GetAngles_srv.response.angle[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetAngles(double &mot_ang, Vec &ang)
{
  if (handle_hand_GetAngles.call(hand_GetAngles_srv))
  {
    mot_ang = hand_GetAngles_srv.response.angleMotor;
    for (int i=0; i < NUM_FINGERS; i++)
      ang[i] = hand_GetAngles_srv.response.angle[i];
    return true;
  }
  else
    return false;
}

bool HandComm::IsMoving(bool &moving)
{
  if (handle_hand_IsMoving.call(hand_IsMoving_srv))
  {
    moving = hand_IsMoving_srv.response.moving;
    return true;
  }
  else
    return false;
}

bool HandComm::SetEncoder(int enc)
{
  hand_SetEncoder_srv.request.enc = enc;
  return handle_hand_SetEncoder.call(hand_SetEncoder_srv);
}

bool HandComm::SetRest()
{
  return handle_hand_SetRest.call(hand_SetRest_srv);
}

bool HandComm::WaitRest(double delay)
{
  hand_WaitRest_srv.request.delay = delay;
  return handle_hand_WaitRest.call(hand_WaitRest_srv);
}

bool HandComm::GetEncoders(int &mot_enc, int enc[NUM_FINGERS])
{
  if (handle_hand_GetEncoders.call(hand_GetEncoders_srv))
  {
    mot_enc = hand_GetEncoders_srv.response.encMotor;
    for (int i=0; i < NUM_FINGERS; i++)
      enc[i] = hand_GetEncoders_srv.response.encFinger[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetEncoders(int &mot_enc, Vec &enc)
{
  if (handle_hand_GetEncoders.call(hand_GetEncoders_srv))
  {
    mot_enc = hand_GetEncoders_srv.response.encMotor;
    for (int i=0; i < NUM_FINGERS; i++)
      enc[i] = hand_GetEncoders_srv.response.encFinger[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetRawForces(int forces[NUM_RAW_HAND_FORCES])
{
  if (handle_hand_GetRawForces.call(hand_GetRawForces_srv))
  {
    for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
      forces[i] = hand_GetRawForces_srv.response.forces[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetRawForces(Vec &forces)
{
  if (handle_hand_GetRawForces.call(hand_GetRawForces_srv))
  {
    for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
      forces[i] = hand_GetRawForces_srv.response.forces[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetForces(double forces[NUM_HAND_FORCES])
{
  if (handle_hand_GetForces.call(hand_GetForces_srv))
  {
    for (int i=0; i < NUM_HAND_FORCES; i++)
      forces[i] = hand_GetForces_srv.response.forces[i];
    return true;
  }
  else
    return false;
}

bool HandComm::GetForces(Vec &forces)
{
  if (handle_hand_GetForces.call(hand_GetForces_srv))
  {
    for (int i=0; i < NUM_HAND_FORCES; i++)
      forces[i] = hand_GetForces_srv.response.forces[i];
    return true;
  }
  else
    return false;
}



bool HandComm::SetAngle(double angle)
{
  hand_SetAngle_srv.request.angle = angle;
  return handle_hand_SetAngle.call(hand_SetAngle_srv);
}

bool HandComm::SetForce(double force)
{
  hand_SetForce_srv.request.force = force;
  return handle_hand_SetForce.call(hand_SetForce_srv);
}

bool HandComm::SetSpeed(double speed)
{
  hand_SetSpeed_srv.request.speed = speed;
  return handle_hand_SetSpeed.call(hand_SetSpeed_srv);
}

bool HandComm::closeHand()
{
  HandComm::SetAngle(80.0); // Standard angle to close
  HandComm::WaitRest(1);

  return true;
}

bool HandComm::openHand()
{
  HandComm::SetAngle(10.0); // Standard angle to open
  HandComm::WaitRest(1);

  return true;
}

bool HandComm::setup(double speed, double force)
{
  HandComm::Calibrate(0);
  HandComm::SetSpeed(speed);
  HandComm::SetForce(force);

  return true;
}

bool HandComm::DADA(double delay1, double angle1, double delay2, double angle2)
{
  hand_DADA_srv.request.delay1 = delay1;
  hand_DADA_srv.request.angle1 = angle1;
  hand_DADA_srv.request.delay2 = delay2;
  hand_DADA_srv.request.angle2 = angle2;
  return handle_hand_DADA.call(hand_DADA_srv);
}
