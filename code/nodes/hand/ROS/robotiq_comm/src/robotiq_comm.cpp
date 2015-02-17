#include "robotiq_comm.h"

RobotiqComm::RobotiqComm()
{
}

RobotiqComm::RobotiqComm(ros::NodeHandle* np)
{
  subscribe(np);
}

RobotiqComm::~RobotiqComm()
{
  shutdown();
}

void RobotiqComm::subscribe(ros::NodeHandle* np)
{
  // Set up our publisher, which will be used to send the hand commands
  robotiq_output_pub = np->advertise<robotiq_c_model_control::CModel_robot_output>("CModelRobotOutput", 5);


  // Setup what our output message will look like
  curOutput.rACT = 0;
  curOutput.rGTO = 0;
  curOutput.rATR = 0;
  curOutput.rPR = ROBOTIQ_DEFAULT_POS;
  curOutput.rSP = ROBOTIQ_DEFAULT_SPD;
  curOutput.rFR = ROBOTIQ_DEFAULT_FORCE;

  // Subscribe to the robotiq input topic so we can read its status when we need to
  robotiq_input_sub = np->subscribe("CModelRobotInput", 10, &RobotiqComm::saveMessage, this);

  got_message = false;
  pthread_mutex_init(&inputMutex, NULL);
}


void RobotiqComm::subscribeInput(ros::NodeHandle* np, int q_len, 
    void (*funcPtr)(const robotiq_c_model_control::CModel_robot_inputConstPtr&))
{
  robotiq_input_sub = np->subscribe("CModelRobotInput", q_len, funcPtr);
}

void RobotiqComm::interpretStatus(const robotiq_c_model_control::CModel_robot_input msg, bool &active, bool &position_req, char &activation_status, char &motion_status, char &fault, int &req_pos, int &pos, int &current)
{
  active = msg.gACT;
  position_req = msg.gGTO;
  activation_status = msg.gSTA;
  motion_status = msg.gOBJ;
  fault = msg.gFLT;
  req_pos = msg.gPR;
  pos = msg.gPO;
  current = msg.gCU * 10;
}



void RobotiqComm::shutdown()
{
  robotiq_output_pub.shutdown();

  robotiq_input_sub.shutdown();
  pthread_mutex_destroy(&inputMutex);
}

bool RobotiqComm::Ping()
{
  return got_message;
}

bool RobotiqComm::GetStatus(bool &active, bool &position_req, char &activation_status, char &motion_status, char &fault, int &req_pos, int &pos, int &current)
{
  robotiq_c_model_control::CModel_robot_input msg;

  ros::Rate loop_rate(10);
  got_message = false;
  while(!got_message && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  pthread_mutex_lock(&inputMutex);
  msg = last_message;
  pthread_mutex_unlock(&inputMutex);

  interpretStatus(msg, active, position_req, activation_status, motion_status, fault, req_pos, pos, current);
  return true;
}

bool RobotiqComm::GetPose(int &pos)
{
  ros::Rate loop_rate(10);
  got_message = false;
  while(!got_message && ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  pthread_mutex_lock(&inputMutex);
  pos = last_message.gPO;
  pthread_mutex_unlock(&inputMutex);
  return true;
}

bool RobotiqComm::GetCurrent(int &current)
{
  ros::Rate loop_rate(10);
  got_message = false;
  while(!got_message)
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  pthread_mutex_lock(&inputMutex);
  current = last_message.gCU * 10;
  pthread_mutex_unlock(&inputMutex);
  return true;
}

bool RobotiqComm::Disable()
{
  curOutput.rACT = 0;
  curOutput.rGTO = 0;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::Enable()
{
  curOutput.rACT = 1;
  curOutput.rATR = 0;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}


bool RobotiqComm::AutoRelease()
{
  curOutput.rATR = 1;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::Open()
{
  curOutput.rGTO = 1;
  curOutput.rPR = 0;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::Close()
{
  curOutput.rGTO = 1;
  curOutput.rPR = 255;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::Stop()
{
  curOutput.rGTO = 0;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::SetPose(int pose)
{
  if (pose > 255) pose = 255;
  if (pose < 0) pose = 0;
  curOutput.rGTO = 1;
  curOutput.rPR = pose;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::SetForce(int force)
{
  if (force > 255) force = 255;
  if (force < 0) force = 0;
  curOutput.rFR = force;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}

bool RobotiqComm::SetSpeed(int speed)
{
  if (speed > 255) speed = 255;
  if (speed < 0) speed = 0;
  curOutput.rSP = speed;
  robotiq_output_pub.publish(curOutput);
  ros::spinOnce();
  return true;
}


bool RobotiqComm::WaitRest(double delay)
{
  ros::Duration(delay).sleep();
  ros::Rate loop_rate(10);
  bool moving = true;
  while (moving && ros::ok())
  {
    ros::spinOnce();
    pthread_mutex_lock(&inputMutex);
    moving = (last_message.gGTO == 1 && last_message.gOBJ == 0x0);
    pthread_mutex_unlock(&inputMutex);
    loop_rate.sleep();
  }
  return true;
}

void RobotiqComm::saveMessage(const robotiq_c_model_control::CModel_robot_inputConstPtr& msg)
{
  got_message = true;
  pthread_mutex_lock(&inputMutex);
  last_message = *msg;
  pthread_mutex_unlock(&inputMutex);
}
