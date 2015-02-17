#include "regrasp_test/hand_tools.h"

void HandTools::init(RobotComm r, HandComm h)
{
  ROS_INFO("initializing tools");
  robot = r;
  hand = h;
}

bool HandTools::setupHand(regrasp_test::SetHand msg)
{
  if (!robot.SetComm(BLOCKING))
    return 0;

  // Setup the arm
  robot.SetSpeed(msg.tcp, msg.ori);
  robot.SetZone(msg.zone);

  // Do the first command in joint coords for safety 
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, 90.0, 0.0);

  // init pose
  robot.SetCartesian(msg.pose.position.x, 
                     msg.pose.position.y, 
                     msg.pose.position.z, 
                     msg.pose.orientation.x, 
                     msg.pose.orientation.y, 
                     msg.pose.orientation.z, 
                     msg.pose.orientation.w);
  
  // Setup the hand
  hand.Calibrate();
  hand.WaitRest(1.0);
  hand.SetSpeed(msg.hand_speed);
  hand.SetForce(msg.hand_force);

  return true;
}

bool HandTools::moveArm(geometry_msgs::Pose pose)
{
  ROS_INFO("in move arm");
  robot.SetCartesian(pose.position.x, pose.position.y, pose.position.z, 
                     pose.orientation.x, pose.orientation.y, 
                     pose.orientation.z, pose.orientation.w);
  
  return true;
}

bool HandTools::returnToCenter(void)
{
  geometry_msgs::Pose pose;
  pose.position.x = 600;
  pose.position.y = 300;
  pose.position.z = 200;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.0;
  moveArm(pose);

  return true;
}

bool HandTools::moveArm(float joints)
{
  // robot.SetJoints(joints[0], joints[1], joints[2], 
  //                 joints[3], joints[4], joints[05], joints[6]);
  
  return true;
}

// check quaternion type is x,y,z,w (not w,x,y,z)
bool HandTools::closeHand(double close_angle)
{
  ROS_INFO("Hand Closing");
  hand.SetAngle(close_angle);
  hand.WaitRest(1);
  
  return true;
}


bool HandTools::openHand(double open_angle)
{
  ROS_INFO("Hand Opening");
  hand.SetAngle(open_angle);
  hand.WaitRest(1);
  
  return true;
}


bool HandTools::vibrate(double open_angle, double close_angle, double v0, 
             double v1, double v2, double v3, double v4, double v5)
{
  ROS_INFO("Object vibrating");
  hand.SetAngle(open_angle);
  hand.WaitRest(1);
  
  usleep(2000000);
  
  robot.SpecialCommand(v0,v1,v2,v3,v4,v5);
  
  hand.SetAngle(close_angle);
  hand.WaitRest(1);
  
  return true;
}

bool HandTools::squeeze()
{
  
  return true;
}

// in setJoints can you only flip along vertical axis?
bool HandTools::invertHand(void)
{
  // 0: invert along x axis, 1: y, 2: z
  ROS_INFO("inverting hand");
  robot.SetJoints(0.0, 0.0, 0.0, 0.0, -90.0, 0.0);
  
  return true;
}

bool HandTools::moveHandToTop(void)
{
  ROS_INFO("moving hand to top");
  robot.SetCartesian(600, 700, 1100, 0.7071, 0.0, 0.0, -0.7071);   //this is the topmost position
  usleep(2000000);
  
  return true;
}

bool HandTools::calcMinFingerAngle(double& min_angle)
{
  double motorAngle;
  double fingerAngle[3];
  hand.GetAngles(motorAngle, fingerAngle);

  min_angle=fingerAngle[0];
  if (fingerAngle[1]<min_angle)
  {
    min_angle=fingerAngle[1];
  }
  if (fingerAngle[2]<min_angle)
  {
    min_angle=fingerAngle[2];
  }
  ROS_INFO("min finger angle:%d", min_angle);
  //min_angle = std::min(fingerAngle[0],fingerAngle[1],fingerAngle[2]);
  
  return true;
}

bool HandTools::calcRelease(double& drop_angle)
{
  // RotMat Xrot;
  // Vec rotXAxis = Vec("1.0 0.0 0.0", 3);
  // double rotXAngle = -90;      //Angle to touch the fixture
  // Xrot.setAxisAngle(rotXAxis, rotXAngle*PI/180.0);
  // HomogTransf TRotX=HomogTransf(Xrot, Vec("0.0 0.0 0.0",3));
    
  // RotMat Yrot;
  // Vec rotYAxis = Vec("0.0 1.0 0.0", 3);
  // double rotYAngle = 180;      //Angle to touch the fixture
  // Yrot.setAxisAngle(rotYAxis, rotYAngle*PI/180.0);
  // HomogTransf TRotY=HomogTransf(Yrot, Vec("0.0 0.0 0.0",3));
  
  // HomogTransf TPos=HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), transSafe); 
  // HomogTransf Cdown=HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), transSafe);
  
  // calcMinFingerAngle(drop_angle);    
    
  return true;
}

