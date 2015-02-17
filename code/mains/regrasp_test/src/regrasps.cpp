#include "regrasp_test/regrasps.h"
#include <matVec/matVec.h>
#include <string>

void Regrasps::init(RobotComm r, HandComm h)
{
  robot = r;
  hand = h;

  regrasp_test::SetHand msg;
  msg.tcp = 100.0;
  msg.ori = 35.0;
  msg.zone = 0;
  msg.hand_speed = 0.99;
  msg.hand_force = 0.9;
  geometry_msgs::Pose pose;
  pose.position.x = 600;
  pose.position.y = 300;
  pose.position.z = 200;
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.0;
  msg.pose = pose;
  double joints[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  robot.setupRobot(msg.tcp, msg.ori, msg.zone, joints, msg.pose);
  hand.setup(msg.hand_speed, msg.hand_force);
}

bool Regrasps::pick(regrasp_test::Pick msg)
{
  ROS_INFO("Starting pick...");
  robot.moveArm(msg.pose);
  hand.openHand();
  robot.moveArm(msg.pose);
  hand.closeHand();
  
  robot.SetDefaults();
  ROS_INFO("Pick completed.");
  return true;
}

bool Regrasps::place(regrasp_test::Place msg)
{
  ROS_INFO("Starting placing...");
  robot.moveArm(msg.pose);
  hand.openHand();

  robot.SetDefaults();
  ROS_INFO("Place completed.");
  return true;
}

bool Regrasps::bouncelessF2E(regrasp_test::BouncelessF2E msg)
{
  ROS_INFO("Starting BouncelessF2E...");
  // move into position
  robot.invertHand();
  robot.moveToTop();

  // prepare for throw
  hand.SetSpeed(msg.throw_speed);
  double minFingerAngle = Regrasps::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle-10);
  usleep(msg.tsleep);

  // throw
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);

  // regrasp
  hand.SetSpeed(msg.grasp_speed);
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("BouncelessF2E completed.");
  return true;
}

bool Regrasps::pushTranslate(regrasp_test::PushTranslate msg)
{
  ROS_INFO("Starting PushTranslate...");
  std::vector<HomogTransf> transforms = Regrasps::calcPushTranslateTransforms(msg.safe_pose);
  HomogTransf TRotX = transforms.at(0);
  HomogTransf TRotY = transforms.at(1);
  HomogTransf TPos = transforms.at(2);
  HomogTransf Cdown = transforms.at(3);
  double minFingerAngle = Regrasps::calcMinFingerAngle();

  Vec transSafe;
  Regrasps::poseToVec(msg.safe_pose, transSafe);

  Vec transPushStop;
  Regrasps::poseToVec(msg.push_stop_pose, transPushStop);

  HomogTransf TouchPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchPos);
  hand.SetAngle(minFingerAngle-.25);
  usleep(500000);
  ROS_INFO("1 Completed!"); 
 
  HomogTransf PushStPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStPos);
  hand.closeHand();
  // usleep(500000);
  ROS_INFO("2 Completed!");

  robot.SetCartesian(TouchPos);
  ROS_INFO("3 Completed!");
  
  HomogTransf TouchInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchInvPos);
  ROS_INFO("4 Completed!");
  
  hand.SetAngle(minFingerAngle-.25);
  usleep(500000);
  
  HomogTransf PushStInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStInvPos);
  ROS_INFO("5 Completed!");
  
  hand.closeHand();
  //usleep(500000);
  
  robot.SetCartesian(TouchInvPos);
  ROS_INFO("6 Completed!");

  robot.SetDefaults();
  ROS_INFO("PushTranslate completed.");
  return true;
}

// TODO: currently only goes from palm to fingertips. goal>>roll the 
// cylinder by a specific angle (msg.roll_angle)
bool Regrasps::rollInHand(regrasp_test::RollInHand msg)
{
  ROS_INFO("Starting RollInHand...");

  double motorAngle;
  double fingerAngle[3];

  hand.GetAngles(motorAngle, fingerAngle);
  double RollmaxfingerAngle=fingerAngle[0];
  if (fingerAngle[1]>RollmaxfingerAngle)
  {
    RollmaxfingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2]>RollmaxfingerAngle)
  {
    RollmaxfingerAngle=fingerAngle[2];
  }
  double RollReleaseAngle= RollmaxfingerAngle-msg.roll_angle;
  
  hand.SetAngle(RollReleaseAngle);
  hand.WaitRest(1); 
  usleep(2000000);
  
  double HAND_CLOSEAfterRegrasp_ANGLE=RollReleaseAngle+5;
  hand.SetAngle(HAND_CLOSEAfterRegrasp_ANGLE);
  usleep(1000000);
  
  hand.openHand();
  
  robot.SetDefaults();
  ROS_INFO("RollInHand completed.");
  return true;
}

// TODO: add params for freq/geometry
bool Regrasps::vibrate(regrasp_test::Vibrate msg)
{
  ROS_INFO("Starting vibrate...");

  robot.invertHand();
  
  hand.SetAngle(msg.vibrate_angle);
  hand.WaitRest(1);
  usleep(2000000);

  robot.vibrate();

  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("Vibrate completed.");
  return true;
}

bool Regrasps::groundRoll(regrasp_test::GroundRoll msg)
{
  ROS_INFO("Starting GroundRoll...");

  robot.moveArm(msg.roll_pose);
  double rollReleaseAngle = Regrasps::calcMinFingerAngle();
  hand.SetAngle(rollReleaseAngle);
  hand.WaitRest(1);
  usleep(2000000);
  
  robot.SetSpeed(msg.tcp, msg.ori);
  robot.moveArm(msg.roll_place_pose);

  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("GroundRoll completed.");
  return true;
}

// TODO: this includes both vert to hor and vice versa. how do we want to separate?
bool Regrasps::rotateInFingers(regrasp_test::RotateInFingers msg)
{
  ROS_INFO("Starting RotateInFingers...");
  robot.SetZone(msg.zone);
  geometry_msgs::Quaternion quatPush = msg.push_pose.orientation;

  robot.SetCartesian(860,230,187,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(865,230,192,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(870,230,195,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(875,230,197,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(880,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(885,230,199,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(890,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(895,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(900,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(905,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(910,230,198,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(910,230,297,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(850,230,197,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  usleep(2000000);
  robot.SetCartesian(860,230,197,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(865,230,193,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(885,230,193,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(895,230,192,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(900,230,191,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(905,230,190,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(910,230,188.5,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(920,230,188,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  robot.SetCartesian(920,230,288,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  
  robot.SetDefaults();
  ROS_INFO("RotateInFingers completed.");
  return true;
}

bool Regrasps::topple(regrasp_test::Topple msg)
{
  ROS_INFO("Starting topple....");
  robot.invertHand();
 
  hand.openHand();

  robot.moveArm(msg.pose);
  usleep(2000000);
 
  hand.closeHand();
  usleep(msg.sleep);
  
  robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);

  robot.SetDefaults();
  ROS_INFO("Topple completed.");
  return true;
}

bool Regrasps::flipInAir(regrasp_test::FlipInAir msg)
{
  ROS_INFO("Starting FlipInAir...");
  robot.moveArm(msg.flip_pose);
  usleep(3000000);

  double minFingerAngle = Regrasps::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle - 20);
  usleep(msg.sleep);

  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("FlipInAir completed.");
  return true;
}

bool Regrasps::flipInHand(regrasp_test::FlipInHand msg)
{
  ROS_INFO("Starting FlipInHand... ");
  robot.SetJoints(-1.37, 31.99, 16.84, 0.1, -75.5, -88); // waypoint bc IK is bad
  usleep(2000000);

  double minFingerAngle = Regrasps::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle-20);
  usleep(msg.sleep);
  
  robot.SetDefaults();
  ROS_INFO("FlipInHand completed.");
  return true;
}

bool Regrasps::vertCyl(regrasp_test::VertCyl msg)
{
  ROS_INFO("Starting VertCyl...");
  robot.moveArm(msg.place_pose);
  hand.openHand();
  robot.moveArm(msg.pick_pose);
  hand.closeHand();
  
  robot.SetSpeed(msg.tcp, msg.ori);
  robot.moveArm(msg.place2_pose);
  usleep(4000000);

  hand.openHand();
  usleep(200000);

  robot.SetDefaults();
  ROS_INFO("VertCyl completed.");
  return true;
}

bool Regrasps::E2F(regrasp_test::E2F msg)
{
  ROS_INFO("Starting E2F...");
  robot.moveToTop();
  hand.SetAngle(msg.partial_open_angle);
  hand.WaitRest(0.250);

  robot.moveArm(msg.pose);
  hand.SetSpeed(msg.fspeed);
  hand.closeHand();
  usleep(msg.sleep);

  robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);

  robot.moveArm(msg.pose1);
  robot.moveArm(msg.pose2);

  hand.SetSpeed(msg.speed);
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("E2F completed.");
  return true;
}

/***********************COMPUTATION***********************/
double Regrasps::calcMinFingerAngle(void)
{
  double motorAngle;
  double fingerAngle[3];

  hand.GetAngles(motorAngle, fingerAngle);
  double minFingerAngle = fingerAngle[0];
  if (fingerAngle[1] < minFingerAngle)
    {
      minFingerAngle = fingerAngle[1];
    } 
  if (fingerAngle[2] < minFingerAngle)
    {
      minFingerAngle = fingerAngle[2];
    }

  return minFingerAngle;
}


std::vector<HomogTransf> Regrasps::calcPushTranslateTransforms(geometry_msgs::Pose safe_pose)
{
  RotMat Xrot;
  Vec rotXAxis = Vec("1.0 0.0 0.0", 3);
  double rotXAngle = -90;      //Angle to touch the fixture
  Xrot.setAxisAngle(rotXAxis, rotXAngle*PI/180.0);
  HomogTransf TRotX=HomogTransf(Xrot, Vec("0.0 0.0 0.0",3));

  RotMat Yrot;
  Vec rotYAxis = Vec("0.0 1.0 0.0", 3);
  double rotYAngle = 180;      //Angle to touch the fixture
  Yrot.setAxisAngle(rotYAxis, rotYAngle*PI/180.0);
  HomogTransf TRotY=HomogTransf(Yrot, Vec("0.0 0.0 0.0",3));
  
  Vec transSafe;
  Regrasps::poseToVec(safe_pose, transSafe);

  HomogTransf TPos=HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), transSafe); 
  HomogTransf Cdown=HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), transSafe);

  std::vector<HomogTransf> transforms;
  transforms.push_back(TRotX);
  transforms.push_back(TRotY);
  transforms.push_back(TPos);
  transforms.push_back(Cdown);

  return transforms;
}

bool Regrasps::poseToVec(geometry_msgs::Pose pose, Vec& vector)
{
  vector = Vec(3);
  vector[0] = pose.position.x;
  vector[1] = pose.position.y;
  vector[2] = pose.position.z;

  return true;
}
