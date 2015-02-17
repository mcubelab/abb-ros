#include "regraspNode/regrasp_node.h"
#include <ros/package.h>

RegraspController::RegraspController(ros::NodeHandle *n)
{
  node = n;
  RobotComm r(node);
  HandComm h(node);
  MatlabComm m;
  m.subscribe(node);
  //std::string pkg_path = ros::package::getPath("tableVision_node");
  std::string pkg_path = "~/Documents/hands/code/nodes/vision/ROS/tableVision_node";
  pkg_path += "/matlab_scripts";
  m.addPath(pkg_path.c_str());

  matlab = m;
  robot = r;
  hand = h;

  RegraspController::init();
}

RegraspController::~RegraspController() {
  handle_regrasp_Execute.shutdown();
}

void RegraspController::advertiseServices()
{
  handle_regrasp_Execute = node->advertiseService("regrasp_Execute", &RegraspController::regrasp_Execute, this);
}

bool RegraspController::regrasp_Execute(regraspComm::regrasp_Execute::Request &req, 
					regraspComm::regrasp_Execute::Response &res)
{
  ROS_INFO("IN SERVICE");
  
  bool success = true;
  for (unsigned int i=0; i<req.regrasps.size();i++)
    {
      int action = req.regrasps[i].action;
      ROS_INFO("Executing regrasp %d", action);

      if (action == PICK)
	{
	  if (!pick(req.regrasps[i].pick)) 
	    success = false;
	}
      if (action == DROOP_IN_FINGERS)
	{
	  if (!droopInFingers(req.regrasps[i].droopInFingers))
	    success = false;
	}
      if (action == PLACE)
	{
	  if (!place(req.regrasps[i].place))
	    success = false;
	}      
      if (action == MOVE)
        {
          if (!move(req.regrasps[i].move))
              success = false;
        }
      if (action == ROLL_TO_GROUND)
	{
	  if (!rollToGround(req.regrasps[i].rollToGround))
	    success = false;
	}
      if (action == THROW_TO_PALM)
	{
	  if (!throwToPalm(req.regrasps[i].throwToPalm))
	    success = false;
	}
      if (action == ROLL_TO_PALM)
	{
	  if (!rollToPalm(req.regrasps[i].rollToPalm))
	    success = false;
	}
      if (action == ROLL_ON_GROUND)
	{
	  if (!rollOnGround(req.regrasps[i].rollOnGround))
	    success = false;
	}
      if (action == PUSH_IN_FINGERS)
	{
	  if (!pushInFingers(req.regrasps[i].pushInFingers))
	    success = false;
	}
      if (action == THROW_AND_FLIP)
	{
	  if(!throwAndFlip(req.regrasps[i].throwAndFlip))
	    success = false;
	}
      if (action == PUSH_IN_ENVELOPING)
	{
	  if(!pushInEnveloping(req.regrasps[i].pushInEnveloping))
	    success = false;
	}
      if (action == VIBRATE)
	{
	  if (!vibrate(req.regrasps[i].vibrate))
	    success = false;
	}
      if (action == SQUEEZE)
	{
	  if (!squeeze(req.regrasps[i].squeeze))
	    success = false;
	}
      if (action == THROW_TO_FINGERTIP)
	{
	  if (!throwToFingertip(req.regrasps[i].throwToFingertip))
	    success = false;
	}
      if (action == ROLL_TO_FINGERTIP)
	{
	  if (!rollToFingertip(req.regrasps[i].rollToFingertip))
	    success = false;
	}
      if (action == LONG_EDGE_DROOP)
	{
	  if (!longEdgeDroop(req.regrasps[i].longEdgeDroop))
	    success = false;
	}
      if (action == SHORT_EDGE_DROOP)
	{
	  if (!shortEdgeDroop(req.regrasps[i].shortEdgeDroop))
	    success = false;
	}
      if (action == TOPPLE)
	{
	  if (!topple(req.regrasps[i].topple))
	    success = false;
	}
      if (action == DROOP)
	{
	  if (!droopGeneral(req.regrasps[i].droop))
	    success = false;
	}

      if (!success)
	break;
    }
  res.success = success;

  return true;
}

void RegraspController::init()
{
  ROS_INFO("Initializing system...");


  regraspComm::SetHand msg; //TODO Nikhil: Should we change the name of msg to SetSystem ?
  msg.tcp = 100.0;
  msg.ori = 35.0;
  msg.zone = 0;
  msg.hand_speed = 0.9; //TODO Nikhil: Changed to 0.9;  0.9 is very close to max limit of 1 (And we rarely need that fast working speed)
  msg.hand_force = 0.9;
  geometry_msgs::Pose pose;
  pose.position.x = 600;
  pose.position.y = 300;
  pose.position.z = 200;
  pose.orientation.w = 0.0;
  pose.orientation.x = 0.707;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.0;
  msg.pose = pose;
  double joints[6] = {0.0, 0.0, 0.0, 0.0, 90.0, 0.0};

  robot.setupRobot(msg.tcp, msg.ori, msg.zone, joints, msg.pose);
  hand.setup(msg.hand_speed, msg.hand_force);

  ROS_INFO("Initialization complete.");
}

bool RegraspController::pick(regraspComm::Pick msg) 
{
  ROS_INFO("Starting pick...");
  hand.openHand();

  // Only grasp something approaching from the top
  robot.SetJoints(0,0,0,0,90,0); //waypoint
  robot.relativeMoveArm(0,0,120,msg.pose);
  robot.moveArm(msg.pose);
  hand.closeHand();
  robot.relativeMoveArm(0,0,120,msg.pose);

  robot.SetDefaults();
  ROS_INFO("Pick completed.");
  return true;
}

bool RegraspController::place(regraspComm::Place msg)
{
  ROS_INFO("Starting placing...");
  robot.moveArm(msg.pose);
  hand.openHand();

  robot.SetDefaults();
  ROS_INFO("Place completed.");
  return true;
}

bool RegraspController::move(regraspComm::Move msg)
{
  ROS_INFO("Starting move...");
  if (msg.cart)
    robot.moveArm(msg.pose);
  else
    robot.SetJoints(msg.joints[0],msg.joints[1],msg.joints[2],msg.joints[3],msg.joints[4],msg.joints[5]);

  if (msg.moveHand)
    hand.SetAngle(msg.angle);

  ROS_INFO("Move completed.");
  return true;
}

bool RegraspController::throwToPalm(regraspComm::ThrowToPalm msg)
{
  ROS_INFO("Starting ThrowToPalm...");
  // move into position
  robot.invertHand();
  robot.moveToTop();

  // prepare for throw
  hand.SetSpeed(msg.throw_speed);
  double minFingerAngle = RegraspController::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle-20);
  usleep(msg.tsleep);

  // throw
  robot.SpecialCommand(1,1082.4,1002.96,3.64,15.98,9.63);

  // regrasp
  hand.SetSpeed(msg.grasp_speed);
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("ThrowToPalm completed.");
  return true;
}

bool RegraspController::pushInEnveloping(regraspComm::PushInEnveloping msg)
{
  ROS_INFO("Starting PushInEnveloping...");
  std::vector<HomogTransf> transforms = RegraspController::calcPushInEnvelopingTransforms(msg.pose);
  HomogTransf TRotX = transforms.at(0);
  HomogTransf TRotY = transforms.at(1);
  HomogTransf TPos = transforms.at(2);
  HomogTransf Cdown = transforms.at(3);
  double maxFingerAngle = RegraspController::calcMaxFingerAngle();

  Vec transSafe;
  RegraspController::poseToVec(msg.pose, transSafe);

  Vec transPushStop;
  RegraspController::poseToVec(msg.push_pose, transPushStop); 

  HomogTransf TouchPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchPos);
  ROS_INFO("maxFingerAngle %f", maxFingerAngle-0.25);
  hand.SetAngle(maxFingerAngle-.25);
  usleep(500000);
  
  HomogTransf PushStPos=HomogTransf(((TPos*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStPos);
  hand.closeHand();
  usleep(500000);

  robot.SetCartesian(TouchPos);
  
  HomogTransf TouchInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transSafe);
  robot.SetCartesian(TouchInvPos);
  robot.SetJoints(0,0,0,0,90,0);

  hand.SetAngle(maxFingerAngle-.25);
  usleep(500000);
  
  HomogTransf PushStInvPos=HomogTransf(((TPos*TRotY*TRotX*TPos.inv())*Cdown).getRotation(), transPushStop);
  robot.SetCartesian(PushStInvPos);
  
  hand.closeHand();
  usleep(500000);
  
  robot.SetCartesian(TouchInvPos);

  robot.SetDefaults();
  ROS_INFO("PushInEnveloping completed.");
  return true;
}

// TODO: currently only goes from palm to fingertips. goal>>roll the 
// cylinder by a specific angle (msg.roll_angle)
//TODO Nikhil: Roll to specific angle is difficult to implement with current shape of fingers.
//We need new design for figners.

bool RegraspController::rollToFingertip(regraspComm::RollToFingertip msg)
//Nikhil: Needs corrections: hand needs to be inclined, so add move Arm (joint move) needed before hand open...
{
  ROS_INFO("Starting RollToFingertip...");
/*
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
  */
  double maxFingerAngle = RegraspController::calcMaxFingerAngle();
  double RollReleaseAngle= maxFingerAngle-msg.roll_angle;  //To go to fingertip, roll_angle=17.5
  
  robot.SetJoints(0,0,0,0,10,-90);
  hand.SetAngle(RollReleaseAngle);
  ROS_INFO("RollReleaseAngle %f, maxFingerAngle %f", RollReleaseAngle, maxFingerAngle);
  usleep(1000000);
  //hand.WaitRest(1); 
  
  
  double HAND_CLOSEAfterRegrasp_ANGLE=RollReleaseAngle+5;
  hand.SetAngle(HAND_CLOSEAfterRegrasp_ANGLE);
  hand.WaitRest(1); //TODO Nikhil: added 
  
  //hand.openHand(); //TODO Nikhil: Should be removed otherwise it will drop the object. 
  
  robot.SetDefaults();
  ROS_INFO("RollToFingertip completed.");
  return true;
}

// TODO: add params for freq/geometry
bool RegraspController::vibrate(regraspComm::Vibrate msg)
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

bool RegraspController::rollOnGround(regraspComm::RollOnGround msg)
//TODO (NIKHIL CHECK): roll_place_pose should be derived from roll_pose and angle of rotation we want.
{
  ROS_INFO("Starting RollOnGround...");

  robot.moveArm(msg.roll_place_pose);
  double rollReleaseAngle = RegraspController::calcMinFingerAngle();
  hand.SetAngle(rollReleaseAngle);
  hand.WaitRest(1);
  usleep(2000000);
  
  double rollTranslation = (57.0/180.0)*(msg.rollAngle); //X-movement 109.95 for full rotation = circumfernce of cyl dia 35 mm, but Nikhil uses 57 for half rotaion
  geometry_msgs::Pose roll_stop_pose;
  roll_stop_pose = msg.roll_place_pose;
  roll_stop_pose.position.x = roll_stop_pose.position.x - rollTranslation; //NIKHIL: PLEASE CHECK IS THI IS CORRECT (ACCESSING ELEMENTS).
  
  robot.SetSpeed(msg.tcp, msg.ori);
  robot.moveArm(roll_stop_pose);

  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("RollOnGround completed.");
  return true;
}

// TODO: this includes both vert to hor and vice versa. how do we want to separate?
//TODO: NIKHIL: I THINK HAND CLSE ANGLE  SHOULD BE PART OF MESSAGE. 
// now computes trajectory given frame of fixture TODO: detect frame 
bool RegraspController::pushInFingers(regraspComm::PushInFingers msg)
{
  ROS_INFO("Starting PushInFingers...");
  robot.SetZone(msg.zone);
  hand.SetAngle(70);
  hand.WaitRest(1);
  geometry_msgs::Quaternion quatPush = msg.frame_pose.orientation;
  
  Vec transFrame;
  RegraspController::poseToVec(msg.frame_pose, transFrame); 

  //robot.SetCartesian(850,230,177,0.0,0.707,0.707,0.0);

  robot.relativeMoveArm(-50,0,65,msg.frame_pose);
  robot.relativeMoveArm(5,0,5);
  robot.relativeMoveArm(5,0,5);
  robot.relativeMoveArm(5,0,5);
  robot.relativeMoveArm(5,0,3);
  robot.relativeMoveArm(5,0,2);
  robot.relativeMoveArm(5,0,1);
  robot.relativeMoveArm(5,0,1);
  robot.relativeMoveArm(5,0,-1);
  robot.relativeMoveArm(5,0,0);
  robot.relativeMoveArm(5,0,0);
  robot.relativeMoveArm(5,0,0);
  robot.relativeMoveArm(5,0,0);
  robot.relativeMoveArm(0,0,100);
  robot.relativeMoveArm(-50,0,95,msg.frame_pose);

  robot.relativeMoveArm(-40,0,84,msg.frame_pose);
  robot.relativeMoveArm(5,0,-3);
  robot.relativeMoveArm(20,0,-1);
  robot.relativeMoveArm(10,0,-1);
  robot.relativeMoveArm(5,0,-1);
  robot.relativeMoveArm(5,0,-2);
  robot.relativeMoveArm(5,0,-2);
  robot.relativeMoveArm(10,0,-1);
  robot.relativeMoveArm(5,0,-1);
  robot.relativeMoveArm(0,0,100);


  /// Vec transPtA1=transFrame+Vec("-50 0 65",3);
  /// Vec transPtA2=transFrame+Vec("-45 0 70",3);
  /// Vec transPtA3=transFrame+Vec("-40 0 75",3);
  /// Vec transPtA4=transFrame+Vec("-35 0 80",3);
  // Vec transPtA5=transFrame+Vec("-30 0 83",3);
  // Vec transPtA6=transFrame+Vec("-25 0 85",3);
  // Vec transPtA7=transFrame+Vec("-20 0 86",3);
  // Vec transPtA8=transFrame+Vec("-15 0 87",3);
  // Vec transPtA9=transFrame+Vec("-10 0 86",3);
  /// Vec transPtA10=transFrame+Vec("-5 0 86",3);
  // Vec transPtA11=transFrame+Vec("00 0 86",3);
  // Vec transPtA12=transFrame+Vec("05 0 86",3);
  // Vec transPtA13=transFrame+Vec("10 0 86",3);
  // Vec transPtA14=transFrame+Vec("10 0 186",3);
  
  // robot.SetCartesian(transPtA1[0],transPtA1[1],transPtA1[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA2[0],transPtA2[1],transPtA2[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA3[0],transPtA3[1],transPtA3[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA4[0],transPtA4[1],transPtA4[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA5[0],transPtA5[1],transPtA5[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA6[0],transPtA6[1],transPtA6[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA7[0],transPtA7[1],transPtA7[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA8[0],transPtA8[1],transPtA8[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA9[0],transPtA9[1],transPtA9[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA10[0],transPtA10[1],transPtA10[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA11[0],transPtA11[1],transPtA11[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA12[0],transPtA12[1],transPtA12[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA13[0],transPtA13[1],transPtA13[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtA14[0],transPtA14[1],transPtA14[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
    
  // robot.SetCartesian(transPtA1[0],transPtA1[1],transPtA1[2]+30,quatPush.x, quatPush.y, quatPush.z, quatPush.w);
   
  // Vec transPtB1=transFrame+Vec("-40 0 84",3);
  // Vec transPtB2=transFrame+Vec("-35 0 81",3);
  // Vec transPtB3=transFrame+Vec("-15 0 80",3);
  // Vec transPtB4=transFrame+Vec("-5 0 79",3);
  // Vec transPtB5=transFrame+Vec("0 0 78",3);
  // Vec transPtB6=transFrame+Vec("5 0 76",3);
  // Vec transPtB7=transFrame+Vec("10 0 74",3);
  // Vec transPtB8=transFrame+Vec("20 0 73",3);
  // Vec transPtB9=transFrame+Vec("25 0 72",3);
  // Vec transPtB10=transFrame+Vec("25 0 172",3);
  
  // robot.SetCartesian(transPtB1[0],transPtB1[1],transPtB1[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB2[0],transPtB2[1],transPtB2[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB3[0],transPtB3[1],transPtB3[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB4[0],transPtB4[1],transPtB4[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB5[0],transPtB5[1],transPtB5[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB6[0],transPtB6[1],transPtB6[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB7[0],transPtB7[1],transPtB7[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB8[0],transPtB8[1],transPtB8[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB9[0],transPtB9[1],transPtB9[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  // robot.SetCartesian(transPtB10[0],transPtB10[1],transPtB10[2],quatPush.x, quatPush.y, quatPush.z, quatPush.w);
  
  /*
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
  */
  robot.SetDefaults();
  ROS_INFO("PushInFingers completed.");
  return true;
}

bool RegraspController::throwAndFlip(regraspComm::ThrowAndFlip msg)
{
  ROS_INFO("Starting topple....");
  robot.invertHand(); 
  hand.WaitRest(0.250);
    
  robot.moveArm(msg.pose);
  hand.SetSpeed(msg.hand_speed);
  hand.SetAngle(msg.topple_opnAngle); 
  hand.WaitRest(1);
  //hand.closeHand();
  hand.SetAngle(60);
  usleep(msg.sleep);
  
  robot.SpecialCommand(3,1080.0,850.0,17.71,18.71,18.71);

  robot.SetDefaults();
  ROS_INFO("ThrowAndFlip completed.");
  return true;
}

bool RegraspController::rollToGround(regraspComm::RollToGround msg)
{
  ROS_INFO("Starting RollToGround...");
  // reset finger 1 (according to weiwei) is pointing up
  robot.SetJoints(0,0,0,0,90,0);
  ROS_INFO("resetting");

  robot.SetJoints(0,20,20,0,-40,31);  //WAY POINT
  usleep(2000000);

  hand.SetSpeed(msg.hand_speed);
  robot.moveArm(msg.flip_pose);
  usleep(3000000);
  //  hand.SetSpeed(msg.hand_speed);
  double minFingerAngle = RegraspController::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle - 20);
  usleep(msg.sleep);
  //ROS_INFO("sleep %f", msg.sleep);
  
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("RollToGround completed.");
  return true;
}

bool RegraspController::rollToPalm(regraspComm::RollToPalm msg)
{
  ROS_INFO("Starting RollToPalm... ");
  //robot.SetJoints(-1.37, 31.99, 16.84, 0.1, -75.5, -88); // waypoint bc IK is bad
  //robot.SetJoints(-1.37,31.99,16.84,0.1,-72,35);
  ROS_INFO("angle %f",msg.angle);
  robot.SetJoints(-1.37,31.99,16.84,0.1,msg.angle,-75);
  usleep(2000000);

  hand.SetSpeed(msg.hand_speed);
  
  double minFingerAngle = RegraspController::calcMinFingerAngle();
  hand.SetAngle(minFingerAngle-20);
  usleep(msg.sleep);
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("RollToPalm completed.");
  return true;
}

bool RegraspController::droopInFingers(regraspComm::Droop msg)
{
  // Assumes the object is at the center of the hand lengthwise 
  // (otherwise do push translate first
  //TODO: Nikhil: future: vision should be able to tell what should be pick pose (close to the end face of the cylinder)
  ROS_INFO("Starting DroopInFingers...");
  //robot.SetJoints(0,0,0,0,90,0);
  geometry_msgs::Pose pre = msg.grasp_pose;
  pre.position.z = pre.position.z + 100;
  robot.moveArm(pre);
  hand.openHand();
  hand.WaitRest(1);
  robot.moveArm(msg.grasp_pose);
  double droopAngle = 50;
  ROS_INFO("droop angle %f", droopAngle);
  hand.SetAngle(droopAngle); // TODO NEEDS TO BE OPTIMIZED
  hand.WaitRest(1);
  
  //robot.SetSpeed(msg.tcp, msg.ori);
  geometry_msgs::Pose lift = msg.grasp_pose;
  lift.position.z = lift.position.z + 105;
  robot.moveArm(lift);
  usleep(1000000);
  geometry_msgs::Pose place = msg.grasp_pose;
  place.position.z = place.position.z + 100;
  robot.moveArm(place);
  usleep(400000);
  //  hand.openHand();
  //usleep(200000);

  robot.SetDefaults();
  ROS_INFO("DroopInFingers completed.");

  double fingerAngle = calcMinFingerAngle();
  if (fingerAngle >= 80 )
    return false;
  else
    return true;

  // ROS_INFO("Starting DroopInFingers...");
  // robot.SetJoints(0,0,0,0,90,0);
  // robot.moveArm(msg.place_pose);
  // hand.openHand();
  // hand.WaitRest(1);
  // robot.moveArm(msg.pick_pose);
  // //ROS_INFO("droop angle %f", msg.droopAngle);
  // hand.SetAngle(msg.droopAngle); // TODO NEEDS TO BE OPTIMIZED
  // hand.WaitRest(1);
  
  // robot.SetSpeed(msg.tcp, msg.ori);
  // robot.moveArm(msg.place2_pose);
  // usleep(4000000);

  // hand.openHand();
  // usleep(200000);
  
  // robot.moveArm(msg.pick2_pose);
  // hand.closeHand();
  // hand.WaitRest(1);
  
  // robot.SetDefaults();
  // ROS_INFO("DroopInFingers completed.");
}

bool RegraspController::throwToFingertip(regraspComm::ThrowToFingertip msg)
{
  ROS_INFO("Starting ThrowToFingertip...");
  robot.invertHand();
  
  //robot.moveToTop();
  //hand.SetAngle(msg.partial_open_angle);
  //hand.WaitRest(0.250);

  robot.moveArm(msg.pose);
  hand.SetSpeed(msg.fspeed);
  hand.SetAngle(msg.partial_open_angle);
  hand.WaitRest(0.25);
  hand.SetAngle(60);
  usleep(msg.sleep);

  robot.SpecialCommand(3,1080.0,950.0,16.71,18.71,18.71);

  robot.invertHand();
  
  //hand.SetSpeed(msg.speed);
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("ThrowToFingertip completed.");
  return true;
}

bool RegraspController::lieToStand(regraspComm::LieToStand msg)
{
  ROS_INFO("Starting LieToStand...");

//   HomogTransf pose;
//   Quaternion quat, quatC, quatCommon, quatMore, quatL;
//   RotMat rotC, rotZ, rotZr, rotZl, rotX;

//   Vec transU = frame + Vec("0 0 300",3); // Hand on top
//   Vec transPut = frame + Vec("-15 5 0",3); // The lower position of the hand to put the block

//   std::cout << transPut << std::endl;

//   transPut[0] = transPut[0] + msg.x;
//   transPut[1] = transPut[1] + msg.y;
//   transPut[2] = transPut[2] + msg.z;
  
//   std::cout << transPut << std::endl;
//   Vec transPutP = frame + Vec("0 0 100",3); // The higher position of the hand to put the block
//   Vec transL = frame + Vec("150 0 45",3); // Hand on left, ready to pick
//   Vec transPick = frame + Vec("30 3 32",3); // Picking position

//   quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down
//   rotC = quatC.getRotMat(); 

//   rotZ.setAxisAngle(z, angle);
//   rotX.setAxisAngle(x, -angle);
//   rotZr.setAxisAngle(z, msg.rotateangle*angle);  //rotate 45 degree more
  
  
//   pose = HomogTransf(rotC,transU);
//   quatCommon = pose.getRotation().getQuaternion();   //commonlly rotate 90 degree

//   pose = HomogTransf(rotC*rotZr,transU);
//   quatMore = pose.getRotation().getQuaternion();   //rotate 45 degree more
    
//   pose = HomogTransf(rotC*rotZ*rotZ*rotX,transL);   
//   quatL = pose.getRotation().getQuaternion();

// // HOLD
//   hand.SetAngle(HAND_CLOSE_LIE);
//   hand.WaitRest(1.25);

// // Action:
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get to top and ready to put.
//   robot.SetSpeed(STCP, SORI);
//   robot.SetCartesian(transPut[0], transPut[1], transPut[2], quatMore[0], quatMore[1], quatMore[2], quatMore[3]); // Put down.
//   hand.SetAngle(HAND_OPEN);                 // When the hand releases, the stuff will rotate around the touching point, but will not drop down
//   hand.WaitRest(1.5);

//   robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get back
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0); // An safty position
//   robot.SetCartesian(transL[0], transL[1], transL[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Hand get to left and ready to pick
//   robot.SetSpeed(STCP, SORI);
//   robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Pick
//   hand.SetAngle(HAND_CLOSE_STAND);
//   hand.WaitRest(1.25);
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatL[0], quatL[1], quatL[2], quatL[3]); // For security
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0);
//   robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Hand get back

  ROS_INFO("LieToStand completed.");
  return true;
}

bool RegraspController::standToLie(regraspComm::StandToLie msg)
{
  ROS_INFO("Starting StandToLie...");
//   HomogTransf pose;
//   Quaternion quat, quatC, quatCommon, quatMore, quatMoreTwo, quatL, quatLPick;
//   RotMat rotC, rotZ, rotHalf, rotNormal, rotX, rotZPick;

//   Vec transU = frame + Vec("0 0 300",3); // Hand on top
//   Vec transPut = frame + Vec("-15 -5 0",3); // The position of the hand to put the block
  
//   transPut[0] = transPut[0] + msg.x;
//   transPut[1] = transPut[1] + msg.y - 60*sin(req.rotateangle*angle);
//   transPut[2] = transPut[2] + msg.z - 80*(1-cos(req.rotateangle*angle));
//   Vec transL = frame + Vec("150 0 45",3); // Hand on left, ready to pick
//   Vec transPick = frame + Vec("30 3 30",3); //Picking Position after flipinhand.....Vec("25 3 40",3); // Picking position
//   //Vec transPick = frame + Vec("34 3 30",3);  // Picking position for place and pick x=37 earlier>>39>>34


//   quatC = Quaternion("0.0 0.7071 0.7071 0.0"); // Hand faces down
//   rotC = quatC.getRotMat(); 

//   rotZ.setAxisAngle(z, angle);
//   rotX.setAxisAngle(x, -angle);
//   rotHalf.setAxisAngle(y, msg.rotateangle*angle/2);
//   rotNormal.setAxisAngle(y, req.rotateangle*angle);  //rotate 45 degree more
//   rotZPick.setAxisAngle(z, -135*PI/180);
  
//   pose = HomogTransf(rotC,transU);
//   quatCommon = pose.getRotation().getQuaternion();   //commenlly rotate 90 degree

//   pose = HomogTransf(rotC*rotNormal,transU);
//   quatMore = pose.getRotation().getQuaternion();
    
//   pose = HomogTransf(rotC*rotZ*rotZ*rotX,transL);
//   quatL = pose.getRotation().getQuaternion();
  
//   pose = HomogTransf(rotC*rotZ*rotZ*rotX*rotZPick,transL);  //Added by nikhil as sequense of regrasps needed specific orientaion
//   quatLPick = pose.getRotation().getQuaternion();

// // HOLD
//   hand.SetAngle(HAND_CLOSE_STAND);
//   hand.WaitRest(1.25);

// // Action:
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get to top and ready to put.
//   robot.SetSpeed(STCP, SORI);
//   robot.SetCartesian(transPut[0], transPut[1], transPut[2], quatMore[0], quatMore[1], quatMore[2], quatMore[3]); // Put down first step.
//   hand.SetAngle(HAND_RELEASE);                 // When the hand releases, the stuff will rotate around the touching point, but will not drop down
//   hand.WaitRest(1.0);
//   hand.SetAngle(HAND_OPEN);
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatCommon[0], quatCommon[1], quatCommon[2], quatCommon[3]); // Hand get back
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0); // A safty position
//   hand.SetAngle(HAND_OPEN_BeforePick);
//   hand.WaitRest(1.0);
//   robot.SetCartesian(transL[0], transL[1], transL[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Hand get to left and ready to pick
//   robot.SetCartesian(transL[0], transL[1], transL[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]);
  
//   robot.SetSpeed(STCP, SORI);
//   //robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatL[0], quatL[1], quatL[2], quatL[3]); // Pick
//   robot.SetCartesian(transPick[0], transPick[1], transPick[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Added by Nikhil 
//   hand.SetAngle(HAND_CLOSE_LIE);
//   ros::Duration(1.5).sleep();
//   robot.SetSpeed(8,10);
//   robot.SetCartesian(transPick[0], transPick[1], transPick[2]+10.0, quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Pick
//   robot.SetSpeed(STCP, SORI);
//   hand.WaitRest(0.25);
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatLPick[0], quatLPick[1], quatLPick[2], quatLPick[3]); // Added by Nikhil
//   robot.SetCartesian(transU[0], transU[1], transU[2], quatL[0], quatL[1], quatL[2], quatL[3]); // for Security
  
//   robot.SetSpeed(FTCP, FORI);
//   robot.SetJoints(-29.0, 33.0, 28.0, 20.0, 40.0, -147.0);
//   robot.SetCartesian(transC[0], transC[1], transC[2], quatC[0], quatC[1], quatC[2], quatC[3]); // Hand get back

  ROS_INFO("StandToLie completed.");
  return true;
}

bool RegraspController::squeeze(regraspComm::Squeeze msg)
{
  ROS_INFO("Starting Squeeze...");
  
  robot.invertHand();
  hand.SetForce(msg.squeeze_force);
  hand.openHand();
  hand.closeHand();

  robot.SetDefaults();
  ROS_INFO("Squeeze completed.");
  return true;
}

bool RegraspController::longEdgeDroop(regraspComm::LongEdgeDroop msg)
{
  ROS_INFO("Starting Long Edge Droop...");
  
  double block_len = 0.01;

  Quaternion quat;
  quat[0] = msg.pick_pose.orientation.w;
  quat[1] = msg.pick_pose.orientation.x;
  quat[2] = msg.pick_pose.orientation.y;
  quat[3] = msg.pick_pose.orientation.z;
  matlab.sendVec("quat", quat);
  matlab.sendCommand("[grasp, angle] = findLongDroopQuat(quat);");
  Quaternion q = matlab.getQuaternion("grasp");
  double angle = matlab.getValue("angle");

  double offset_x = (block_len*(3/8))*cos(angle);
  double offset_y = (block_len*(3/8))*sin(angle);

  Vec v (3);
  v[0] = msg.pick_pose.position.x+offset_x;
  v[1] = msg.pick_pose.position.y+offset_y;
  v[2] = msg.pick_pose.position.z;

  //Quaternion q ("0.3796 -0.5951 -0.5972 0.381");
  //Vec v ("635 200 100", 3);

  // check that regrasp is possible
  HomogTransf approach = HomogTransf(q,v);
  double approach_joints[6];
  bool suc = robot.GetIK(approach, approach_joints);

  if (suc)
    {
      robot.SetCartesian(v[0], v[1], v[2]+50, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      hand.SetAngle(35);
      hand.WaitRest(0.25);
      robot.SetCartesian(v[0], v[1], v[2]-25, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      hand.SetAngle(65);
      hand.WaitRest(0.25);
      robot.SetCartesian(v[0], v[1], v[2]-35, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      hand.SetAngle(75);
      hand.WaitRest(0.25);
      robot.SetCartesian(v[0], v[1], v[2]+100, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      robot.SetCartesian(v[0]-25, v[1], v[2]+40, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      robot.SetCartesian(v[0]-35, v[1], v[2]+10, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      robot.SetCartesian(v[0]-55, v[1], v[2]-5, q[0], q[1], q[2], q[3]);
      usleep(4000000);
      
      hand.SetAngle(25);
      hand.WaitRest(0.25);

      return true;
    }
  else
    return false;

}

bool RegraspController::shortEdgeDroop(regraspComm::ShortEdgeDroop msg)
{
  ROS_INFO("Starting Short Edge Droop...");
  double block_len = 0.01;

  Quaternion quat;
  quat[0] = msg.pick_pose.orientation.w;
  quat[1] = msg.pick_pose.orientation.x;
  quat[2] = msg.pick_pose.orientation.y;
  quat[3] = msg.pick_pose.orientation.z;
  matlab.sendVec("quat", quat);
  matlab.sendCommand("[grasp, angle] = findLongDroopQuat(quat);");
  Quaternion q = matlab.getQuaternion("grasp");
  double angle = matlab.getValue("angle");
  
  double offset_x = (block_len/2)*cos(angle);
  double offset_y = (block_len/2)*sin(angle);
  cout << "x_offset " << offset_x << " y " << offset_y << endl;

  Vec v (3);
  v[0] = msg.pick_pose.position.x+offset_x;
  v[1] = msg.pick_pose.position.y+offset_y;
  v[2] = msg.pick_pose.position.z;

  //Quaternion q ("0.3796 -0.5951 -0.5972 0.381");
  //Vec v ("635 200 100", 3);

  // move to waypoint
  robot.SetCartesian(600,200,300,0,0.707,0.707,0);

  // check that regrasp is possible
  HomogTransf approach = HomogTransf(q,v);
  //double approach_joints[6];
  cout << "hey robbie " << approach.getTranslation() << " " << approach.getQuaternion() << endl; 
  bool suc = true;//robot.GetIK(approach, approach_joints);
  
  cout << "v: " << v << endl;
  cout << "q: " << q << endl;

  robot.SetCartesian(v[0], v[1], v[2]+50, quat[0], quat[1], quat[2], quat[3]);

  if (suc)
    {
      // //  Quaternion q ("0.3796 -0.5951 -0.5972 0.381");
      // //Vec v ("600 200 100", 3);
      // robot.SetCartesian(v[0], v[1], v[2]+50, q[0], q[1], q[2], q[3]);
      // usleep(4000000);
      // hand.SetAngle(35);
      // hand.WaitRest(0.25);
      // robot.SetCartesian(v[0], v[1], v[2]-35, q[0], q[1], q[2], q[3]);
      // usleep(4000000);
      // hand.SetAngle(75);
      // hand.WaitRest(0.25);
      // robot.SetCartesian(v[0], v[1], v[2]+50, q[0], q[1], q[2], q[3]);
      // usleep(4000000);
      
      // // place back on the table
      // robot.SetCartesian(v[0], v[1], v[2], q[0], q[1], q[2], q[3]);
      // usleep(4000000);
      // hand.SetAngle(15);
      // hand.WaitRest(0.25);

     return true;
    }
  else
    {
      ROS_INFO("regrasp failed due to bad starting position");
      return false;
    }
}

bool RegraspController::topple(regraspComm::Topple msg)
{
  ROS_INFO("Starting topple...");
  
  hand.SetAngle(80);
  robot.SetCartesian(msg.pose.position.x, 
		     msg.pose.position.y+40, 
		     msg.pose.position.z-75, 
		     msg.pose.orientation.w, 
		     msg.pose.orientation.x, 
		     msg.pose.orientation.y, 
		     msg.pose.orientation.z);
  robot.SetCartesian(msg.pose.position.x, 
		     msg.pose.position.y-25, 
		     msg.pose.position.z-75, 
		     msg.pose.orientation.w, 
		     msg.pose.orientation.x, 
		     msg.pose.orientation.y, 
		     msg.pose.orientation.z);


  return true;
}

/***********************COMPUTATION***********************/
double RegraspController::calcMinFingerAngle(void)
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

double RegraspController::calcMaxFingerAngle(void)
{
  double motorAngle;
  double fingerAngle[3];

  hand.GetAngles(motorAngle, fingerAngle);
  double maxFingerAngle = fingerAngle[0];
  if (fingerAngle[1] > maxFingerAngle)
  {
    maxFingerAngle=fingerAngle[1];
  }
  if (fingerAngle[2] > maxFingerAngle)
  {
    maxFingerAngle=fingerAngle[2];
  }
  return maxFingerAngle;
}

std::vector<HomogTransf> RegraspController::calcPushInEnvelopingTransforms(geometry_msgs::Pose safe_pose)
{
  RotMat Xrot;
  Vec rotXAxis = Vec("1.0 0.0 0.0", 3);
  double rotXAngle = -90; 
  Xrot.setAxisAngle(rotXAxis, rotXAngle*PI/180.0);
  HomogTransf TRotX=HomogTransf(Xrot, Vec("0.0 0.0 0.0",3));

  RotMat Yrot;
  Vec rotYAxis = Vec("0.0 1.0 0.0", 3);
  double rotYAngle = 180; 
  Yrot.setAxisAngle(rotYAxis, rotYAngle*PI/180.0);
  HomogTransf TRotY=HomogTransf(Yrot, Vec("0.0 0.0 0.0",3));
  
  Vec transSafe;
  RegraspController::poseToVec(safe_pose, transSafe);

  HomogTransf TPos=HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), transSafe); 
  HomogTransf Cdown=HomogTransf(Quaternion("0.0 0.7071 0.7071 0.0").getRotMat(), transSafe);

  std::vector<HomogTransf> transforms;
  transforms.push_back(TRotX);
  transforms.push_back(TRotY);
  transforms.push_back(TPos);
  transforms.push_back(Cdown);

  return transforms;
}

bool RegraspController::poseToVec(geometry_msgs::Pose pose, Vec& vector)
{
  vector = Vec(3);
  vector[0] = pose.position.x;
  vector[1] = pose.position.y;
  vector[2] = pose.position.z;

  return true;
}

bool RegraspController::droopGeneral(regraspComm::Droop msg)
{
  double HAND_DEPTH = 200;
  
  vector<Vec> verts; // = msg.object_vertices;
  for (unsigned int i=0;i<msg.object_vertices.size();i+=3)
    {
      cout << "i " << i << endl;
      Vec v(3);
      v[0] = msg.object_vertices[i];
      v[1] = msg.object_vertices[i+1];
      v[2] = msg.object_vertices[i+2];
      verts.push_back(v);

    }

  cout << "size of verts: " << verts.size() << endl;
  GeometryTools gt(verts);

  //approach(msg.grasp_pose);
  Vec grasp(3);
  grasp[0] = msg.grasp_pose.position.x;
  grasp[1] = msg.grasp_pose.position.y;
  grasp[2] = msg.grasp_pose.position.z;  
  Quaternion q;
  q[0] = msg.grasp_pose.orientation.w;
  q[1] = msg.grasp_pose.orientation.x;
  q[2] = msg.grasp_pose.orientation.y;
  q[3] = msg.grasp_pose.orientation.z;

  // find the center of mass
  Vec com(3);
  gt.centroid(com);

  // find droop direction

  Vec dir = grasp - com;
  cout << "dir: " << dir << endl;

  // Droops (change in orientation) if vec from grasp to centroid 
  // does not point in the boundary
  bool willRotate = gt.pointsIn(grasp, dir);
  cout << "whats up: " << willRotate << endl;
  // Only rotates if obj fits in the hand
  double dist;
  Vec intersectionPt;
  Vec outwards = dir*-1;
  outwards[2] = 0;
  Vec inwards = outwards*-1;
  cout << "inwards: " << inwards << endl;
  gt.findIntersect(grasp, (grasp+outwards),intersectionPt, dist);
  cout << "intersectionPt: " << intersectionPt << " dist: " << dist << endl;
  if (dist >= HAND_DEPTH)
    willRotate = false;

  cout << "willRotate: " << willRotate << endl;

  if (willRotate)
    {
      // grasp the block
      Quaternion grasp_quat;
      double angle;      
      gt.findSideQuat(q,grasp_quat,angle);
      approach(grasp, grasp_quat);

      // lift (droop)
      double height = gt.liftEdge(inwards);
      robot.SetCartesian(msg.grasp_pose.position.x, 
  			 msg.grasp_pose.position.y, 
  			 msg.grasp_pose.position.z+height, 
  			 msg.grasp_pose.orientation.w,
  			 msg.grasp_pose.orientation.x,
  			 msg.grasp_pose.orientation.y,
  			 msg.grasp_pose.orientation.z);
      
      // place the object
      int steps = 5;
      double deltaX = (msg.grasp_pose.position.x - com[0])/steps;
      double deltaY = (msg.grasp_pose.position.y - com[1])/steps; 
      for (int i=0;i<steps;i++)
	{
	  robot.SetCartesian(msg.grasp_pose.position.x-(i*deltaX), 
			     msg.grasp_pose.position.y-(i*deltaY), 
			     msg.grasp_pose.position.z+(height-10*i), 
			     msg.grasp_pose.orientation.w,
			     msg.grasp_pose.orientation.x,
			     msg.grasp_pose.orientation.y,
			     msg.grasp_pose.orientation.z);
	  cout << "place pose:" << endl;
	  cout << msg.grasp_pose.position.x+(i*deltaX) << endl;
	  cout << msg.grasp_pose.position.y+(i*deltaY) << endl; 
	  cout << msg.grasp_pose.position.z+(height-5*i) << endl;
	  
	  hand.SetAngle(10);
	}
      ROS_INFO("General droop is completed!");

    }
  else
    {
      ROS_INFO("ERROR: Regrasp failed!");
      return false;
    }

  return true;

}
    
bool RegraspController::approach(Vec v, Quaternion q)
{
  ROS_INFO("In approach");

  double x = v[0]; //pose.position.x;
  double y = v[1]; //pose.position.y;
  double z = v[2]; //pose.position.z;
  double q0 = q[0]; //pose.orientation.w;
  double q1 = q[1]; //pose.orientation.x;
  double q2 = q[2]; //pose.orientation.y;
  double q3 = q[3]; //pose.orientation.z;

  // PROB: pose close to the table 
  // circular-esque set of waypoints
  // x^2/10^2 + y^2/100^2 = 1
  int n = 6;
  double t[6]= {0,2,4,6,8,10};
  double fingerAngles[6] = {65,55,45,35,25,15};

  cout << "starting" << endl;
  for (int i=n-1;i>=0;i--)
    {
      cout << "z " << ellipse(t[i])+z << endl;
      cout << "fingerAngle  " << fingerAngles[i] << endl; 
      robot.SetCartesian(x,y,z+ellipse(t[i]),q0,q1,q2,q3);
      hand.SetAngle(fingerAngles[i]);
      hand.WaitRest(0.25);
    }

 

  return true;
}

double RegraspController::ellipse(double t)
{
  return -10*sqrt(100-pow(t,2)) +100;
}

// bool RegraspController::droopGeneral(regraspComm::DroopGeneral msg)
// {

//   ROS_INFO("Starting DroopInFingers...");

//   geometry_msgs::Pose grasp = msg.grasp;
//   vector<Vec> obj_boundary = msg.obj_boundary;
//   int verts = obj_boundary.size();

//   Vec center = Vec("0 0 0",3);
//   for (int i=0;i<verts;i++)
//     {
//       center[0] = center[0] + obj_boundary[i][0];
//       center[1] = center[1] + obj_boundary[i][1];
//       center[2] = center[2] + obj_boundary[i][2];
//     }

//   Vec toCenter(3);
//   toCenter[0] = grasp.position.x - center[0];
//   toCenter[1] = grasp.position.y - center[1];
//   toCenter[2] = grasp.position.z - center[2];

//   // object will droop in direction of toCenter if it can
//   Vec toEdge = -1*toCenter;
//   toEdge[2] = 0;
//   Vec g = Vec(3);
//   g[0] = grasp.position.x;
//   g[1] = grasp.position.y;
//   g[2] = 0;//grasp.position.z;

//   // find the edge closest to the palm
//   // the vertices are only the verts of the boundary of the block
//   for (int i=0;i<(verts-1);i++)
//     {
//       Vec x1 = verts[i];
//       Vec x2 = verts[i+1];
//       Vec x3 = g;
//       Vec x4 = g+toEdge;

//       Vec pt(3);
//       pt[0] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[0]-x4[0]) - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]))/
// 	((x1[0] - x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]));
//       pt[1] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]))/
// 	((x1[0] - x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]));
//       pt[2] = 0;
      
//       double dist = sqrt(pow((pt[0]-g[0]),2)+pow((pt[1]-g[1]),2)+pow((pt[2]-grasp.position.z),2));
//       double hand_height = 0.06;
//       if (dist >= hand_height)  // will not droop
// 	return 0;

//       // now you should approach at a very long angle
//       // then lift, and place with a lighter grasp


//   //     // edge is (x,y,z) = (a1,a2,a3)+slope(b1-a1,b2-a2,b3-a3)
//   //     Vec slope = obj_boundary[i+1] - obj_boundary[i];
//   //     Vec a = obj_boundary[i];

//   //     // find soln to system of eqns
//   //     // a1 + t(b1-a1) = g1+s(toEdge[0])
//   //     // a2 + t(b2-a2) = g2+s(toEdge[1])
//   //     // a3 + t(b3-a3) = g3+s(toEdge[3])

//   //     Vec tmp = (a-g);
//   //     double x = (tmp[0]/toEdge[0]);
//   //     double y = (toEdge[0]/slope[0]);

//   //     double t = (tmp[1] - x*toEdge[1])/(y*toEdge[1]-slope[1]);
//   //     double s = x * t*y;
//   //     if (tmp[2] == s*toEdge[2] -t*slope[2]) //intersects
//   // 	{
//   // 	  Vec pt = a+t*slope;
//   // 	  double dist = sqrt(pow((pt[0]-g[0]),2)+pow((pt[1]-g[1]),2)+pow((pt[2]-g[2]),2));

//   // 	  if (dist >= 

//   // 	}


//   //   }

//   // toEdge[0] = grasp.position.x - center[0];
//   // toEdge[1] = grasp.position.y - center[1];
//   // toEdge[2] = grasp.position.z - center[2];


//   //Robot.Setjoints(0,0,0,0,90,0);
//   geometry_msgs::Pose pre = msg.pose;
//   pre.position.z = pre.position.z + 100;
//   robot.moveArm(pre);
//   hand.openHand();
//   hand.WaitRest(1);
//   robot.moveArm(msg.pose);
//   ROS_INFO("droop angle %f", msg.droopAngle);
//   hand.SetAngle(msg.droopAngle); // TODO NEEDS TO BE OPTIMIZED
//   hand.WaitRest(1);
  
//   //robot.SetSpeed(msg.tcp, msg.ori);
//   geometry_msgs::Pose lift = msg.pose;
//   lift.position.z = lift.position.z + 105;
//   robot.moveArm(lift);
//   usleep(1000000);
//   geometry_msgs::Pose place = msg.pose;
//   place.position.z = place.position.z + 100;
//   robot.moveArm(place);
//   usleep(400000);
//   //  hand.openHand();
//   //usleep(200000);

//   robot.SetDefaults();
//   ROS_INFO("DroopInFingers completed.");

//   double fingerAngle = calcMinFingerAngle();
//   if (fingerAngle >= 80 )
//     return false;
//   else
//     return true;
// }


// GeometryTools::GeometryTools(vector<Vec> v)
// {
//   vertices = v;
//   num_sides = v.size();

//   for (int i=0;i<(num_sides-1);i++)
//     {
//       side_lengths.push_back(dist(vertices[i][0], vertices[i][1], 
// 				  vertices[i+1][0], vertices[i+1][1]));
//     }
//   side_lengths.push_back(dist(vertices[num_sides-1][0], vertices[num_sides-1][1], 
// 			      vertices[0][0], vertices[0][1]));


// }

// bool GeometryTools::findSideQuat(Quaternion q, Quaternion &grasp, double &angle)
// {
//   Quaternion yaw = q.angle2quat(0.0,0.0,(PI*(5/12)));
//   grasp = q^yaw;

//   Quaternion def = Quaternion("0.0 0.707 0.707 0.0");
//   Quaternion tmp = q/def;
//   double p1,p2;
//   tmp.quat2angle(angle, p1, p2);

// }

// bool GeometryTools::findIntersect(Vec x1, Vec x2, Vec &i, double &d)
// {
//   d = INFINITY;
//   for (int j=0;j<(num_sides-1);j++)
//     {
//       Vec intersect(3);
//       if (findIntersect(vertices[j], vertices[j+1], x1, x2, intersect))
// 	{
// 	  double tmp = dist(x1[0],x1[1],intersect[0],intersect[1]);
// 	  if (tmp<d)
// 	    {
// 	      d = tmp;
// 	      i = intersect;
// 	    }
// 	}
//     }

//   Vec intersect(3);
//   if (findIntersect(vertices[num_sides-1], vertices[0], x1, x2, intersect))
//     {
//       double tmp = dist(x1[0],x1[1],intersect[0],intersect[1]);
//       if (tmp<d)
// 	{
// 	  d = tmp;
// 	  i = intersect;
// 	}
//     }

//   return true;
// }

// bool GeometryTools::findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i)
// {
//   double den = ((x1[0] - x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]));
//   if (den == 0)
//     return false;

//   i[0] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[0]-x4[0]) - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]))/den;
//   i[1] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]))/den;
//   i[2] = 0;
  
//   return true;

// }

// double GeometryTools::liftEdge(Vec dir)
// {
//   double edge= 0 ;
//   double best = -INFINITY;
//   for (int i=0;i<(num_sides-1);i++)
//     {
//       Vec side = vertices[i] - vertices[i+1];
//       double dot = abs(dir*side);
//       if (dot > best)
//         {
//           best = dot;
//           edge = dist(vertices[i][0],vertices[i][1],
//                       vertices[i+1][0],vertices[i+1][1]);
//         }
//     }
//   Vec side = vertices[num_sides-1] - vertices[0];
//   double dot = abs(dir*side);
//   if (dot > best)
//     {
//       best = dot;
//       edge = dist(vertices[num_sides-1][0],vertices[num_sides-1][1],
//                   vertices[0][0],vertices[0][1]);

//     }
//   return edge;
// }

// bool GeometryTools::pointsIn(Vec pt, Vec dir)
// {
//   // find where line intersects plane of z=0
//   double scalar = pt[2]/dir[2];
//   Vec intersect = pt - dir*scalar;
//   // cout << "pt: " << pt << endl;
//   // cout << "dir: " << dir << endl;
//   // cout << "scalar: " << scalar << endl;
//   // cout << "intersect " << intersect << endl;

//   // assuming some shit i think
//   double x_min, y_min, x_max, y_max;
//   boundingBox(x_min, y_min, x_max, y_max);

//   if ((intersect[0] < x_max) && (intersect[0] > x_min) &&
//       (intersect[1] < y_max) && (intersect[1] > y_min))
//     return true;
//   else
//     return false;

// }

// bool GeometryTools::centroid(Vec &com)
// {
//   double x = 0;
//   double y = 0;
  
//   for (int i=0;i<num_sides;i++)
//     {
//       x += vertices[i][0];
//       y += vertices[i][1];
//     }

//   x = x/num_sides;
//   y = y/num_sides;

//   com[0] = x;
//   com[1] = y;
//   com[2] = 0;

//   return true;
// }

// double GeometryTools::area(void)
// {
//   if (regular())
//     return (pow(side_lengths[0],2)*num_sides)/(4*tan(PI/num_sides));

//   if (num_sides==4)
//     {
//       double width = std::max(side_lengths[0], side_lengths[2]);
//       double height = std::max(side_lengths[1], side_lengths[3]);
//       return (width*height);
//     }
  
//   ROS_INFO("ERROR: area for this shape not implemented yet");
//   return 0;
  
// }

// double GeometryTools::max(vector<double> x)
// {
//   double m = x[0];
//   for (int i=1;i<x.size();i++)
//     {
//       if (x[i] > m)
// 	m = x[i];
//     }
//   return m;
// }

// int GeometryTools::argmax(vector<double> x)
// {
//   double m = x[0];
//   int ind = 0;
//   for (int i=1;i<x.size();i++)
//     {
//       if (x[i] > m)
// 	{
// 	  m = x[i];
// 	  ind = i;
// 	}
//     }
//   return m;
// }

// double GeometryTools::min(vector<double> x)
// {
//   double m = x[0];
//   for (int i=1;i<x.size();i++)
//     {
//       if (x[i] < m)
// 	m = x[i];
//     }
//   return m;
// }

// int GeometryTools::argmin(vector<double> x)
// {
//   double m = x[0];
//   int ind = 0;
//   for (int i=1;i<x.size();i++)
//     {
//       if (x[i] < m)
// 	{
// 	  m = x[i];
// 	  ind = i;
// 	}
//     }
//   return m;
// }

// bool GeometryTools::regular(void)
// {
//   bool tmp = side_lengths[0];
//   for (int i=1;i<num_sides;i++)
//     {
//       if (side_lengths[i] != tmp)
// 	return false;
//     }
//   return true;
// }

// double GeometryTools::dist(double dX0, double dY0, double dX1, double dY1)
// {
//   return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
// }

// bool GeometryTools::boundingBox(double &x_min, double &y_min, 
// 				double &x_max, double &y_max)
// {
//   x_min = INFINITY;
//   y_min = INFINITY;
//   x_max = -INFINITY;
//   y_max = -INFINITY;
  
//   for (int i=0;i<num_sides;i++)
//     {
//       double x = vertices[i][0];
//       double y = vertices[i][1];

//       if (x < x_min)
// 	x_min = x;
//       if (x > x_max)
// 	x_max = x;
//       if (y < y_min)
// 	y_min = y;
//       if (y > y_max)
// 	y_max = y;
//     }
  
//   return true;
// }

// vector<double> GeometryTools::getSideLengths(void)
// {
//   return side_lengths;
// }


int main(int argc, char** argv)
{
  ros::init(argc, argv, "regrasp_controller");
  ros::NodeHandle node;
  RegraspController regrasper(&node);

  regrasper.advertiseServices();
  ROS_INFO("Regrasp server ready...");
  ros::spin();

  return 0;

}
