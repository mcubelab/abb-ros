#include "regraspComm/regrasp_comm.h"

RegraspComm::RegraspComm()
{
}

RegraspComm::RegraspComm(ros::NodeHandle *np)
{
  node = np;
  subscribe();
}

RegraspComm::~RegraspComm()
{
  shutdown();
}

void RegraspComm::subscribe()
{
  handle_regrasp_Execute = node->serviceClient<regraspComm::regrasp_Execute>("regrasp_Execute");
}

void RegraspComm::shutdown()
{
  handle_regrasp_Execute.shutdown();
}



bool RegraspComm::Execute(vector<regraspComm::Regrasp> regrasps)
{
  ROS_INFO("Executing regrasps...");
  regrasp_Execute_srv.request.regrasps = regrasps;
  // for (int i=0; i<regrasps.size(); i++)
  //   regrasp_Execute_srv.request.regrasps[i] = regrasps[i];
  return handle_regrasp_Execute.call(regrasp_Execute_srv);
}

bool RegraspComm::Execute(vector<int> regrasps)
{
  //ROS_INFO("Executing regrasps...");
  for (int i=0; i<regrasps.size(); i++)
    regrasp_Execute_srv.request.regrasps.push_back(getDefaults(regrasps[i]));
  return handle_regrasp_Execute.call(regrasp_Execute_srv);
}    

bool RegraspComm::Execute(regraspComm::Regrasp regrasp)
{
  vector<regraspComm::Regrasp> regrasps;
  regrasps.push_back(regrasp);
  return Execute(regrasps);
}

bool RegraspComm::Execute(int regrasp)
{
  vector<int> regrasps;
  regrasps.push_back(regrasp);
  return Execute(regrasps);
}


regraspComm::Regrasp RegraspComm::getDefaults(int action)
{
  //  ROS_INFO("Getting/setting default values.");
  regraspComm::Regrasp regrasp_msg;
  regrasp_msg.action = action;

  if (action == PICK) // Pick
    {
      double pick_position_x, pick_position_y, pick_position_z, 
	pick_orientation_x, pick_orientation_y, pick_orientation_z, 
	pick_orientation_w;
      node->getParam("/pick_position_x", pick_position_x);
      node->getParam("/pick_position_y", pick_position_y);
      node->getParam("/pick_position_z", pick_position_z);
      node->getParam("/pick_orientation_x", pick_orientation_x);
      node->getParam("/pick_orientation_y", pick_orientation_y);
      node->getParam("/pick_orientation_z", pick_orientation_z);
      node->getParam("/pick_orientation_w", pick_orientation_w);

      geometry_msgs::Pose pose;
      pose.position.x = pick_position_x;
      pose.position.y = pick_position_y;
      pose.position.z = pick_position_z;
      pose.orientation.x = pick_orientation_x;
      pose.orientation.y = pick_orientation_y;
      pose.orientation.z = pick_orientation_z;
      pose.orientation.w = pick_orientation_w;

      regraspComm::Pick msg;
      msg.pose = pose;

      regrasp_msg.pick = msg;
    }
  else if (action == PLACE) // Place
    {
      double place_position_x, place_position_y, place_position_z, 
	place_orientation_x, place_orientation_y, place_orientation_z, 
	place_orientation_w;
      node->getParam("/place_position_x", place_position_x);
      node->getParam("/place_position_y", place_position_y);
      node->getParam("/place_position_z", place_position_z);
      node->getParam("/place_orientation_x", place_orientation_x);
      node->getParam("/place_orientation_y", place_orientation_y);
      node->getParam("/place_orientation_z", place_orientation_z);
      node->getParam("/place_orientation_w", place_orientation_w);

      geometry_msgs::Pose pose;
      pose.position.x = place_position_x;
      pose.position.y = place_position_y;
      pose.position.z = place_position_z;
      pose.orientation.x = place_orientation_x;
      pose.orientation.y = place_orientation_y;
      pose.orientation.z = place_orientation_z;
      pose.orientation.w = place_orientation_w;

      regraspComm::Place msg;
      msg.pose = pose;

      regrasp_msg.place = msg;
    }
  else if(action == MOVE)
    {
      // initialization pose
      geometry_msgs::Pose pose;
      pose.position.x = 600.0;
      pose.position.y = 300;
      pose.position.z = 200;
      pose.orientation.w = 0.0;
      pose.orientation.x = 0.707;
      pose.orientation.y = 0.707;
      pose.orientation.z = 0.0;

      regraspComm::Move msg;
      msg.cart = true; 
      msg.pose = pose;
      msg.moveHand = false;

      regrasp_msg.move = msg;
    }

  else if (action == THROW_TO_PALM) // ThrowToPalm
    {
      double throwToPalm_setup_position_x, throwToPalm_setup_position_y, 
	throwToPalm_setup_position_z, throwToPalm_setup_orientation_x, 
	throwToPalm_setup_orientation_y, throwToPalm_setup_orientation_z, 
	throwToPalm_setup_orientation_w, tsleep, throw_speed, 
	grasp_speed;
      node->getParam("/throwToPalm_setup_position_x", throwToPalm_setup_position_x);
      node->getParam("/throwToPalm_setup_position_y", throwToPalm_setup_position_y);
      node->getParam("/throwToPalm_setup_position_z", throwToPalm_setup_position_z);
      node->getParam("/throwToPalm_setup_orientation_x", throwToPalm_setup_orientation_x);
      node->getParam("/throwToPalm_setup_orientation_y", throwToPalm_setup_orientation_y);
      node->getParam("/throwToPalm_setup_orientation_z", throwToPalm_setup_orientation_z);
      node->getParam("/throwToPalm_setup_orientation_w", throwToPalm_setup_orientation_w);
      node->getParam("/throwToPalm_tsleep", tsleep);
      node->getParam("/throwToPalm_throw_speed", throw_speed);
      node->getParam("/throwToPalm_grasp_speed", grasp_speed);

      geometry_msgs::Pose pose;
      pose.position.x = throwToPalm_setup_position_x;
      pose.position.y = throwToPalm_setup_position_y;
      pose.position.z = throwToPalm_setup_position_z;
      pose.orientation.x = throwToPalm_setup_orientation_x;
      pose.orientation.y = throwToPalm_setup_orientation_y;
      pose.orientation.z = throwToPalm_setup_orientation_z;
      pose.orientation.w = throwToPalm_setup_orientation_w;

      regraspComm::ThrowToPalm msg;
      msg.setup_pose = pose;
      msg.tsleep = tsleep;
      msg.throw_speed = throw_speed;
      msg.grasp_speed = grasp_speed;

      regrasp_msg.throwToPalm = msg;
    }
  else if (action == PUSH_IN_ENVELOPING) // PushInEnveloping
    {
      double pushInEnveloping_position_x, pushInEnveloping_position_y,
	pushInEnveloping_position_z, pushInEnveloping_orientation_x,
	pushInEnveloping_orientation_y, pushInEnveloping_orientation_z,
	pushInEnveloping_orientation_w, pushInEnveloping_push_position_x, 
	pushInEnveloping_push_position_y,
	pushInEnveloping_push_position_z, pushInEnveloping_push_orientation_x,
	pushInEnveloping_push_orientation_y, pushInEnveloping_push_orientation_z,
	pushInEnveloping_push_orientation_w;

      node->getParam("/pushInEnveloping_position_x", pushInEnveloping_position_x);
      node->getParam("/pushInEnveloping_position_y", pushInEnveloping_position_y);
      node->getParam("/pushInEnveloping_position_z", pushInEnveloping_position_z);
      node->getParam("/pushInEnveloping_orientation_x", pushInEnveloping_orientation_x);
      node->getParam("/pushInEnveloping_orientation_y", pushInEnveloping_orientation_y);
      node->getParam("/pushInEnveloping_orientation_z", pushInEnveloping_orientation_z);
      node->getParam("/pushInEnveloping_orientation_w", pushInEnveloping_orientation_w);
      node->getParam("/pushInEnveloping_push_position_x", pushInEnveloping_push_position_x);
      node->getParam("/pushInEnveloping_push_position_y", pushInEnveloping_push_position_y);
      node->getParam("/pushInEnveloping_push_position_z", pushInEnveloping_push_position_z);
      node->getParam("/pushInEnveloping_push_orientation_x", pushInEnveloping_push_orientation_x);
      node->getParam("/pushInEnveloping_push_orientation_y", pushInEnveloping_push_orientation_y);
      node->getParam("/pushInEnveloping_push_orientation_z", pushInEnveloping_push_orientation_z);
      node->getParam("/pushInEnveloping_push_orientation_w", pushInEnveloping_push_orientation_w);

      geometry_msgs::Pose pose;
      pose.position.x = pushInEnveloping_position_x;
      pose.position.y = pushInEnveloping_position_y;
      pose.position.z = pushInEnveloping_position_z;
      pose.orientation.x = pushInEnveloping_orientation_x;
      pose.orientation.y = pushInEnveloping_orientation_y;
      pose.orientation.z = pushInEnveloping_orientation_z;
      geometry_msgs::Pose push_pose;
      push_pose.position.x = pushInEnveloping_push_position_x;
      push_pose.position.y = pushInEnveloping_push_position_y;
      push_pose.position.z = pushInEnveloping_push_position_z;
      push_pose.orientation.x = pushInEnveloping_push_orientation_x;
      push_pose.orientation.y = pushInEnveloping_push_orientation_y;
      push_pose.orientation.z = pushInEnveloping_push_orientation_z;

      regraspComm::PushInEnveloping msg;
      msg.pose = pose;
      msg.push_pose = push_pose;

      regrasp_msg.pushInEnveloping = msg;
    }
  else if (action == VIBRATE) // Vibrate
    {

      double angle;
      node->getParam("/vibrate_angle", angle);

      regraspComm::Vibrate msg;
      msg.vibrate_angle = angle;
      
      regrasp_msg.vibrate = msg;
    }
  else if (action == THROW_AND_FLIP) // ThrowAndFlip
    {
      double throwAndFlip_pose_position_x, throwAndFlip_pose_position_y, 
	throwAndFlip_pose_position_z, throwAndFlip_pose_orientation_x, 
	throwAndFlip_pose_orientation_y, throwAndFlip_pose_orientation_z, 
	throwAndFlip_pose_orientation_w, sleep, speed, angle;
      node->getParam("/throwAndFlip_pose_position_x", throwAndFlip_pose_position_x);
      node->getParam("/throwAndFlip_pose_position_y", throwAndFlip_pose_position_y);
      node->getParam("/throwAndFlip_pose_position_z", throwAndFlip_pose_position_z);
      node->getParam("/throwAndFlip_pose_orientation_x", throwAndFlip_pose_orientation_x);
      node->getParam("/throwAndFlip_pose_orientation_y", throwAndFlip_pose_orientation_y);
      node->getParam("/throwAndFlip_pose_orientation_z", throwAndFlip_pose_orientation_z);
      node->getParam("/throwAndFlip_pose_orientation_w", throwAndFlip_pose_orientation_w);
      node->getParam("/throwAndFlip_sleep", sleep);
      node->getParam("/throwAndFlip_hand_speed", speed);
      node->getParam("/throwAndFlip_topple_OpnAngle", angle);

      geometry_msgs::Pose pose;
      pose.position.x = throwAndFlip_pose_position_x;
      pose.position.y = throwAndFlip_pose_position_y;
      pose.position.z = throwAndFlip_pose_position_z;
      pose.orientation.x = throwAndFlip_pose_orientation_x;
      pose.orientation.y = throwAndFlip_pose_orientation_y;
      pose.orientation.z = throwAndFlip_pose_orientation_z;
      pose.orientation.w = throwAndFlip_pose_orientation_w;

      regraspComm::ThrowAndFlip msg;
      msg.pose = pose;
      msg.sleep = sleep;
      msg.hand_speed = speed;
      msg.topple_opnAngle = angle;

      regrasp_msg.throwAndFlip = msg;
    }
  else if (action == PUSH_IN_FINGERS) // PushInFingers
    {
      double pushInFingers_frame_position_x, pushInFingers_frame_position_y, 
	pushInFingers_frame_position_z, pushInFingers_frame_orientation_x, 
	pushInFingers_frame_orientation_y, pushInFingers_frame_orientation_z, 
	pushInFingers_frame_orientation_w, zone;
      node->getParam("/pushInFingers_frame_position_x", pushInFingers_frame_position_x);
      node->getParam("/pushInFingers_frame_position_y", pushInFingers_frame_position_y);
      node->getParam("/pushInFingers_frame_position_z", pushInFingers_frame_position_z);
      node->getParam("/pushInFingers_frame_orientation_x", pushInFingers_frame_orientation_x);
      node->getParam("/pushInFingers_frame_orientation_y", pushInFingers_frame_orientation_y);
      node->getParam("/pushInFingers_frame_orientation_z", pushInFingers_frame_orientation_z);
      node->getParam("/pushInFingers_frame_orientation_w", pushInFingers_frame_orientation_w);
      node->getParam("/pushInFingers_zone", zone);

      geometry_msgs::Pose pose;
      pose.position.x = pushInFingers_frame_position_x;
      pose.position.y = pushInFingers_frame_position_y;
      pose.position.z = pushInFingers_frame_position_z;
      pose.orientation.x = pushInFingers_frame_orientation_x;
      pose.orientation.y = pushInFingers_frame_orientation_y;
      pose.orientation.z = pushInFingers_frame_orientation_z;
      pose.orientation.w = pushInFingers_frame_orientation_w;

      regraspComm::PushInFingers msg;
      msg.frame_pose = pose;
      msg.zone = zone;
     
      regrasp_msg.pushInFingers = msg;
    }
  else if (action == ROLL_TO_FINGERTIP) // RollToFingertip
    {
      double roll_angle;
      node->getParam("/rollToFingertip_roll_angle", roll_angle);

      regraspComm::RollToFingertip msg;
      msg.roll_angle = roll_angle;
      
      regrasp_msg.rollToFingertip = msg;
    }
  else if (action == ROLL_ON_GROUND) // RollOnGround
    {
      double rollOnGround_position_x, 
	rollOnGround_position_y, rollOnGround_position_z, 
	rollOnGround_orientation_x, rollOnGround_orientation_y, 
	rollOnGround_orientation_z, rollOnGround_orientation_w, tcp, ori, rollAngle;
      node->getParam("/rollOnGround_position_x", rollOnGround_position_x);
      node->getParam("/rollOnGround_position_y", rollOnGround_position_y);
      node->getParam("/rollOnGround_position_z", rollOnGround_position_z);
      node->getParam("/rollOnGround_orientation_x", rollOnGround_orientation_x);
      node->getParam("/rollOnGround_orientation_y", rollOnGround_orientation_y);
      node->getParam("/rollOnGround_orientation_z", rollOnGround_orientation_z);
      node->getParam("/rollOnGround_orientation_w", rollOnGround_orientation_w);
      node->getParam("/rollOnGround_tcp", tcp);
      node->getParam("/rollOnGround_ori", ori);
      node->getParam("/rollOnGround_roll_angle", rollAngle);

      geometry_msgs::Pose pose;
      pose.position.x = rollOnGround_position_x;
      pose.position.y = rollOnGround_position_y;
      pose.position.z = rollOnGround_position_z;
      pose.orientation.x = rollOnGround_orientation_x;
      pose.orientation.y = rollOnGround_orientation_y;
      pose.orientation.z = rollOnGround_orientation_z;
      pose.orientation.w = rollOnGround_orientation_w;

      regraspComm::RollOnGround msg;
      //msg.roll_pose = roll_pose;
      msg.roll_place_pose = pose;
      msg.tcp = tcp;
      msg.ori = ori;
      msg.rollAngle = rollAngle;

      regrasp_msg.rollOnGround = msg;
    }
  else if (action == ROLL_TO_PALM) // RollToPalm
    {
      double sleep, speed;
      node->getParam("/rollToPalm_sleep", sleep);
      node->getParam("/rollToPalm_hand_speed", speed);

      regraspComm::RollToPalm msg;
      msg.sleep = sleep;
      msg.hand_speed = speed;

      regrasp_msg.rollToPalm = msg;
    }
  else if (action == ROLL_TO_GROUND) // RollToGround
    {
      double rollToGround_flip_position_x, rollToGround_flip_position_y, 
	rollToGround_flip_position_z, rollToGround_flip_orientation_x, 
	rollToGround_flip_orientation_y, rollToGround_flip_orientation_z, 
	rollToGround_flip_orientation_w, sleep, speed;
      node->getParam("/rollToGround_flip_position_x", rollToGround_flip_position_x);
      node->getParam("/rollToGround_flip_position_y", rollToGround_flip_position_y);
      node->getParam("/rollToGround_flip_position_z", rollToGround_flip_position_z);
      node->getParam("/rollToGround_flip_orientation_x", rollToGround_flip_orientation_x);
      node->getParam("/rollToGround_flip_orientation_y", rollToGround_flip_orientation_y);
      node->getParam("/rollToGround_flip_orientation_z", rollToGround_flip_orientation_z);
      node->getParam("/rollToGround_flip_orientation_w", rollToGround_flip_orientation_w);
      node->getParam("/rollToGround_sleep", sleep);
      node->getParam("/rollToGround_hand_speed", speed);

      geometry_msgs::Pose flip_pose;
      flip_pose.position.x = rollToGround_flip_position_x;
      flip_pose.position.y = rollToGround_flip_position_y;
      flip_pose.position.z = rollToGround_flip_position_z;
      flip_pose.orientation.x = rollToGround_flip_orientation_x;
      flip_pose.orientation.y = rollToGround_flip_orientation_y;
      flip_pose.orientation.z = rollToGround_flip_orientation_z;
      flip_pose.orientation.w = rollToGround_flip_orientation_w;

      regraspComm::RollToGround msg;
      msg.flip_pose = flip_pose;
      msg.sleep = sleep;
      msg.hand_speed = speed;

      regrasp_msg.rollToGround = msg;
    }
  else if (action == DROOP_IN_FINGERS) // DroopInFingers
    {
      double droopInFingers_pick_position_x, droopInFingers_pick_position_y, 
	droopInFingers_pick_position_z, droopInFingers_pick_orientation_x, 
	droopInFingers_pick_orientation_y, droopInFingers_pick_orientation_z, 
	droopInFingers_pick_orientation_w, angle;
      // , droopInFingers_pick2_position_x, 
      // 	droopInFingers_pick2_position_y, droopInFingers_pick2_position_z, 
      // 	droopInFingers_pick2_orientation_x, droopInFingers_pick2_orientation_y, 
      // 	droopInFingers_pick2_orientation_z, droopInFingers_pick2_orientation_w, 
      // 	droopInFingers_place_position_x, droopInFingers_place_position_y, 
      // 	droopInFingers_place_position_z, droopInFingers_place_orientation_x, 
      // 	droopInFingers_place_orientation_y, droopInFingers_place_orientation_z, 
      // 	droopInFingers_place_orientation_w, droopInFingers_place2_position_x, 
      // 	droopInFingers_place2_position_y, droopInFingers_place2_position_z, 
      // 	droopInFingers_place2_orientation_x, droopInFingers_place2_orientation_y, 
      // 	droopInFingers_place2_orientation_z, droopInFingers_place2_orientation_w, tcp, ori,

      node->getParam("/droopInFingers_pick_position_x", droopInFingers_pick_position_x);
      node->getParam("/droopInFingers_pick_position_y", droopInFingers_pick_position_y);
      node->getParam("/droopInFingers_pick_position_z", droopInFingers_pick_position_z);
      node->getParam("/droopInFingers_pick_orientation_x", droopInFingers_pick_orientation_x);
      node->getParam("/droopInFingers_pick_orientation_y", droopInFingers_pick_orientation_y);
      node->getParam("/droopInFingers_pick_orientation_z", droopInFingers_pick_orientation_z);
      node->getParam("/droopInFingers_pick_orientation_w", droopInFingers_pick_orientation_w);
      node->getParam("/droopInFingers_droopAngle", angle);

      geometry_msgs::Pose pose;
      pose.position.x = droopInFingers_pick_position_x;
      pose.position.y = droopInFingers_pick_position_y;
      pose.position.z = droopInFingers_pick_position_z;
      pose.orientation.x = droopInFingers_pick_orientation_x;
      pose.orientation.y = droopInFingers_pick_orientation_y;
      pose.orientation.z = droopInFingers_pick_orientation_z;
      pose.orientation.w = droopInFingers_pick_orientation_w;

      //      regraspComm::DroopInFingers msg;
      regraspComm::Droop msg;
      msg.grasp_pose = pose;
      //msg.droopAngle = angle;
      
      cout << "done getting params" << endl;

      regrasp_msg.droopInFingers = msg;
    }
  else if (action == THROW_TO_FINGERTIP) // ThrowToFingertip
    {
      double throwToFingertip_pose_position_x, throwToFingertip_pose_position_y, 
	throwToFingertip_pose_position_z, throwToFingertip_pose_orientation_x, 
	throwToFingertip_pose_orientation_y, throwToFingertip_pose_orientation_z, 
	throwToFingertip_pose_orientation_w, throwToFingertip_pose1_position_x, throwToFingertip_pose1_position_y, 
	throwToFingertip_pose1_position_z, throwToFingertip_pose1_orientation_x, 
	throwToFingertip_pose1_orientation_y, throwToFingertip_pose1_orientation_z, 
	throwToFingertip_pose1_orientation_w, throwToFingertip_pose2_position_x, throwToFingertip_pose2_position_y, 
	throwToFingertip_pose2_position_z, throwToFingertip_pose2_orientation_x, 
	throwToFingertip_pose2_orientation_y, throwToFingertip_pose2_orientation_z, 
	throwToFingertip_pose2_orientation_w, speed, fspeed, sleep, angle;
      node->getParam("/throwToFingertip_pose_position_x", throwToFingertip_pose_position_x);
      node->getParam("/throwToFingertip_pose_position_y", throwToFingertip_pose_position_y);
      node->getParam("/throwToFingertip_pose_position_z", throwToFingertip_pose_position_z);
      node->getParam("/throwToFingertip_pose_orientation_x", throwToFingertip_pose_orientation_x);
      node->getParam("/throwToFingertip_pose_orientation_y", throwToFingertip_pose_orientation_y);
      node->getParam("/throwToFingertip_pose_orientation_z", throwToFingertip_pose_orientation_z);
      node->getParam("/throwToFingertip_pose_orientation_w", throwToFingertip_pose_orientation_w);
      node->getParam("/throwToFingertip_pose1_position_x", throwToFingertip_pose1_position_x);
      node->getParam("/throwToFingertip_pose1_position_y", throwToFingertip_pose1_position_y);
      node->getParam("/throwToFingertip_pose1_position_z", throwToFingertip_pose1_position_z);
      node->getParam("/throwToFingertip_pose1_orientation_x", throwToFingertip_pose1_orientation_x);
      node->getParam("/throwToFingertip_pose1_orientation_y", throwToFingertip_pose1_orientation_y);
      node->getParam("/throwToFingertip_pose1_orientation_z", throwToFingertip_pose1_orientation_z);
      node->getParam("/throwToFingertip_pose1_orientation_w", throwToFingertip_pose1_orientation_w);
      node->getParam("/throwToFingertip_pose2_position_x", throwToFingertip_pose2_position_x);
      node->getParam("/throwToFingertip_pose2_position_y", throwToFingertip_pose2_position_y);
      node->getParam("/throwToFingertip_pose2_position_z", throwToFingertip_pose2_position_z);
      node->getParam("/throwToFingertip_pose2_orientation_x", throwToFingertip_pose2_orientation_x);
      node->getParam("/throwToFingertip_pose2_orientation_y", throwToFingertip_pose2_orientation_y);
      node->getParam("/throwToFingertip_pose2_orientation_z", throwToFingertip_pose2_orientation_z);
      node->getParam("/throwToFingertip_pose2_orientation_w", throwToFingertip_pose2_orientation_w);
      node->getParam("/throwToFingertip_speed", speed);
      node->getParam("/throwToFingertip_fspeed", fspeed);
      node->getParam("/throwToFingertip_sleep", sleep);
      node->getParam("/throwToFingertip_E2F_open_angle", angle);

      geometry_msgs::Pose pose;
      pose.position.x = throwToFingertip_pose_position_x;
      pose.position.y = throwToFingertip_pose_position_y;
      pose.position.z = throwToFingertip_pose_position_z;
      pose.orientation.x = throwToFingertip_pose_orientation_x;
      pose.orientation.y = throwToFingertip_pose_orientation_y;
      pose.orientation.z = throwToFingertip_pose_orientation_z;
      pose.orientation.w = throwToFingertip_pose_orientation_w;
      geometry_msgs::Pose pose1;
      pose1.position.x = throwToFingertip_pose1_position_x;
      pose1.position.y = throwToFingertip_pose1_position_y;
      pose1.position.z = throwToFingertip_pose1_position_z;
      pose1.orientation.x = throwToFingertip_pose1_orientation_x;
      pose1.orientation.y = throwToFingertip_pose1_orientation_y;
      pose1.orientation.z = throwToFingertip_pose1_orientation_z;
      pose1.orientation.w = throwToFingertip_pose1_orientation_w;
      geometry_msgs::Pose pose2;
      pose2.position.x = throwToFingertip_pose2_position_x;
      pose2.position.y = throwToFingertip_pose2_position_y;
      pose2.position.z = throwToFingertip_pose2_position_z;
      pose2.orientation.x = throwToFingertip_pose2_orientation_x;
      pose2.orientation.y = throwToFingertip_pose2_orientation_y;
      pose2.orientation.z = throwToFingertip_pose2_orientation_z;
      pose2.orientation.w = throwToFingertip_pose2_orientation_w;

      regraspComm::ThrowToFingertip msg;
      msg.pose = pose;
      msg.pose1 = pose1;
      msg.pose2 = pose2;
      msg.speed = speed;
      msg.fspeed = fspeed;
      msg.sleep = sleep;
      msg.partial_open_angle = angle;
      
      regrasp_msg.throwToFingertip = msg;
    }
  // TODO: i can't find where the param values are in the other regrasp_node
  else if (action == LIE_TO_STAND) // LieToStand
    {
      double lieToStand_x, lieToStand_y, lieToStand_z, lieToStand_rotate_angle;      
      node->getParam("/lieToStand_rotate_angle", lieToStand_rotate_angle);
      node->getParam("/lieToStand_x", lieToStand_x);
      node->getParam("/lieToStand_y", lieToStand_y);
      node->getParam("/lieToStand_z", lieToStand_z);

      regraspComm::LieToStand msg;
      msg.rotate_angle = lieToStand_rotate_angle;
      msg.x = lieToStand_x;
      msg.y = lieToStand_y;
      msg.z = lieToStand_z;
      regrasp_msg.lieToStand = msg;

    }
  // TODO: i can't find where the param values are in the other regrasp_node
  else if (action == STAND_TO_LIE) //StandToLie
    {
      double standToLie_x, standToLie_y, standToLie_z, standToLie_rotate_angle;      
      node->getParam("/standToLie_rotate_angle", standToLie_rotate_angle);
      node->getParam("/standToLie_x", standToLie_x);
      node->getParam("/standToLie_y", standToLie_y);
      node->getParam("/standToLie_z", standToLie_z);

      regraspComm::StandToLie msg;
      msg.rotate_angle = standToLie_rotate_angle;
      msg.x = standToLie_x;
      msg.y = standToLie_y;
      msg.z = standToLie_z;
      regrasp_msg.standToLie = msg;
    }
  else if (action == LONG_EDGE_DROOP)
    {
      double longEdgeDroop_pick_position_x, longEdgeDroop_pick_position_y, 
	longEdgeDroop_pick_position_z, longEdgeDroop_pick_orientation_x, 
	longEdgeDroop_pick_orientation_y, longEdgeDroop_pick_orientation_z, 
	longEdgeDroop_pick_orientation_w;
      node->getParam("/longEdgeDroop_pick_position_x", longEdgeDroop_pick_position_x);
      node->getParam("/longEdgeDroop_pick_position_y", longEdgeDroop_pick_position_y);
      node->getParam("/longEdgeDroop_pick_position_z", longEdgeDroop_pick_position_z);
      node->getParam("/longEdgeDroop_pick_orientation_x", longEdgeDroop_pick_orientation_x);
      node->getParam("/longEdgeDroop_pick_orientation_y", longEdgeDroop_pick_orientation_y);
      node->getParam("/longEdgeDroop_pick_orientation_z", longEdgeDroop_pick_orientation_z);
      node->getParam("/longEdgeDroop_pick_orientation_w", longEdgeDroop_pick_orientation_w);

      geometry_msgs::Pose pick_pose;
      pick_pose.position.x = longEdgeDroop_pick_position_x;
      pick_pose.position.y = longEdgeDroop_pick_position_y;
      pick_pose.position.z = longEdgeDroop_pick_position_z;
      pick_pose.orientation.x = longEdgeDroop_pick_orientation_x;
      pick_pose.orientation.y = longEdgeDroop_pick_orientation_y;
      pick_pose.orientation.z = longEdgeDroop_pick_orientation_z;
      pick_pose.orientation.w = longEdgeDroop_pick_orientation_w;

      regraspComm::LongEdgeDroop msg;
      msg.pick_pose = pick_pose;

      regrasp_msg.longEdgeDroop = msg;
    }
  else if (action == SHORT_EDGE_DROOP)
    {
      double shortEdgeDroop_pick_position_x, shortEdgeDroop_pick_position_y, 
	shortEdgeDroop_pick_position_z, shortEdgeDroop_pick_orientation_x, 
	shortEdgeDroop_pick_orientation_y, shortEdgeDroop_pick_orientation_z, 
	shortEdgeDroop_pick_orientation_w;
      node->getParam("/shortEdgeDroop_pick_position_x", shortEdgeDroop_pick_position_x);
      node->getParam("/shortEdgeDroop_pick_position_y", shortEdgeDroop_pick_position_y);
      node->getParam("/shortEdgeDroop_pick_position_z", shortEdgeDroop_pick_position_z);
      node->getParam("/shortEdgeDroop_pick_orientation_x", shortEdgeDroop_pick_orientation_x);
      node->getParam("/shortEdgeDroop_pick_orientation_y", shortEdgeDroop_pick_orientation_y);
      node->getParam("/shortEdgeDroop_pick_orientation_z", shortEdgeDroop_pick_orientation_z);
      node->getParam("/shortEdgeDroop_pick_orientation_w", shortEdgeDroop_pick_orientation_w);

      geometry_msgs::Pose pick_pose;
      pick_pose.position.x = shortEdgeDroop_pick_position_x;
      pick_pose.position.y = shortEdgeDroop_pick_position_y;
      pick_pose.position.z = shortEdgeDroop_pick_position_z;
      pick_pose.orientation.x = shortEdgeDroop_pick_orientation_x;
      pick_pose.orientation.y = shortEdgeDroop_pick_orientation_y;
      pick_pose.orientation.z = shortEdgeDroop_pick_orientation_z;
      pick_pose.orientation.w = shortEdgeDroop_pick_orientation_w;

      regraspComm::ShortEdgeDroop msg;
      msg.pick_pose = pick_pose;

      regrasp_msg.shortEdgeDroop = msg;
    }
  else if (action == TOPPLE)
    {
      double topple_position_x, topple_position_y, 
	topple_position_z, topple_orientation_x, 
	topple_orientation_y, topple_orientation_z, 
	topple_orientation_w;
      int dir_x, dir_y, dir_z;
      node->getParam("/topple_position_x", topple_position_x);
      node->getParam("/topple_position_y", topple_position_y);
      node->getParam("/topple_position_z", topple_position_z);
      node->getParam("/topple_orientation_x", topple_orientation_x);
      node->getParam("/topple_orientation_y", topple_orientation_y);
      node->getParam("/topple_orientation_z", topple_orientation_z);
      node->getParam("/topple_orientation_w", topple_orientation_w);
      node->getParam("/topple_dir_x", dir_x);
      node->getParam("/topple_dir_y", dir_y);
      node->getParam("/topple_dir_z", dir_z);
 
      geometry_msgs::Pose pose;
      pose.position.x = topple_position_x;
      pose.position.y = topple_position_y;
      pose.position.z = topple_position_z;
      pose.orientation.x = topple_orientation_x;
      pose.orientation.y = topple_orientation_y;
      pose.orientation.z = topple_orientation_z;
      pose.orientation.w = topple_orientation_w;

      regraspComm::Topple msg;
      msg.pose = pose;
      msg.dir_x = dir_x;
      msg.dir_y = dir_y;
      msg.dir_z = dir_z;
     
      regrasp_msg.topple = msg;
    }

  
  return regrasp_msg;
}
