#include "ros/ros.h"
#include "regrasp_test/hand_tools.h"
#include "regrasp_test/BouncelessF2E.h"
#include "regrasp_test/PushTranslate.h"
#include "regrasp_test/Regrasp.h"
#include "regrasp_test/RegraspService.h"
#include "regrasp_test/regrasps.h"

class RegraspHand
{

  ros::ServiceClient client;

public:
  RegraspHand(ros::NodeHandle nh);

  bool setupHand();
  bool run();
  bool pickAndPlace(geometry_msgs::Pose pick, 
		    geometry_msgs::Pose place);
  bool bouncelessThrow();
  bool pushTranslate();
  bool vibrate();
  bool topple();
  bool rotateInFingers();
  bool rollInHand();
  bool groundRoll();
  bool flipInHand();
  bool flipInAir();
  bool vertCyl();
  bool E2F();

};

RegraspHand::RegraspHand(ros::NodeHandle nh)
{
  client = nh.serviceClient<regrasp_test::RegraspService>("regrasp");
}

bool RegraspHand::run()
{

  geometry_msgs::Pose pick;
  pick.position.x = 600.0;
  pick.position.y = 200.0;
  pick.position.z = 100.0;
  pick.orientation.x = 0.0;
  pick.orientation.y = 0.707;
  pick.orientation.z = 0.707;
  pick.orientation.w = 0.0;  

  geometry_msgs::Pose place;
  place.position.x = 600.0;
  place.position.y = 300.0;
  place.position.z = 100.0;
  place.orientation.x = 0.0;
  place.orientation.y = 0.707;
  place.orientation.z = 0.707;
  place.orientation.w = 0.0;  

  //pickAndPlace(pick,place);
  //vibrate();
  //pushTranslate();
  //flipInHand();
  //flipInAir();
  //rollInHand();
  //rotateInFingers();
  topple();

  return 1;
}

bool RegraspHand::pickAndPlace(geometry_msgs::Pose pick, 
			       geometry_msgs::Pose place)
{
  // Pick object
  regrasp_test::Pick pick_msg;
  pick_msg.pose = pick;
  pick_msg.open_angle = 20.0;
  pick_msg.close_angle = 30.0;

  regrasp_test::Regrasp pick_regrasp;
  pick_regrasp.action = 0;
  pick_regrasp.pick = pick_msg;

  // Place object
  regrasp_test::Place place_msg;
  place_msg.pose = place;
  place_msg.open_angle = 20.0;

  regrasp_test::Regrasp place_regrasp;
  place_regrasp.action = 1;
  place_regrasp.place = place_msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = pick_regrasp;
  srv.request.regrasps[1] = place_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;

}

bool RegraspHand::bouncelessThrow()
{
  regrasp_test::BouncelessF2E msg;

  // setup pose for msg/BouncelessF2E.setup_pose
  geometry_msgs::Pose init_pose;
  init_pose.position.x = 600.0;
  init_pose.position.y = 300.0;
  init_pose.position.z = 200.0;
  init_pose.orientation.x = 0.0;
  init_pose.orientation.y = 0.707;
  init_pose.orientation.z = 0.707;
  init_pose.orientation.w = 0.0;
  msg.setup_pose = init_pose;

  regrasp_test::Regrasp throw_regrasp;
  throw_regrasp.action = 2;
  throw_regrasp.bouncelessF2E = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = throw_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;

}

bool RegraspHand::pushTranslate()
{
  regrasp_test::PushTranslate msg;

  geometry_msgs::Pose safe_pose;
  safe_pose.position.x = 600.0;
  safe_pose.position.y = 200.0;
  safe_pose.position.z = 200.0;
  safe_pose.orientation.x = 0.0;
  safe_pose.orientation.y = 0.707;
  safe_pose.orientation.z = 0.707;
  safe_pose.orientation.w = 0.0;
  geometry_msgs::Pose push_stop_pose;
  push_stop_pose.position.x = 600.0;
  push_stop_pose.position.y = 200.0;
  push_stop_pose.position.z = 72.0;
  push_stop_pose.orientation.x = 0.0;
  push_stop_pose.orientation.y = 0.707;
  push_stop_pose.orientation.z = 0.707;
  push_stop_pose.orientation.w = 0.0;
  msg.safe_pose = safe_pose;
  msg.push_stop_pose = push_stop_pose;

  regrasp_test::Regrasp pushTranslate_regrasp;
  pushTranslate_regrasp.action = 3;
  pushTranslate_regrasp.pushTranslate = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = pushTranslate_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::vibrate()
{
  regrasp_test::Vibrate msg;
  msg.vibrate_angle = 43.0;
  
  regrasp_test::Regrasp vibrate_regrasp;
  vibrate_regrasp.action = 4;
  vibrate_regrasp.vibrate = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = vibrate_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::topple()
{
  regrasp_test::Topple msg;
  msg.sleep = 83000;
  geometry_msgs::Pose pose;
  pose.position.x = 600.0;
  pose.position.y = 700.0;
  pose.position.z = 800.0; //1100
  pose.orientation.x = 0.707;
  pose.orientation.y = 0.0;
  pose.orientation.z = 0.0;
  pose.orientation.w = -0.707;
  msg.pose = pose;
  
  regrasp_test::Regrasp topple_regrasp;
  topple_regrasp.action = 5;
  topple_regrasp.topple = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = topple_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::rotateInFingers()
{
  regrasp_test::RotateInFingers msg;
  msg.zone = 1;
  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = 0.0; 
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.0;
  msg.push_pose = pose;
  
  regrasp_test::Regrasp rotate_regrasp;
  rotate_regrasp.action = 6;
  rotate_regrasp.rotateInFingers = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = rotate_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::rollInHand()
{
  regrasp_test::RollInHand msg;
  msg.roll_angle = 10.0;
  geometry_msgs::Pose safe_pose;
  safe_pose.position.x = 700.0;
  safe_pose.position.y = 325.0;
  safe_pose.position.z = 810.0; 
  safe_pose.orientation.x = 1.0;
  safe_pose.orientation.y = 0.0;
  safe_pose.orientation.z = 0.0;
  safe_pose.orientation.w = 0.0;
  msg.safe_pose = safe_pose;
  geometry_msgs::Pose push_pose;
  push_pose.position.x = 700.0;
  push_pose.position.y = 325.0;
  push_pose.position.z = 810.0; 
  push_pose.orientation.x = 1.0;
  push_pose.orientation.y = 0.0;
  push_pose.orientation.z = 0.0;
  push_pose.orientation.w = 0.0;
  msg.push_pose = push_pose;
  geometry_msgs::Pose push_intm_pose;
  push_intm_pose.position.x = 700.0;
  push_intm_pose.position.y = 325.0;
  push_intm_pose.position.z = 810.0; 
  push_intm_pose.orientation.x = 0.0;
  push_intm_pose.orientation.y = 0.707;
  push_intm_pose.orientation.z = 0.0;
  push_intm_pose.orientation.w = 0.707;
  msg.push_intm_pose = push_intm_pose;
  geometry_msgs::Pose push_inv_pose;
  push_inv_pose.position.x = 700.0;
  push_inv_pose.position.y = 325.0;
  push_inv_pose.position.z = 810.0; 
  push_inv_pose.orientation.x = 0.0;
  push_inv_pose.orientation.y = 0.0;
  push_inv_pose.orientation.z = 0.0;
  push_inv_pose.orientation.w = 1.0;
  msg.push_inv_pose = push_inv_pose;
  
  regrasp_test::Regrasp roll_regrasp;
  roll_regrasp.action = 7;
  roll_regrasp.rollInHand = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = roll_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::groundRoll()
{
  regrasp_test::GroundRoll msg;
  msg.tcp = 40.0;
  msg.ori = 20.0;
  geometry_msgs::Pose roll_pose;
  roll_pose.position.x = 650.0;
  roll_pose.position.y = 200.0;
  roll_pose.position.z = 67.5; 
  roll_pose.orientation.x = 0.0;
  roll_pose.orientation.y = 0.707;
  roll_pose.orientation.z = 0.707;
  roll_pose.orientation.w = 0.0;
  msg.roll_pose = roll_pose;  
  geometry_msgs::Pose roll_place_pose;
  roll_place_pose.position.x = 705.0;
  roll_place_pose.position.y = 200.0;
  roll_place_pose.position.z = 67.5; 
  roll_place_pose.orientation.x = 0.0;
  roll_place_pose.orientation.y = 0.707;
  roll_place_pose.orientation.z = 0.707;
  roll_place_pose.orientation.w = 0.0;
  msg.roll_place_pose = roll_place_pose;

  regrasp_test::Regrasp roll_regrasp;
  roll_regrasp.action = 8;
  roll_regrasp.groundRoll = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = roll_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::flipInHand()
{
  regrasp_test::FlipInHand msg;
  msg.sleep = 670000;
  
  regrasp_test::Regrasp flip_regrasp;
  flip_regrasp.action = 9;
  flip_regrasp.flipInHand = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = flip_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::flipInAir()
{
  regrasp_test::FlipInAir msg;
  msg.sleep = 570000;
  geometry_msgs::Pose pose;
  pose.position.x = 612.0;
  pose.position.y = 188.0;
  pose.position.z = 132.0; //1100
  pose.orientation.x = 0.487;
  pose.orientation.y = 0.720;
  pose.orientation.z = 0.410;
  pose.orientation.w = -0.275;
  msg.flip_pose = pose;

  regrasp_test::Regrasp flip_regrasp;
  flip_regrasp.action = 10;
  flip_regrasp.flipInAir = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = flip_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::vertCyl()
{
  regrasp_test::VertCyl msg;
  msg.tcp = 20.0;
  msg.ori = 10.0;
  geometry_msgs::Pose pick_pose;
  pick_pose.position.x = 600.0;
  pick_pose.position.y = 235.0;
  pick_pose.position.z = 79.5; //1100
  pick_pose.orientation.x = 0.0;
  pick_pose.orientation.y = 0.707;
  pick_pose.orientation.z = 0.707;
  pick_pose.orientation.w = 0.0;
  msg.pick_pose = pick_pose;
  geometry_msgs::Pose pick2_pose;
  pick2_pose.position.x = 600.0;
  pick2_pose.position.y = 235.0;
  pick2_pose.position.z = 194.0; //1100
  pick2_pose.orientation.x = 0.0;
  pick2_pose.orientation.y = 0.707;
  pick2_pose.orientation.z = 0.707;
  pick2_pose.orientation.w = 0.0;
  msg.pick2_pose = pick2_pose;
  geometry_msgs::Pose place_pose;
  place_pose.position.x = 600.0;
  place_pose.position.y = 200.0;
  place_pose.position.z = 78.5; //1100
  place_pose.orientation.x = 0.0;
  place_pose.orientation.y = 0.707;
  place_pose.orientation.z = 0.707;
  place_pose.orientation.w = 0.0;
  msg.place_pose = place_pose;
  geometry_msgs::Pose place2_pose;
  place2_pose.position.x = 593.0;
  place2_pose.position.y = 207.0;
  place2_pose.position.z = 58.5; //1100
  place2_pose.orientation.x = 0.0;
  place2_pose.orientation.y = 0.707;
  place2_pose.orientation.z = 0.707;
  place2_pose.orientation.w = 0.0;
  msg.place2_pose = place2_pose;

  regrasp_test::Regrasp vertcyl_regrasp;
  vertcyl_regrasp.action = 11;
  vertcyl_regrasp.vertCyl = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = vertcyl_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

bool RegraspHand::E2F()
{
  regrasp_test::E2F msg;
  msg.speed = 0.9;
  msg.fspeed = 0.78;
  msg.sleep = 500;
  msg.partial_open_angle = 15.0;
  geometry_msgs::Pose pose;
  pose.position.x = 593.0;
  pose.position.y = 207.0;
  pose.position.z = 58.5; //1100
  pose.orientation.x = 0.0;
  pose.orientation.y = 0.707;
  pose.orientation.z = 0.707;
  pose.orientation.w = 0.0;
  msg.pose = pose;
  geometry_msgs::Pose pose1;
  pose1.position.x = 593.0;
  pose1.position.y = 207.0;
  pose1.position.z = 58.5; //1100
  pose1.orientation.x = 0.0;
  pose1.orientation.y = 0.707;
  pose1.orientation.z = 0.707;
  pose1.orientation.w = 0.0;
  msg.pose1 = pose1;
  geometry_msgs::Pose pose2;
  pose2.position.x = 593.0;
  pose2.position.y = 207.0;
  pose2.position.z = 58.5; //1100
  pose2.orientation.x = 0.0;
  pose2.orientation.y = 0.707;
  pose2.orientation.z = 0.707;
  pose2.orientation.w = 0.0;
  msg.pose2 = pose2;

  regrasp_test::Regrasp e2f_regrasp;
  e2f_regrasp.action = 12;
  e2f_regrasp.E2F = msg;

  regrasp_test::RegraspService srv;
  srv.request.regrasps[0] = e2f_regrasp;

  if (client.call(srv))
    ROS_INFO("Regrasp success: %d", srv.response.success);
  else
    {
      ROS_INFO("Failed to call service regrasp");
      return 0;
    }

  return 1;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "regrasp_hand");
  ros::NodeHandle nh;
  ROS_INFO("Starting Simple Hand Regrasp Node...");

  RegraspHand hand(nh);
  hand.run();
}
