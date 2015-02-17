// echo_force.cpp : test program for receiving force data from ATI mini-40 mounted on the table

#include <ros/ros.h>

// MLAB node for driving the ABB 
// #include <robot_comm/robot_comm.h>

// MLAB library for kinematic calculations
// #include <matVec/matVec.h>

// The Net F/T driver reports the sensor state using a standard
// message type WrenchStamped, by default on topic /netft_data.

// See /opt/ros/*/stacks/common_msgs/geometry_msgs/msg_gen/cpp/include/geometry_msgs/WrenchStamped.h
#include <geometry_msgs/WrenchStamped.h>

/****************************************************************/
// Callback function for force data.
static void forceDataCallback( const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  printf("Z force: %f\n", msg->wrench.force.z );
}

/****************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "echoForce");
  ros::NodeHandle node;

  ros::Subscriber forceDataSub = node.subscribe("/netft_data", 100, forceDataCallback );

  ROS_INFO("Entering force reporting loop...\n");
  int rate = 10; // Hz
  static ros::Rate loop_rate(rate);

  while ( ros::ok() ) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  ROS_INFO("Demo completed.");

  return 0;
}
/****************************************************************/
