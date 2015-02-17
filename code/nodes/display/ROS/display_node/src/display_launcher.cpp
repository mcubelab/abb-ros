#include "ros/ros.h"
int main(int argc, char *argv[])
{
ros::init(argc, argv, "dummy");
ROS_INFO("Shutting down dummy node...");
return 0;
}
