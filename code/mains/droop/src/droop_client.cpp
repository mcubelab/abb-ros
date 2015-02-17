#include "ros/ros.h"
#include "droop/Pull.h"
#include "droop/Push.h"
#include "droop/Play.h"
#include "droop/Pick.h"
#include "droop/Polyhedron.h"
#include <geometry_msgs/Pose.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "droop_client");

  ros::NodeHandle n;
  // //ros::ServiceClient client = n.serviceClient<droop::Pull>("droop_pull");
  // ros::ServiceClient client = n.serviceClient<droop::Push>("droop_push");
  // droop::Pull srv;
  // srv.request.obj = RecObj::BIG_TRIANGLE;
  // //srv.request.goal = 2; 
  // srv.request.goal = 1; 
  // if (client.call(srv))
  //   cout << "done? " << srv.response.ret << endl;
  // else
  // {
  //   ROS_ERROR("Failed to call service droop");
  //   return 1;
  // }


  // ros::ServiceClient client = n.serviceClient<droop::Pick>("droop_pick");
  // droop::Pick srv;
  // srv.request.p = 65; 
  // if (client.call(srv))
  //   cout << "done? " << srv.response.ret << endl;
  // else
  // {
  //   ROS_ERROR("Failed to call service droop");
  //   return 1;
  // }

  ros::ServiceClient client = n.serviceClient<droop::Play>("droop_play");
  droop::Play srv;
  srv.request.obj = RecObj::BIG_TRIANGLE;
  //srv.request.goal = 2; 
  srv.request.goal = 1; 
  if (client.call(srv))
    cout << "done? " << srv.response.ret << endl;
  else
  {
    ROS_ERROR("Failed to call service droop");
    return 1;
  }

  return 0;
}
