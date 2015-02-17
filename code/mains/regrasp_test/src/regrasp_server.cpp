#include <ros/ros.h>
#include "regrasp_test/regrasps.h"
//#include "regrasp_test/hand_tools.h"
//#include "regrasp_test/BouncelessF2E.h"
//#include "regrasp_test/PushTranslate.h"
#include "regrasp_test/Regrasp.h"
#include "regrasp_test/RegraspService.h"

class RegraspServer
{
  Regrasps regrasps;

public:
  RegraspServer(RobotComm r, HandComm h);
  bool execute(regrasp_test::RegraspService::Request &req, 
               regrasp_test::RegraspService::Response &res);
};

RegraspServer::RegraspServer(RobotComm r, HandComm h)
{
  regrasps.init(r,h);
}

bool RegraspServer::execute(regrasp_test::RegraspService::Request &req, 
			    regrasp_test::RegraspService::Response &res)
{
  bool success = true;
  for (int i=0; i<req.regrasps.size();i++)
    {
      int action = req.regrasps[i].action;
      if (action == 0)
	{
	  if (!regrasps.pick(req.regrasps[i].pick)) 
	    success = false;
	}
      if (action == 1)
	{
	  if (!regrasps.place(req.regrasps[i].place))
	    success = false;
	}
      if (action == 2)
	{
	  if (!regrasps.bouncelessF2E(req.regrasps[i].bouncelessF2E))
	    success = false;
	}
      if (action == 3)
	{
	  if(!regrasps.pushTranslate(req.regrasps[i].pushTranslate))
	    success = false;
	}
      if (action == 4)
	{
	  if (!regrasps.vibrate(req.regrasps[i].vibrate))
	    success = false;
	}
      if (action == 5)
	{
	  if (!regrasps.topple(req.regrasps[i].topple))
	    success = false;
	}
      if (action == 6)
	{
	  if (!regrasps.rotateInFingers(req.regrasps[i].rotateInFingers))
	    success = false;
	}
      if (action == 7)
	{
	  if (!regrasps.rollInHand(req.regrasps[i].rollInHand))
	    success = false;
	}
      if (action == 8)
	{
	  if (!regrasps.groundRoll(req.regrasps[i].groundRoll))
	    success = false;
	}
      if (action == 9)
	{
	  if (!regrasps.flipInHand(req.regrasps[i].flipInHand))
	    success = false;
	}
      if (action == 10)
	{
	  if (!regrasps.flipInAir(req.regrasps[i].flipInAir))
	    success = false;
	}

      if (!success)
	break;
    }
  res.success = success;

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "regrasp_server");
  ros::NodeHandle nh;

  RobotComm robot(&nh);
  HandComm hand(&nh);

  RegraspServer server(robot, hand);

  ros::ServiceServer service = nh.advertiseService("regrasp", 
                                                   &RegraspServer::execute, 
                                                   &server);
  ROS_INFO("Ready to execute a regrasp.");
  ros::spin();

  return 0;
}
