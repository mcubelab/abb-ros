#include "Stoc_opt.h"
#include <iostream>
#include <stdio.h>

//#define PI 3.14159265
#define BB_COVER 1
#define ShieldCan 2

/*"

 add parameters to motion program

 run motion program
 
read USER entry 0/1

Send answer to matlab
}

print optimized parameters

*/
int Assembly;
MatlabComm _matlab; 
RobotComm _robot;

bool Debug()
{
	char reply;
	printf("\n Enter Y or N");
	cin >> reply;
	if('Y' == reply || 'y' == reply)
	{
		return true;
	}
	else
	{
		return false;
	}
}
int main(int argc,char *argv[])
{ 

	ros::init(argc, argv, "Stoc_opt");
	ros::NodeHandle node;
	_robot.subscribe(&node);
	while(!_robot.Ping());

	_matlab.subscribe(&node);

	Vec nextP(nParams,0);
	Vec maxP(nParams,0);
	



	//First we Set up Robot Parameters
	
	if(!Set_Up_Robot(2))
	{	
		printf("Robot Not Setup");
		return 0;
	}
	printf("\n Setup of Robot Complete");
	//Next We Set up Optimization Parameters
	Set_Up_Optimization();
	printf("\n Setup of matlab Complete");
	int _iter_no = 0;

	while(ros::ok())
	{
		_iter_no++;
		_matlab.sendCommand("Stoc_opt_Learner");
		nextP = _matlab.getVec("nextP");
		printf("\n Attempt no %d for values %lf  and %lf",_iter_no,nextP[0],nextP[1]);

		if(Attempt_Assembly(nextP))
		//if(Debug())
		{
			_matlab.sendValue("userin",1);
		}
		else
		{
			_matlab.sendValue("userin",0);
		}
	}
	
	return 0;
}

bool Attempt_Assembly(Vec params)
{
	char reply;
	if(Assembly == BB_COVER)
	{

		Attempt_BB_Cover(params);
	}
	else if(Assembly == ShieldCan)
	{
		
		Attempt_ShieldCan(params);
	}
	printf("\n Was the Assembly Successful [Y/N]?");

	cin >> reply;	
	if('Y' == reply || 'y' == reply)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Attempt_BB_Cover(Vec params)
{
	int tilt =5;
	Vec trans(3);
	Quaternion orientation;
	HomogTransf  location ,newLocation;
	printf("\n Robot Setup \n");
	RotMat RotX;
	RotX.rotX(tilt);
	_robot.SetVacuum(1);
	printf("\n Vacuum on \n");
	char input_debug;

	
	cin>>input_debug;
	// We are positioned to take a snapshot of initial condition
	trans[0] = 60.0; 	trans[1] = 150.0; 	trans[2] = 100.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);
	


	_robot.SetVacuum(0);
	cin>>input_debug;

	// Next we move 60mm forward

	trans[0] = 0.0; 	trans[1] = 150.0; 	trans[2] = 100.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Next we move 100mm downwards to pick up the cover
	trans[0] = 0.0; 	trans[1] = 150.0; 	trans[2] = -25.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// After Picking up the cover we move  100mm up

	trans[0] = 0.0; 	trans[1] = 150.0; 	trans[2] = 100.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Now we move to position above the fixture
	//Initial Pos
	trans[0] = 0.0; 	trans[1] = 0.0; 	trans[2] = 100.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Now we rotate by angle RotX

	trans[0] = 11.662; 	trans[1] = -0.192; 	trans[2] = 11.283;
	orientation = RotX.getQuaternion();
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// We reach Step1
	trans[0] = 11.662; 	trans[1] = -0.192; 	trans[2] = 11.283;
	orientation = RotX.getQuaternion();
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);;


	// Now we reach Step2
	trans[0] = 12.196; 	trans[1] = -0.192; 	trans[2] = 4.920;
	orientation = RotX.getQuaternion();
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Step3
	trans[0] = 2.753; 	trans[1] = 0.068; 	trans[2] = 4.920;
	orientation = RotX.getQuaternion();
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);
	
	// Step 4 
	
	trans[0] = 3.0; 	trans[1] = 0.0; 	trans[2] = -1.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Step 5

	trans[0] = -0.50; 	trans[1] = 0.0; 	trans[2] = -1.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

	// Retract

    trans[0] = 0.0; 	trans[1] = 0.0; 	trans[2] = 100.0;
	orientation[0] = 1.0; 	orientation[1] = 0.0;	orientation[2] = 0.0;	orientation[3] = 0.0;
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);

}

void Homog2RobotSrv2Ros (HomogTransf homog)
{

	char input;
	Vec trans = homog.getTranslation();
	Quaternion orientation = homog.getRotation().getQuaternion();
	_robot.SetCartesian(trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);


}
void Attempt_ShieldCan(Vec param)
{	
	Vec trans(3);
	Quaternion orientation;
	RotMat RotX, RotY;
	HomogTransf newLocation ;

	HomogTransf location(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("15.0 -15.0 -15.0",3)); //actual location
	HomogTransf cornerOffset(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("-20.0 -12.0 0.0",3));
	
	//printf("\n Verifying param0 = %lf and param1 - %lf",param[0],param[1]);
	//RotX.rotX((5*PI/180.0));
	//RotY.rotY((4.0*PI/180.0));
	RotX.rotX((param[0]*PI/180.0));
	RotY.rotY((param[1]*PI/180.0));
	HomogTransf rotation(RotX*RotY,Vec("0.0 0.0 0.0",3));

	newLocation = cornerOffset*rotation*cornerOffset.inv()*location;
	trans =newLocation .getTranslation();
	orientation = newLocation.getRotation().getQuaternion();
	
	//printf("\n Start here");
	Homog2RobotSrv2Ros(newLocation);
	location = newLocation;


	//printf("\n Get Ready for Assembly");
	HomogTransf Down(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("4.0 8.0 10.5",3));
	newLocation=location*Down;
	Homog2RobotSrv2Ros(newLocation);
	location = newLocation;



	//printf("\n Go down until the corner touches the shield can with an offsetfrom the actual corner\n");	
	orientation = newLocation.getRotation().getQuaternion();
	trans[0] = 8;trans[1] = -8;trans[2] =-4.5;
	//printf(" \n  pos = %lf %lf %lf , quat = %lf %lf %lf %lf ",trans[0],trans[1],trans[2],orientation[0],orientation[1],orientation[2],orientation[3]);
	newLocation  = HomogTransf(orientation.getRotMat(),trans);
	Homog2RobotSrv2Ros(newLocation);
	location = newLocation;
	
	
	orientation = newLocation.getRotation().getQuaternion();
	trans[0] = 8.0; 	trans[1] = -8.0; 	trans[2] = -1.0;
	newLocation  = HomogTransf(orientation.getRotMat(),trans);
	Homog2RobotSrv2Ros(newLocation);
	location = newLocation;
	
	//Now we retract to the ideal position

	//printf("\n Sweep the Shield Can to the Corner\n");
	HomogTransf Back(orientation.getRotMat(),Vec("1.0 -1.0 -1.0",3));
	newLocation= Back;
 	

	Homog2RobotSrv2Ros(newLocation);
	location = newLocation;

	//Now we undo the rotation and we should be in the origin of the WorkObject
	HomogTransf Origin(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 0.0",3));	
	Homog2RobotSrv2Ros(Origin);
	//printf("Undo the rotation and we should be in the origin of the WorkObject\n");
	
	location = Origin;

	//Now we need to do a rolling to push into the 4 corners
	//Corner +X-Y
	

	//RotX.rotX((2.0*PI/180.0));
	//RotY.rotY((-1.0*PI/180.0));
	RotX.rotX((5.0*PI/180.0));
	RotY.rotY((-2.0*PI/180.0));
	rotation = HomogTransf(RotX*RotY,Vec("0.0 0.0 0.0",3));
	newLocation = cornerOffset*rotation*cornerOffset.inv()*location;
	Homog2RobotSrv2Ros(newLocation);

	//printf("Rolling to push into the 4 corners -> +X-Y\n");
	

	//Corner +X+Y
	
	//RotX.rotX((2.0*PI/180.0));
	//RotY.rotY((1.0*PI/180.0));
	RotX.rotX((5.0*PI/180.0));
	RotY.rotY((2.0*PI/180.0));
	rotation = HomogTransf(RotX*RotY,Vec("0.0 0.0 0.0",3));
	newLocation = cornerOffset*rotation*cornerOffset.inv()*location;
	Homog2RobotSrv2Ros(newLocation);

	//printf("Rolling to push into the 4 corners -> +X+Y\n");
	
	
	
	//Corner -X+Y
	
	//RotX.rotX((-2.0*PI/180.0));
	//RotY.rotY((1.0*PI/180.0));
	RotX.rotX((-5.0*PI/180.0));
	RotY.rotY((2.0*PI/180.0));
	rotation = HomogTransf(RotX*RotY,Vec("0.0 0.0 0.0",3));
	newLocation = cornerOffset*rotation*cornerOffset.inv()*location;
	Homog2RobotSrv2Ros(newLocation);
	//Homog2RobotSrv2Ros(rotation);
	//printf("Rolling to push into the 4 corners -> -X+Y\n");
	
	
	//Corner -X-Y
	//RotX.rotX((-2.0*PI/180.0));
	//RotY.rotY((-1.0*PI/180.0));
	RotX.rotX((-5.5*PI/180.0));
	RotY.rotY((-3.1*PI/180.0));
	rotation = HomogTransf(RotX*RotY,Vec("0.0 0.0 0.0",3));
	newLocation = cornerOffset*rotation*cornerOffset.inv()*location;
	Homog2RobotSrv2Ros(newLocation);
	//Homog2RobotSrv2Ros(rotation);
	//printf("Rolling to push into the 4 corners -> -X-Y\n");
		
	//Finally we push down vertically
	
  	HomogTransf Push(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 3.5",3));
	Homog2RobotSrv2Ros(Push);	
	//printf("Push down vertically\n");
	
	
	HomogTransf Pull(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 -59.0",3));
	Homog2RobotSrv2Ros(Pull);
	//printf("Pull vertically\n");


}

void Set_Up_Optimization()
{	
	printf("\n Setting up Matlab");
	_matlab.sendCommand("clear");
	_matlab.sendCommand("clc");
	_matlab.sendValue("nParams", nParams);
	_matlab.sendValue("nMin", nMin);
	_matlab.sendValue("nMax", nMax);

	switch(nParams)
	{
	case 1:
		_matlab.sendValue("MaxA1", MaxA1);
		_matlab.sendValue("MinA1", MinA1);
		break;
	case 2:
		_matlab.sendValue("MaxA1", MaxA1);
		_matlab.sendValue("MaxA2", MaxA2);
		_matlab.sendValue("MinA1", MinA1);
		_matlab.sendValue("MinA2", MinA2);
		break;
	case 3:
		_matlab.sendValue("MaxA1", MaxA1);
		_matlab.sendValue("MaxA2", MaxA2);
		_matlab.sendValue("MinA1", MinA1);
		_matlab.sendValue("MinA2", MinA2);
		_matlab.sendValue("MaxA3", MaxA3);
		_matlab.sendValue("MinA3", MinA3);
	}

	_matlab.sendValue("grid", grid);
}

bool Set_Up_Robot(int assembly_selection)
{
	char input;
	switch(assembly_selection)
	{
	case 1:
		printf("\n Calling Bb_Cover. Enter char if thats correct else exit");
		cin>>input;
		Assembly = BB_COVER;
		if (!_robot.SetWorkObject(BB_WObj_x,BB_WObj_y,BB_WObj_z,BB_WObj_q0,BB_WObj_qx,BB_WObj_qy,BB_WObj_qz))
		{
	              ROS_ERROR("Could not set the WorkObj");
		      return false;
	        }
		if (!_robot.SetTool(BB_Tool_x,BB_Tool_y,BB_Tool_z,BB_Tool_q0,BB_Tool_qx,BB_Tool_qy,BB_Tool_qz))
		{
	              ROS_ERROR("Could not set the Tool.");
		      return false;
	        }
	  	if (!_robot.SetComm(1))
		{
	              ROS_ERROR("Could not set the Robot communication mode to blocking.");
		      return false;
	        }
		 if (!_robot.SetZone(ZONE))
	         {
	              ROS_ERROR("Could not set the Robot zone.");
		     return false;
	         }
		 if (!_robot.SetSpeed(FLY_SPEED_TCP, FLY_SPEED_ORI))
	         {
			ROS_ERROR("Could not set the Robot zone.");
		      return false;
	         }
		if (!_robot.SetVacuum(1))
	         {
			ROS_ERROR("Could not set Vacuum to 1");
		      return false;
	         }
		_robot.SetJoints(15,25,5,5,75,5);
		_robot.SetCartesian(BB_Home_x,BB_Home_y,BB_Home_z,BB_Home_q0,BB_Home_qx,BB_Home_qy,BB_Home_qz);
		if (!_robot.SetVacuum(0))
	         {
			ROS_ERROR("Could not set Vacuum to 0");
		      return false;
	         }
		return true;
	case 2:
		printf("\n Calling Shieldcan. Enter char if thats correct else exit");
		cin>>input;
		Assembly = ShieldCan;
		if (!_robot.SetWorkObject(ShieldCan_WObj_x,ShieldCan_WObj_y,ShieldCan_WObj_z,ShieldCan_WObj_q0,ShieldCan_WObj_qx,ShieldCan_WObj_qy,ShieldCan_WObj_qz))
		{
	              ROS_ERROR("Could not set the WorkObj");
		      return false;
	        }
		if (!_robot.SetTool(ShieldCan_Tool_x,ShieldCan_Tool_y,ShieldCan_Tool_z,ShieldCan_Tool_q0,ShieldCan_Tool_qx,ShieldCan_Tool_qy,ShieldCan_Tool_qz))
		{
	              ROS_ERROR("Could not set the Tool");
		      return false;
	        }
		if (!_robot.SetComm(1))
		{
	              ROS_ERROR("Could not set the Robot communication mode to blocking.");
		      return false;
	        }
		 if (!_robot.SetZone(ZONE))
	         {
	              ROS_ERROR("Could not set the Robot zone.");
		      return false;
	         }
		 if (!_robot.SetSpeed(FLY_SPEED_TCP, FLY_SPEED_ORI))
	         {
			ROS_ERROR("Could not set the Robot zone.");
		     return false;
	         }
		_robot.SetJoints(15,25,5,5,75,5);
		_robot.SetCartesian(ShieldCan_Home_x,ShieldCan_Home_y,ShieldCan_Home_z,ShieldCan_Home_q0,ShieldCan_Home_qx,ShieldCan_Home_qy,ShieldCan_Home_qz);
	
		return true;
	default:
		printf("\n Please Enter Correct Assembly No");
		return false;
	}
	
}

