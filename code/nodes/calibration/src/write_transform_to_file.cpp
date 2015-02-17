/* This file simply takes a set of points in two frames and writes the
 * transform to file.
 *
 * Nick Stanley
 * Carnegie Mellon University
 * July 2012
 */

#include <matlab_comm/matlab_comm.h>

// Standard C++ includes
#include <iostream>
#include <string>
#include <fstream>
#include <math.h>
#include <string.h>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <pthread.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "write_transform_to_file");
	ros::NodeHandle node;
	MatlabComm matlab(&node);
	matlab.subscribe(&node);

	ROS_INFO("Waiting for Matlab Node...");
	while (!matlab.Ping()) ;
	ROS_INFO("Getting data.");

	if (!matlab.sendCommand("A = csvread('/home/simplehands/Documents/hands/code/nodes/calibration/data.txt', 0, 0, 'A1..C72')"))
		ROS_ERROR("Couldn't send command 1.");
	if (!matlab.sendCommand("B = csvread('/home/simplehands/Documents/hands/code/nodes/calibration/data.txt', 0, 3, 'D1..F72')"))
		ROS_ERROR("Couldn't send command 2.");
	if (!matlab.sendCommand("A = A(~any(isnan(A),2),:)"))
		ROS_ERROR("Couldn't send command 3.");
	if (!matlab.sendCommand("A = A * 1000")) 
		ROS_ERROR("Couldn't send command 4.");
	if (!matlab.sendCommand("cd ~/Documents/"))
		ROS_ERROR("Couldn't send command 5.");
	if (!matlab.sendCommand("open absor.m"))
		ROS_ERROR("Couldn't send command 6.");
	if (!matlab.sendCommand("[regParams, Bfit, ErrorStats] = absor(A', B')"))
		ROS_ERROR("Couldn't send command 7.");
	if (!matlab.sendCommand("q = regParams.q"))
		ROS_ERROR("Couldn't send command 8.");
	if (!matlab.sendCommand("t = regParams.t"))
		ROS_ERROR("Couldn't send command 9.");
	

	Vec quaternion (4); 
	Vec translation(3); 

	if (!matlab.getVec("q", quaternion))
	{
		ROS_ERROR("Didn't get quaternion.");
	}
	if (!matlab.getVec("t", translation))
	{
		ROS_ERROR("Didn't get translation.");
	}

	ROS_INFO("Matlab analysis complete. Writing data."); 
	// Write to file.
	fstream datafile;
	datafile.open ("/home/simplehands/Documents/hands/code/parameters/transform_params.yaml");
	datafile << "qw: " << quaternion[0] << endl;
	datafile << "qx: " << quaternion[1] << endl;
	datafile << "qy: " << quaternion[2] << endl;
	datafile << "qz: " << quaternion[3] << endl;
	datafile << "tx: " << translation[0] / 1000.0 << endl;
	datafile << "ty: " << translation[1] / 1000.0 << endl;
	datafile << "tz: " << translation[2] / 1000.0 << endl;
	ROS_INFO("Result is q = <%f, %f, %f, %f>, \n t = <%f, %f, %f>.",
		quaternion[0], quaternion[1], quaternion[2], quaternion[3],
		translation[0], translation[1], translation[2]);

	datafile << "# This simply holds the latest params. Old ones are pushed to";
	datafile << endl;
	datafile << "#old_transform_params/<time>_params.yaml .";

	datafile.close();

	ROS_INFO("Data written.");
	
}
