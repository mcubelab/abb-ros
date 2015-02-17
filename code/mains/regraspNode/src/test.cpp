#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include "regraspNode/geometry_tools.h"

using namespace std;

int readModel(vector<Vec> &verts)
{
  string intro = "/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/objectFolder/vert_files";
  string tri = intro + "/big_triangle.txt";
  string rect = intro + "/rectangle.txt";
  string cyl = intro + "/cylinder.txt";

  string delimiter = ",";

  string line;
  ifstream myfile (tri.c_str());
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
      {
	line = line.substr(0, line.size()-1);
	//cout << line << '\n';
	// parse the line
	size_t pos = 0;
	string token;
	int i = 0;
	Vec v = Vec(3);
	while ((pos = line.find(delimiter)) != string::npos) 
	  {
	    token = line.substr(0, pos);
	    //cout << token << endl;
	    double tmp = ::atof(token.c_str());
	    v[i] = tmp;
	    i++;
	    line.erase(0, pos + delimiter.length());
	  }
	//cout << line << endl;
	double tmp = ::atof(line.c_str());
	v[i] = tmp;
	verts.push_back(v);
      }
    myfile.close();
  }

  else cout << "Unable to open file"; 

  for (size_t j=0;j<verts.size();j++)
    cout << verts[j] << endl;  

  return 0;
}

int main () 
{

  vector<Vec> verts;
  readModel(verts);

  GeometryTools gt(verts);
  



}
