#ifndef GEOMETRY_TOOLS_NODE_H
#define GEOMETRY_TOOLS_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
//#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>

using namespace std;

class GeometryTools
{
 public:
  GeometryTools(vector<Vec> v);
  
  void rotate(Quaternion q);
  bool findBoundary();

  double area(void); // overestimates
  bool centroid(Vec &com);
  bool boundingBox(double &x_min, double &y_min, double &x_max, double &y_max);
  bool pointsIn(Vec pt, Vec dir);
  double liftEdge(Vec dir);
  bool findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i);
  bool findIntersect(Vec x1, Vec x2, Vec &i, double &dist);
  bool regular(void);
  double dist(double dX0, double dY0, double dX1, double dY1);
  double max(vector<double> x);
  double min(vector<double> x);
  int argmax(vector<double> x);
  int argmin(vector<double> x);
  vector<double> getSideLengths(void);
  bool findSideQuat(Quaternion q, Quaternion &grasp, double &angle);
  bool findPrincipalDir(Vec &dir, double &dist);

 private:
  vector<Vec> vertices;
  int num_sides;
  vector<double> side_lengths;
};


#endif
