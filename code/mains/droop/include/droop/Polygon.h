#ifndef POLYGON_NODE_H
#define POLYGON_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
//#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>

using namespace std;

class Polygon
{
 public:
  Polygon(vector<Vec> v);
  Polygon() {};

  void print();
  bool coplanar();
  bool coplanar(Vec pt);
  Vec findNormal();
  bool switchNormal(bool reverse);
  Vec findCentroid();
  Vec operator[](const int i) const;
  bool findPlanarAxes(Vec &x, Vec &y);
  bool convexHull(vector<Vec> v, vector<Vec> &ordered_v);
  vector<Vec> insert(vector<Vec> v, int pos, Vec val);
  double cross(Vec o, Vec p1, Vec p2);
  bool in(Vec pt);
  bool findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i);
  bool findEdgeIntersect(Vec o, Vec slope, Vec &i);
  void rotate(Quaternion q);
  void rotate(RotMat r);
  void translate(Vec v);
  bool ptIntersection(Vec o, Vec slope, Vec &intersection);
  bool ptPlaneDist(Vec v, double &dist);
  Vec projection(Vec v);
  bool onEdge(Vec v, double &d);
  bool operator==(const Polygon &p) const;

  // gettrs
  vector<Vec> getVertices() const {return vertices;};
  int size() const {return n;};
  Vec getNormal() const {return normal;};
  Vec getCentroid() const {return centroid;};

 private:
  vector<Vec> vertices;
  int n;
  Vec normal;
  Vec centroid; 

  //  vector<double> side_lengths;
};


#endif
