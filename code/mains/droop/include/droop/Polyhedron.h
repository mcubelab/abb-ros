#ifndef POLYHEDRON_NODE_H
#define POLYHEDRON_NODE_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
//#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include "droop/Polygon.h"
#include <objRec_comm/objRec_comm.h>

using namespace std;

class Polyhedron
{
 public:
  Polyhedron(vector<Polygon> f);
  Polyhedron(const Polyhedron &orig);
  Polyhedron(int obj);

  void print();
  Vec findCentroid();
  double findMoment(Vec grasp);
  vector<Vec> getVertices();
  void rotate(Quaternion q);
  void rotate(RotMat r);
  void translate(Vec v);
  Polygon operator[](const int i) const;

  // find the face supporting the object given gravity and a grasp
  bool findSupportFace(Vec grasp, int &face_index);  
  bool findGraspRotation(Vec grasp, Quaternion &q); // dont think this works
  bool getFaceGrasp(int face, Vec &grasp);
  bool graspable(Vec grasp, int &grasp_face, Vec &new_grasp);
  bool testgraspable(Vec grasp, int &grasp_face, Vec &new_grasp, bool &give_up);
  bool findApproach(Vec grasp, int grasp_face, Quaternion &q);
  bool findGraspForce(int &close);

  bool pick(int goal_face, Vec &orig_grasp, Vec &usable_grasp, Quaternion &q);
  bool lift(Vec orig_grasp, Vec usable_grasp, Quaternion q, int goal_face, vector<geometry_msgs::Pose> &waypts);
  bool findHandOffset(Quaternion q, Vec &offset);
  bool angleBetweenSides(int s1, int s2, double &angle);
  bool in(Vec pt);
  int quat2face(Quaternion q);
  vector<int> getSymmetricFaces(int face);
  double bottom();
  double top();
  bool alignNormals();

  bool push(Vec grasp, Quaternion q, vector<geometry_msgs::Pose> &waypts);

  bool pushgraspable(Vec grasp, int &grasp_face, Vec &new_grasp, bool &give_up);
  bool findPivotEdge(int goal_face, Vec &pivot);
  bool findPushGrasp(Vec pivot, Vec &grasp);
  bool pushPick(int goal_face, Vec &grasp, Quaternion &q);
  bool pushApproach(Vec grasp, int grasp_face, Quaternion &q);

  bool findFaceAngle(int face, double &angle);
  bool findRotAxis(Vec grasp, Vec &axis);
  bool findRotationAngle(Vec grasp, int goal, double &angle);

  // helpers
  int readModel(int obj, vector<Vec> &verts, vector<Vec> &surfaces);
  geometry_msgs::Pose cart2pose(Vec v, Quaternion q);
  void pose2cart(geometry_msgs::Pose p, Vec &v, Quaternion &q);
  
  // gettrs
  vector<Polygon> getFaces() const {return faces;};
  int size() const {return n;};
  Vec getCentroid() const {return centroid;};
  Polygon getFace(int i) const {return faces[i];};
  Quaternion getRotation() const {return rotation;};
  double getMass() const {return mass;};
  int getObj() const {return obj;};

 private:
  int n;
  Vec centroid;
  vector<Polygon> faces;
  Quaternion rotation;
  double mass;
  int obj;

  // helper funcitons
  bool findFaces(vector<Vec> verts, vector<Vec> tri_ind);
  bool permutation(vector<int> t1, vector<int> t2);
  bool coplanar(Vec x1, Vec x2, Vec x3, Vec x4);
  bool edge(Vec v1, Vec v2, int &pt);
  bool find(vector<Vec> arr, Vec elem, int &ind);

};

#endif
