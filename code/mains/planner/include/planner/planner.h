#ifndef PLANNER_H
#define PLANNER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <objRec_comm/objRec_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <regraspComm/regrasp_comm.h>
#include <regraspComm/Regrasp.h>
#include <regraspNode/geometry_tools.h>
#include <math.h> 
#include "planner/block.h"
#include "planner/cylinder.h"
#include "planner/triangle.h"

#include <ctime>
#include <unistd.h>

#include <stdio.h>
#include <limits.h>

using namespace std;

namespace ActionType
{
  enum Type
    {
      WORLD_TO_HAND = 0,
      HAND_TO_WORLD,
      WORLD_TO_WORLD,
      HAND_TO_HAND
    };
}

class Planner
{
 public:
  Planner(ros::NodeHandle * n);
  Planner(ros::NodeHandle * n, int obj);
  virtual ~Planner();

  ros::NodeHandle *node;
  /* Cylinder cyl; */
  /* Triangle tri; */
  Block * current_obj;

  // Utilities to detect the block 
  bool setCurrentObj();
  bool setCurrentObj(int obj, Vec startV, Quaternion startQ, 
		     string state, vector<double> verts);
  bool lookForBlock(int &obj, vector<double> &verts, 
		    Vec &blockVec, Quaternion &blockQuat);
  bool findState(int vision_obj, string &state, Vec &startVec);

  // Utilities to search the regrasp graph
  bool dijkstra(int src, int goal, vector<int> &path);
  int minDistance(int num_states, double dist[], bool sptSet[]);
  bool shortestPath(int src, int goal, int previous[], vector<int> &path);
  void printPlan(vector<int> plan);
  
  // Utilities to execute plan
  bool findPlan(int goal, vector<int> &plan, bool pickAndPlace = false);
  bool fillPlanParams(vector<int> plan, Vec goal_vec, Quaternion goal_quat,
		      vector<regraspComm::Regrasp> &rplan);
  bool execute(vector<regraspComm::Regrasp> plan);
  bool executeObj(int goal_state, Vec goal_vec, Quaternion goal_quat);

  bool getDroopGrasp(regraspComm::Droop &msg);

  // for testing 
  bool testObj();
  
 private:
  RobotComm robot;
  HandComm hand;
  MatlabComm matlab;
  RegraspComm regrasp;
  TableVisionComm tableVision;
  
  vector< vector< double > > triangle_graph; 
  vector< vector< double > > cylinder_graph;
  map<int, double> actions;
  

  string current_state;
  Vec start_vec;
  Quaternion start_quat;
  vector<double> vertices;

  void get_graphs(vector< vector< double > > &tri, vector< vector< double > > &cyl,
  		  map<int, double> &actions);
					       
};

#endif 
