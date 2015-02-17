#ifndef BLOCK_H
#define BLOCK_H

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <matlab_comm/matlab_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <regraspComm/regrasp_comm.h>
#include <regraspComm/Regrasp.h>
#include <math.h> 
#include <objRec_comm/objRec_comm.h>

using namespace std;

class Block
{
 public:
  string name;
  int obj;
  int num_states;
  map<string, int> states;
  map<int, double> actions;
  vector< vector< double > > graph;

  Block();
  Block(int obj);
  ~Block();

  virtual void set_values();
  virtual void create_graph();
  virtual string getStateName(int state);
  virtual string getActionName(int action);
  virtual string getName() {return name;}
  virtual int getObj() {return obj;}
  virtual int getNumStates() {return num_states;}
  virtual map<string, int> getStates() {return states;}
  virtual map<int, double> getActions() {return actions;}
  virtual vector< vector< double > > getGraph() {return graph;}
  
};

#endif
