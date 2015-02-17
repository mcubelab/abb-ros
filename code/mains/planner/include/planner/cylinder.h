#ifndef CYLINDER_H
#define CYLINDER_H

class Cylinder: public Block
{
 public:
  string name;
  int obj;
  int num_states;
  map<string,int> states;
  /* enum states {FLAT_PALM, TALL_PALM, FLAT_FINGERTIP, */
  /* 	       TALL_FINGERTIP, FLAT_WORLD, TALL_WORLD}; */
  map<int, double> actions;
  vector< vector< double > > graph;

  Cylinder();

  void set_values();
  void create_graph();
  string getStateName(int state);
  string getActionName(int action);
  
  string getName();
  int getObj();
  int getNumStates();
  map<string, int> getStates();
  map<int, double> getActions();
  vector< vector< double > > getGraph();
  /* typedef enum States { */

  
};

#endif
