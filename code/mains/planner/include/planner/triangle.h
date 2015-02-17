#ifndef TRIANGLE_H
#define TRIANGLE_H

class Triangle: public Block
{
 public:
  string name;
  int obj;
  int num_states;
  map<string, int> states;
  /* {TRI_WORLD, LONG_WORLD, SHORT_WORLD, TRI_FINGERTIP,  */
  /*     LONG_FINGERTIP, SHORT_FINGERTIP, TRI_PALM,  */
  /*     LONG_PALM, SHORT_PALM, LONG_FLIP_PALM, LONG_FLIP_FINGERTIP}; */
  map<int, double> actions;
  vector< vector< double > > graph;

  Triangle();

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
