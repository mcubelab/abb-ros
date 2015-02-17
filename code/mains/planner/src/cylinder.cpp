#include "planner/block.h"
#include "planner/cylinder.h"

Cylinder::Cylinder() {}

void Cylinder::set_values()
{
  name = "cylinder";
  obj = RecObj::CYLINDER;
  num_states = 6;
  states["FLAT_PALM"] = 0;
  states["TALL_PALM"] = 1;
  states["FLAT_FINGERTIP"] = 2;
  states["TALL_FINGERTIP"] = 3;
  states["FLAT_WORLD"] = 4;
  states["TALL_WORLD"] = 5;

  create_graph();

  //define actions
  actions[0] = 0;
  actions[PICK] = 0.9;
  actions[PLACE] = 1.0;
  actions[ROLL_TO_GROUND] = 0.9;
  actions[ROLL_TO_PALM] = 0.95;
  actions[DROOP_IN_FINGERS] = 0.9;
}

string Cylinder::getStateName(int state)
{
  string names [6] = {"FLAT_PALM","TALL_PALM","FLAT_FINGERTIP",
		       "TALL_FINGERTIP","FLAT_WORLD","TALL_WORLD"};
  return names[state];

}

string Cylinder::getActionName(int action)
{
  string names [5] = {"PICK", "PLACE", "ROLL_TO_PALM", "ROLL_TO_FINGERTIP", 
		       "DROOP_IN_FINGERS"};
  return names[action];
}

void Cylinder::create_graph()
{

  vector< vector< double > > cyl (num_states, vector< double > (num_states, 0.0) );
  // cyl[0][2] = ROLL_TO_FINGERTIP;
  // cyl[2][4] = PLACE;
  // cyl[2][0] = ROLL_TO_PALM;
  // cyl[3][5] = PLACE;
  // cyl[4][2] = PICK;
  // cyl[4][3] = DROOP_IN_FINGERS;
  // cyl[5][3] = PICK;
  
  cyl[states["FLAT_PALM"]][states["FLAT_FINGERTIP"]] = ROLL_TO_FINGERTIP;
  cyl[states["FLAT_FINGERTIP"]][states["FLAT_WORLD"]] = PLACE;
  cyl[states["FLAT_FINGERTIP"]][states["FLAT_PALM"]] = ROLL_TO_PALM;
  cyl[states["TALL_FINGERTIP"]][states["TALL_WORLD"]] = PLACE;
  cyl[states["FLAT_WORLD"]][states["FLAT_FINGERTIP"]] = PICK;
  cyl[states["FLAT_WORLD"]][states["TALL_FINGERTIP"]] = DROOP_IN_FINGERS;
  cyl[states["TALL_WORLD"]][states["TALL_FINGERTIP"]] = PICK;
  
  graph = cyl;

}

string Cylinder::getName() 
{
  return name;
}

int Cylinder::getObj() {return obj;}
int Cylinder::getNumStates() {return num_states;}
map<string, int> Cylinder::getStates() {return states;}
map<int, double> Cylinder::getActions() {return actions;}
vector< vector< double > > Cylinder::getGraph() {return graph;}
