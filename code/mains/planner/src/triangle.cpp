#include "planner/block.h"
#include "planner/triangle.h"

Triangle::Triangle() {}

void Triangle::set_values()
{
  name = "triangle";
  obj = RecObj::BIG_TRIANGLE;
  num_states = 11;

  states["TRI_WORLD"] = 0;
  states["LONG_WORLD"] = 1;
  states["SHORT_WORLD"] = 2;
  states["TRI_FINGERTIP"] = 3;
  states["LONG_FINGERTIP"] = 4;
  states["SHORT_FINGERTIP"] = 5;
  states["TRI_PALM"] = 6;
  states["LONG_PALM"] = 7;
  states["SHORT_PALM"] = 8;
  states["LONG_FLIP_PALM"] = 9;
  states["LONG_FLIP_FINGERTIP"] = 10;

  create_graph();

  //define actions
  actions[0] = 0;
  actions[PICK] = 0.9;
  actions[PLACE] = 1.0;
  actions[ROLL_TO_GROUND] = 0.9;
  actions[THROW_TO_PALM] = 0.8;
  actions[THROW_TO_FINGERTIP] = 0.7;
  actions[THROW_AND_FLIP] = 0.7;
  actions[ROLL_TO_PALM] = 0.95;
  actions[LONG_EDGE_DROOP] = 0.95;
  actions[SHORT_EDGE_DROOP] = 0.95;
  
}

string Triangle::getStateName(int state)
{
  string names [11] = {"TRI_WORLD", "LONG_WORLD", "SHORT_WORLD",
		       "TRI_FINGERTIP","LONG_FINGERTIP",
		       "SHORT_FINGERTIP","TRI_PALM","LONG_PALM",
		       "SHORT_PALM","LONG_FLIP_PALM","LONG_FLIP_FINGERTIP"};
  return names[state];

}

string Triangle::getActionName(int action)
{
  string names [10] = {"PICK", "PLACE", "ROLL_TO_GROUND", "THROW_TO_PALM", 
		       "THROW_TO_FINGERTIP", "THROW_TO_PALM", "THROW_AND_FLIP",
		       "ROLL_TO_PALM", "LONG_EDGE_DROOP", "SHORT_EDGE_DROOP"};
  return names[action];
}

void Triangle::create_graph()
{
  vector< vector< double > > tri (num_states, vector< double > (num_states, 0.0) );

  // tri[0][3] = PICK;
  // tri[1][4] = PICK;
  // tri[2][5] = PICK;
  // tri[2][10] = PICK;

  // tri[3][0] = PLACE;
  // tri[4][1] = PLACE;
  // tri[5][2] = PLACE;
  // tri[9][2] = PLACE;
  // tri[10][2] = PLACE;
  // tri[5][1] = PLACE;

  // tri[3][6] = THROW_TO_PALM;
  // tri[3][1] = ROLL_TO_GROUND;
  // tri[3][9] = ROLL_TO_PALM;

  // tri[6][9] = THROW_AND_FLIP;
  // tri[6][3] = THROW_TO_FINGERTIP;

  // tri[9][10] = THROW_TO_FINGERTIP;
  // tri[10][9] = THROW_TO_PALM;

  // tri[8][5] = THROW_TO_FINGERTIP;
  // tri[5][8] = THROW_TO_PALM;

  // tri[2][1] = SHORT_EDGE_DROOP;
  // tri[1][2] = LONG_EDGE_DROOP;
  // //tri[1][0] = TOPPLE;
  // //tri[2][0] = TOPPlE;


  tri[states["TRI_WORLD"]][states["TRI_FINGERTIP"]] = PICK;
  tri[states["LONG_WORLD"]][states["LONG_FINGERTIP"]] = PICK;
  tri[states["SHORT_WORLD"]][states["SHORT_FINGERTIP"]] = PICK;
  tri[states["SHORT_WORLD"]][states["LONG_FLIP_FINGERTIP"]] = PICK;

  tri[states["TRI_FINGERTIP"]][states["TRI_WORLD"]] = PLACE;
  tri[states["LONG_FINGERTIP"]][states["LONG_WORLD"]] = PLACE;
  tri[states["SHORT_FINGERTIP"]][states["SHORT_WORLD"]] = PLACE;
  tri[states["LONG_FLIP_PALM"]][states["SHORT_WORLD"]] = PLACE;
  tri[states["LONG_FLIP_FINGERTIP"]][states["SHORT_WORLD"]] = PLACE;
  tri[states["SHORT_FINGERTIP"]][states["LONG_WORLD"]] = PLACE;

  tri[states["TRI_FINGERTIP"]][states["TRI_PALM"]] = THROW_TO_PALM;
  tri[states["TRI_FINGERTIP"]][states["LONG_WORLD"]] = ROLL_TO_GROUND;
  tri[states["TRI_FINGERTIP"]][states["LONG_FLIP_PALM"]] = ROLL_TO_PALM;

  tri[states["TRI_PALM"]][states["LONG_FLIP_PALM"]] = THROW_AND_FLIP;
  tri[states["TRI_PALM"]][states["TRI_FINGERTIP"]] = THROW_TO_FINGERTIP;

  tri[states["LONG_FLIP_PALM"]][states["LONG_FLIP_FINGERTIP"]] = THROW_TO_FINGERTIP;
  tri[states["LONG_FLIP_FINGERTIP"]][states["LONG_FLIP_PALM"]] = THROW_TO_PALM;

  tri[states["SHORT_PALM"]][states["SHORT_FINGERTIP"]] = THROW_TO_FINGERTIP;
  tri[states["SHORT_FINGERTIP"]][states["SHORT_PALM"]] = THROW_TO_PALM;

  tri[states["SHORT_WORLD"]][states["LONG_WORLD"]] = SHORT_EDGE_DROOP;
  tri[states["LONG_WORLD"]][states["SHORT_WORLD"]] = LONG_EDGE_DROOP;
  //tri[states["LONG_WORLD"]][states["TRI_WORLD"]] = TOPPLE;
  //tri[states["SHORT_WORLD"]][states["TRI_WORLD"]] = TOPPlE;

  graph = tri;
}
string Triangle::getName() {return name;}
int Triangle::getObj() {return obj;}
int Triangle::getNumStates() {return num_states;}
map<string, int> Triangle::getStates() {return states;}
map<int, double> Triangle::getActions() {return actions;}
vector< vector< double > > Triangle::getGraph() {return graph;}
