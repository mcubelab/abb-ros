#include "planner/block.h"


Block::Block()
{


}

Block::Block(int obj)
{
  // if (obj == RecObj::BIG_TRIANGLE)
  //   Triangle();
  // if (obj == RecObj::CYLINDER)
  //   Cylinder();
}

Block::~Block() {}

void Block::create_graph() {}

string Block::getStateName(int state) {return "";}

string Block::getActionName(int action) {return "";}

void Block::set_values() {}
