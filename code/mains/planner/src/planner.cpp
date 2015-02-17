#include "planner/planner.h"

Planner::Planner(ros::NodeHandle *n)
{
  node = n;
  RobotComm r(node);
  HandComm h(node);
  RegraspComm rg(node);
  TableVisionComm tv(node);

  MatlabComm m;
  m.subscribe(node);
  std::string pkg_path = "~/Documents/hands/code/nodes/vision/ROS/tableVision_node";
  pkg_path += "/matlab_scripts";
  m.addPath(pkg_path.c_str());

  matlab = m;
  robot = r;
  hand = h;
  regrasp = rg;
  tableVision = tv;
}

Planner::Planner(ros::NodeHandle *n, int obj)
{
  node = n;
  RobotComm r(node);
  HandComm h(node);
  RegraspComm rg(node);
  TableVisionComm tv(node);

  MatlabComm m;
  m.subscribe(node);
  std::string pkg_path = "~/Documents/hands/code/nodes/vision/ROS/tableVision_node";
  pkg_path += "/matlab_scripts";
  m.addPath(pkg_path.c_str());

  matlab = m;
  robot = r;
  hand = h;
  regrasp = rg;
  tableVision = tv;

  if (obj == RecObj::BIG_TRIANGLE)
    {
      current_obj = new Triangle();
      current_obj->set_values();
    }
  else if (obj == RecObj::CYLINDER)
    {
      current_obj = new Cylinder();
      current_obj->set_values();
    }

}

Planner::~Planner()
{
  delete current_obj;
}

/***************************************************************/
/******************** Detecting the block **********************/

bool Planner::setCurrentObj(int obj, Vec startV, Quaternion startQ, 
			    string state, vector<double> verts)
{

  if (obj == RecObj::BIG_TRIANGLE)
    {
      current_obj = new Triangle();
      current_obj->set_values();
    }
  else if (obj == RecObj::CYLINDER)
    {
      current_obj = new Cylinder();
      current_obj->set_values();
    }

  vertices = verts;
  current_state = state;
  start_vec = startV;
  start_quat = startQ;
  return true;
}

bool Planner::setCurrentObj()
{
  int vision_obj;
  bool suc = lookForBlock(vision_obj, vertices, start_vec, start_quat);
  cout << "suc? " << suc << endl;
  if (suc)
    {
      findState(vision_obj, current_state, start_vec);
      return true;
    }
  else
    return false;

}

bool Planner::lookForBlock(int &obj, vector<double> &verts, 
			   Vec &blockVec, Quaternion &blockQuat)
{
  ROS_INFO("Looking for a block on the table");
  // robot.SetJoints(20,0,0,0,90,0);
  // usleep(10000000);
 
  // try vision 5 times
  int count = 0;
  bool success = false;

  do 
    {
      success = tableVision.GetObjAndPose(obj, verts, blockVec, blockQuat);
      count++;
    }
  while ((!success) && (count<5));

  // cout << "verts size: " << verts.size() << endl;
  // for (int i=0; i<verts.size();i++)
  //   {
  //     cout << verts[i] << endl;
  //   }
  //cout << "verts: " << verts << endl;
  
  return success;

}

bool Planner::findState(int vision_obj, string &state, Vec &startVec)
{

  if (vision_obj==1) // flat triangle
    {
      ROS_INFO("flat tri");
      current_obj = new Triangle();
      state = "TRI_WORLD";
      startVec[2] = 85;
    }
  else if (vision_obj==2) 
    {
      ROS_INFO("short tri ");
      current_obj = new Triangle();
      state = "SHORT_WORLD";
      startVec[2] = 110;
    }
  else if (vision_obj==3)
    {
      ROS_INFO("tall cyl");
      current_obj = new Cylinder();
      state = "TALL_WORLD";
      startVec[2] = 175;
    }
  else if (vision_obj==4)
    {
      ROS_INFO("long tri");
      current_obj = new Triangle();
      state = "LONG_WORLD";
      startVec[2] = 90;
    }
  else if (vision_obj==5)
    {
      ROS_INFO("long cyl");
      current_obj = new Cylinder();
      state = "FLAT_WORLD";
      startVec[2] = 80;
    }
  else 
    return false;

  current_obj->set_values();
  return true;
}


/***************************************************************/
/******************** Dijkstra *********************************/

bool Planner::dijkstra(int src, int goal, vector<int> &path)
{
  int num_states = current_obj->getNumStates();
  map<int, double> actions = current_obj->getActions();
  vector< vector< double > > graph = current_obj->getGraph();

  bool seen[num_states]; 
  int previous[num_states];
  double dist[num_states]; 

  for (int i = 0; i < num_states; i++)
    {
      dist[i] = INFINITY;
      seen[i] = false;
      previous[i] = -1; 
    }
  
  dist[src] = 0;
 
  for (int count = 0; count < num_states-1; count++)
    {
      int u = minDistance(num_states, dist, seen);
      if (u==-1)
	break;
      seen[u] = true;
      //cout << "u " << TriangleStateNames[u] << endl;
      
      for (int v = 0; v < num_states; v++)
	{
	  double alt = dist[u] + 
	    actions[graph[u][v]];
	  
	  if ((alt < dist[v]) && (graph[u][v]!=0))
	    {
	      //cout << "v " << TriangleStateNames[v] << " " << alt << endl;
	      dist[v] = alt;
	      previous[v] = u;
	    }
	}
    }
  
  //ROS_INFO("filled in dists");
  //printSolution(dist, previous, num_states);
  //cout << "dgoal " << TriangleStateNames[goal] << endl;
  //cout << "dsrc " << TriangleStateNames[src] << endl;
  bool ret = shortestPath(src, goal, previous, path);
  return ret;
}

int Planner::minDistance(int num_states, double dist[], bool seen[])
{
  double min = INFINITY;
  int min_index = -1;
 
  for (int v = 0; v < num_states; v++)
    {
      if (seen[v] == false && dist[v] <= min)
	{
	  min = dist[v], min_index = v;
	  
	}
    }
  return min_index;
}

bool Planner::shortestPath(int src, int goal, int previous[], vector<int> &path)
{
  
  // if (src==goal)
  //   return false;

  vector< vector< double > > graph = current_obj->getGraph();
  vector< int > state_path;
  int u = goal;
  int max_len = 5;
  int count = 0;
  while (previous[u] != src) 
    {
      if ((count >= max_len) || (previous[u]==-1))
	return false;
      state_path.push_back(previous[u]);
      path.push_back(graph[previous[u]][u]);
      u = previous[u];
      count++;
    }
  path.push_back(graph[src][u]);
  ROS_INFO("found a shortest path");
  
  return true;
  // if (path.size() != 0)
  //   return true;
  // else
  //   return false;
}

void Planner::printPlan(vector<int> plan)
{
  map<int, string> act;
  act[PICK] = "pick";
  act[PLACE] = "place";
  act[MOVE] = "move";
  act[DROOP_IN_FINGERS] = "droop in fingers";
  act[PUSH_IN_ENVELOPING] = "push in enveloping";
  act[PUSH_IN_FINGERS] = "push in fingers";
  act[ROLL_ON_GROUND] = "roll on ground";
  act[ROLL_TO_FINGERTIP] = "roll to fingertip";
  act[ROLL_TO_GROUND] = "roll to ground";
  act[ROLL_TO_PALM] = "roll to palm";
  act[SQUEEZE] = "squeeze";
  act[VIBRATE] = "vibrate";
  act[THROW_AND_FLIP] = "throw and flip";
  act[THROW_TO_PALM] = "throw to palm";
  act[THROW_TO_FINGERTIP] = "throw to fingertip";
  act[LONG_EDGE_DROOP] = "long edge droop";
  act[SHORT_EDGE_DROOP] = "short edge droop";

  for (int i=0; (unsigned)i<plan.size(); i++)
    cout << "Step i: " << act[plan[i]] << endl;
  
}

/***************************************************************/


/***************************************************************/
/******************** EXECUTING THE PLAN ***********************/

bool Planner::findPlan(int goal, vector<int> &plan, bool pickAndPlace)
{
  map<string, int> states = current_obj->getStates();
  cout << "pickAndPlace: " << pickAndPlace << endl;
  cout << "start: " << current_state << " goal: " << goal << endl;
  if ((goal == states[current_state]) || pickAndPlace)
    {
      plan.push_back(PLACE);
      plan.push_back(MOVE);
      plan.push_back(PICK);
    }    
  else
    {
      bool suc = dijkstra(states[current_state], goal, plan);
      if (!suc)
	return false;
    }

  return true;
}
 
bool Planner::fillPlanParams(vector<int> plan, Vec goal_vec, Quaternion goal_quat,
		      vector<regraspComm::Regrasp> &rplan)
{
  int l = plan.size();
      
  // reverse the order and get defaults
  //vector<regraspComm::Regrasp> rplan;
  for (int i=(l-1); i>=0; i--)
    rplan.push_back(regrasp.getDefaults(plan[i]));


  map<int, double> actionsType;
  actionsType[0] = 0;
  actionsType[PICK] = ActionType::WORLD_TO_HAND;
  actionsType[PLACE] = ActionType::HAND_TO_WORLD;
  actionsType[ROLL_TO_GROUND] = ActionType::HAND_TO_WORLD;
  actionsType[THROW_TO_PALM] = ActionType::HAND_TO_HAND;
  actionsType[THROW_TO_FINGERTIP] = ActionType::HAND_TO_HAND;
  actionsType[THROW_AND_FLIP] = ActionType::HAND_TO_HAND;
  actionsType[ROLL_TO_PALM] = ActionType::HAND_TO_HAND;
  actionsType[DROOP_IN_FINGERS] = ActionType::WORLD_TO_HAND;
  actionsType[LONG_EDGE_DROOP] = ActionType::WORLD_TO_WORLD;
  actionsType[SHORT_EDGE_DROOP] = ActionType::WORLD_TO_WORLD;

  for (int i=0;i<l;i++)
    {
      if (rplan[i].action == MOVE)
	{
	  vector<double> joints;
	  joints.push_back(0.0);
	  joints.push_back(0.0);
	  joints.push_back(0.0);
	  joints.push_back(0.0);
	  joints.push_back(90.0);
	  joints.push_back(0.0);
	  geometry_msgs::Pose p;
	  p.position.x = 600;
	  p.position.y = 200;
	  p.position.z = 400;
	  p.orientation.w = 0.0;
	  p.orientation.x = 0.707;
	  p.orientation.y = 0.707;
	  p.orientation.z = 0.0;
	  regraspComm::Move m;
	  m.cart = true;
	  m.pose = p;
	  m.joints = joints;
	  rplan[i].move = m;
	}
      if (actionsType[rplan[i].action]==ActionType::WORLD_TO_WORLD)
        {
          geometry_msgs::Pose obj;
          obj.position.x = start_vec[0];
          obj.position.y = start_vec[1];
          obj.position.z = start_vec[2];
          obj.orientation.w = start_quat[0];
          obj.orientation.x = start_quat[1];
          obj.orientation.y = start_quat[2];
          obj.orientation.z = start_quat[3];
	  
          if (rplan[i].action==LONG_EDGE_DROOP)
            rplan[i].longEdgeDroop.pick_pose = obj;
          if (rplan[i].action==SHORT_EDGE_DROOP)
            rplan[i].shortEdgeDroop.pick_pose = obj;
          
        } 
      else if (actionsType[rplan[i].action]==ActionType::HAND_TO_WORLD)
        {
          geometry_msgs::Pose obj;
          obj.position.x = goal_vec[0];
          obj.position.y = goal_vec[1];
          obj.position.z = goal_vec[2];
          obj.orientation.w = goal_quat[0];
          obj.orientation.x = goal_quat[1];
          obj.orientation.y = goal_quat[2];
          obj.orientation.z = goal_quat[3];
          
          if (rplan[i].action==PLACE)
            rplan[i].place.pose = obj;
          if (rplan[i].action==ROLL_TO_GROUND)
            rplan[i].rollToGround.flip_pose = obj; 
          
        }
      else if (actionsType[rplan[i].action]==ActionType::WORLD_TO_HAND)
        {
          geometry_msgs::Pose obj;
          obj.position.x = start_vec[0];
          obj.position.y = start_vec[1];
          obj.position.z = start_vec[2];
          obj.orientation.w = start_quat[0];
          obj.orientation.x = start_quat[1];
          obj.orientation.y = start_quat[2];
          obj.orientation.z = start_quat[3];
          
          if (rplan[i].action==PICK)
            rplan[i].pick.pose = obj;
          if (rplan[i].action==DROOP_IN_FINGERS)
	    {
	      regraspComm::Droop tmp = rplan[i].droopInFingers; 
	      tmp.object_vertices = vertices;
	      getDroopGrasp(tmp);
	      rplan[i].droopInFingers = tmp;
	    }
          
        }	  
    }

  return true;
}

bool Planner::getDroopGrasp(regraspComm::Droop &msg)
{
  double threshold = 10; // 25 mm for the fingers

  vector<double> tmp = msg.object_vertices;

  vector<double> replacement;

  vector<Vec> verts;// = msg.object_vertices; 
  for (unsigned int i=0;i<tmp.size();i+=2)
    {
      Vec v(3);
      v[0] = tmp[i];
      v[1] = tmp[i+1];
      v[2] = 0;
      verts.push_back(v);

      replacement.push_back(tmp[i]);
      replacement.push_back(tmp[i+1]);
      replacement.push_back(0);

      cout << "vert: " << v << endl;
    }
  GeometryTools gt(verts);
  Vec com (3);
  gt.centroid(com);
  cout << "com: " << com << endl;
  Vec pc;
  double dist;
  gt.findPrincipalDir(pc, dist);
  cout << "dist " << dist << endl;
  pc.normalize();
  cout << "pc " << pc << endl;
  Vec grasp = com + pc*(dist-threshold);
  grasp[2] = start_vec[2];
  
  geometry_msgs::Pose pose;
  msg.grasp_pose.position.x = grasp[0];
  msg.grasp_pose.position.y = grasp[1];
  msg.grasp_pose.position.z = start_vec[2];//grasp[2];

  Vec rotation = Vec("0 0 0.524",3); // 60 deg in z axis
  matlab.sendVec("q", start_quat);
  matlab.sendVec("r", rotation);
  matlab.sendCommand("quat = rotateGrasp(q,r);");
  Quaternion quat = matlab.getQuaternion("quat");
  msg.grasp_pose.orientation.w = quat[0];
  msg.grasp_pose.orientation.x = quat[1];
  msg.grasp_pose.orientation.y = quat[2];
  msg.grasp_pose.orientation.z = quat[3];
  
  cout << "grasp vec: " << grasp << endl; 
  cout << "quat: " << quat << endl;
  return true;
}


bool Planner::execute(vector<regraspComm::Regrasp> plan)
{
  return regrasp.Execute(plan);
}

/***************************************************************/
/************************FOR TESTING****************************/

bool Planner::testObj()
{
  int obj1 = RecObj::CYLINDER;
  Vec startV1 = Vec("400 200 150",3);
  Quaternion startQ1 = Quaternion("0.0 0.707 0.707 0");
  string state1 = "FLAT_WORLD";

  //double v[12] = { 380,195,0,420,195,0,420,205,0,380,205,0 };
  //double v[12] = { 395,180,0,405,180,0,405,220,0,395,220,0 };
  //double v[12] = { 405,220,0,395,220,0,395,180,0,405,180,0 };
  double v[12] = { 380,195,0,408.3,223.3,0,401.2,230.4,0,372.9,202.1,0 };
  std::vector<double> verts(&v[0], &v[0]+12);

  // Vec v1 = Vec("380 195 0",3);
  // Vec v2 = Vec("420 195 0",3);
  // Vec v3 = Vec("420 205 0",3);
  // Vec v4 = Vec("380 205 0",3);
  // verts.push_back(v1);
  // verts.push_back(v2);
  // verts.push_back(v3);
  // verts.push_back(v4);
  setCurrentObj(obj1, startV1, startQ1, state1, verts);

  int goal1 = current_obj->getStates()["TALL_WORLD"];
  cout << "current " << current_obj->getName() << endl;
  cout << "goal " << goal1 << endl;
  Vec goal_vec1 = Vec("750 200 175",3);
  Quaternion goal_quat1 = Quaternion("0.0 0.707 0.707 0");
  vector<int> tmp_plan1;
  vector<regraspComm::Regrasp> plan1;
  findPlan(goal1, tmp_plan1, false);
  printPlan(tmp_plan1);
  fillPlanParams(tmp_plan1, goal_vec1, goal_quat1, plan1);

  return true;

}

bool Planner::executeObj(int goal_state, Vec goal_vec, Quaternion goal_quat)
{
  bool suc = setCurrentObj();
  if (suc)
    {
      vector<int> tmp_plan;
      vector<regraspComm::Regrasp> plan;
      findPlan(goal_state, tmp_plan, true);
      printPlan(tmp_plan);
      fillPlanParams(tmp_plan, goal_vec, goal_quat, plan);
      execute(plan);

      return true; 
    }
  else
    return false;
}

/***************************************************************/

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Planner");
  ros::NodeHandle node;
  Cylinder cyl;
  cyl.set_values();
  Triangle tri;
  tri.set_values();
  map<string, int> cyl_states = cyl.getStates();
  map<string, int> tri_states = tri.getStates();

  Planner planner(&node);
  planner.testObj();


/***************************************************************/
/************************ ARCH DEMO ****************************/

  // int goal1 = cyl_states["TALL_WORLD"];
  // cout << "goal " << goal1 << endl;
  // Vec goal_vec1 = Vec("750 200 175",3);
  // Quaternion goal_quat1 = Quaternion("0.0 0.707 0.707 0");
  // planner.executeObj(goal1,goal_vec1,goal_quat1);

  // int goal2 = cyl_states["TALL_WORLD"];
  // Vec goal_vec2 = Vec("650 200 175",3);
  // Quaternion goal_quat2 = Quaternion("0.0 0.707 0.707 0");
  // planner.executeObj(goal2,goal_vec2,goal_quat2);

  // int goal3 = tri_states["LONG_WORLD"];
  // Vec goal_vec3 = Vec("700 200 247",3);
  // Quaternion goal_quat3 = Quaternion("0.0 1.0 0.0 0");
  // planner.executeObj(goal3,goal_vec3,goal_quat3);
  
}
