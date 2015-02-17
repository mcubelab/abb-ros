#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <matVec/matVec.h>
#include <util_comm/util_comm.h>
#include <objRec_comm/objRec_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <regraspComm/regrasp_comm.h>
//#include <planner/Plan.h>
#include <regraspComm/Regrasp.h>
#include <math.h> 

#include <ctime>
#include <unistd.h>

#include <stdio.h>
#include <limits.h>

using namespace std;

#define NUM_TRI_STATES 11
#define NUM_CYL_STATES 8
#define NUM_STATES 11

namespace TriangleStates
{ 
  enum Type 
    {
      TRI_WORLD=0,
      LONG_WORLD,
      SHORT_WORLD,
      TRI_FINGERTIP,
      LONG_FINGERTIP,
      SHORT_FINGERTIP,
      TRI_PALM,
      LONG_PALM,
      SHORT_PALM,
      LONG_FLIP_PALM,
      LONG_FLIP_FINGERTIP
    };
}

string TriangleStateNames[NUM_TRI_STATES] = {"TRI_WORLD", "LONG_WORLD", "SHORT_WORLD",
					     "TRI_FINGERTIP","LONG_FINGERTIP",
					     "SHORT_FINGERTIP","TRI_PALM","LONG_PALM",
					     "SHORT_PALM","LONG_FLIP_PALM","LONG_FLIP_FINGERTIP"};

namespace CylinderStates 
{
  enum Type
    { 
      FLAT_PALM = 0,
      TALL_PALM,
      FLAT_FINGERTIP,
      TALL_FINGERTIP,
      FLAT_WORLD,
      TALL_WORLD
    };
}

string CylinderStateNames[NUM_CYL_STATES] = {"FLAT_PALM","TALL_PALM","FLAT_FINGERTIP",
					     "TALL_FINGERTIP","FLAT_WORLD","TALL_WORLD"};

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

// bool updatePlan(int obj, int start, int goal, 
// 		vector<int> &plan);
bool findBlockOnTable(TableVisionComm tableVision, int nobj, int &obj,
		      vector<double> &verts, Vec &blockVec, Quaternion &blockQuat);
bool findState(int vision_obj, int &obj, int &state);
void triangle_graph(vector< vector< double > > &tri, vector< vector< double > > &cyl, 
		    map<int, double> &actions);
bool dijkstra(int num_states, vector< vector< double > > graph, int src, int goal, 
	      map<int, double> actions, vector<int> &path);
//bool dijkstra(int num_states, int graph[9][9], int src, int dist[]);
void printSolution(double dist[], int previous[], int n);
int minDistance(int num_states, double dist[], bool sptSet[]);
bool shortestPath(int src, int goal, int previous[], map<int, double> actions, 
		  vector< vector< double > > graph, vector<int> &path);
void printPlan(vector<regraspComm::Regrasp> plan);
void printPlan(vector<int> plan);

bool findPlan(Vec startV, int start_obj, int start_state, int goal_state, 
              int goal_obj, Vec goal_vec, vector<int> &plan);
bool fillPlanParams(RegraspComm regrasp, vector<int> plan, Vec start_vec, 
                    Quaternion start_quat, Vec goal_vec, Quaternion goal_quat,
                    vector<regraspComm::Regrasp> &rplan);

bool findPlan(int obj, int start, int goal, vector<int> &plan, bool pickAndPlace = false);


int main(int argc, char** argv)
{
  ros::init(argc, argv, "Executioner");
  ros::NodeHandle node;

  RobotComm robot(&node);
  HandComm hand(&node);
  RegraspComm regrasp(&node);
  TableVisionComm tableVision(&node);

  MatlabComm matlab;
  matlab.subscribe(&node);
  std::string pkg_path = ros::package::getPath("regrasp_demo");
  pkg_path += "/matlab_scripts";
  matlab.addPath(pkg_path.c_str());

  /****************************************************/
  // geometry_msgs::Pose pose;
  // pose.position.x = 685;
  // pose.position.y = 200;
  // pose.position.z = 60;
  // pose.orientation.w = 0.0;
  // pose.orientation.x = 0.707;
  // pose.orientation.y = 0.707;
  // pose.orientation.z = 0.0;
  // vector<regraspComm::Regrasp> tmp;
  // regraspComm::Droop d;
  // d.grasp_pose = pose;
  // static const double verts_tmp[] = {530.0,180.0,0.0, 670.0,180.0,0.0, 
  //       			     670.0,220.0,0.0, 530.0,220.0,0.0};
  // vector<double> verts (verts_tmp, verts_tmp+sizeof(verts_tmp) / sizeof(verts_tmp[0]));
  // d.object_vertices = verts;
  // regraspComm::Regrasp r;
  // r.action = DROOP;
  // r.droop = d;
  // tmp.push_back(r);
  // regrasp.Execute(tmp);
  /*********************************************************/

  // DEFINE GOAL:: TEMP ARCH
  int num_blocks = 3;
  int goal_states[num_blocks];
  int goal_objs[num_blocks];
  Vec goal_vecs[num_blocks];
  Quaternion goal_quats[num_blocks];

  goal_states[0] = CylinderStates::TALL_WORLD;
  goal_states[1] = CylinderStates::TALL_WORLD;
  goal_states[2] = TriangleStates::LONG_WORLD;
  goal_objs[0] = RecObj::CYLINDER;
  goal_objs[1] = RecObj::CYLINDER;
  goal_objs[2] = RecObj::BIG_TRIANGLE;
  goal_vecs[0] = Vec("650 200 200", 3);
  goal_vecs[1] = Vec("550 200 200", 3);
  goal_vecs[2] = Vec("600 200 300", 3);
  goal_quats[0] = Quaternion("0.0 0.707 0.707 0.0");
  goal_quats[1] = Quaternion("0.0 0.707 0.707 0.0");
  goal_quats[2] = Quaternion("0.0 0.707 0.707 0.0");

  for (int i=0;i<num_blocks;i++)
    {
      bool success = false;
// bool findBlockOnTable(TableVisionComm tableVision, int nobj, int &obj,
// 		      vector<double> &verts, Vec &blockVec, Quaternion &blockQuat);
      // FIND START
      Vec startV(3);
      Quaternion startQ;
      int obj, vision_obj, state;
      robot.SetJoints(25,0,0,0,90,0);
      vector<double> verts;
      success = findBlockOnTable(tableVision, goal_objs[i], vision_obj, verts, startV, startQ);
      // double start[7] = {startV[0], startV[1], startV[2], 
      //                    startQ[0], startQ[1], startQ[2], startQ[3]};
      findState(vision_obj, obj, state);


      vector<int> plan;
      vector<regraspComm::Regrasp> rplan;
      bool ret = findPlan(startV, obj, state, goal_states[i], 
                          goal_objs[i], goal_vecs[i], plan);
      fillPlanParams(regrasp, plan, startV, startQ, 
                     goal_vecs[i], goal_quats[i], rplan);
      if (success && ret) 
        {
          bool regrasp_success = regrasp.Execute(rplan);
          cout << "regrasp success: " << regrasp_success << endl;
        }


    }

  return 1;
}

bool findPlan(int obj, int start, int goal, vector<int> &plan, bool pickAndPlace)
{
  if (pickAndPlace)
    {
      plan.push_back(PICK);
      plan.push_back(PLACE);
      return true;
    }




}

bool findPlan(Vec startV, int start_obj, int start_state, int goal_state, 
              int goal_obj, Vec goal_vec, vector<int> &plan)
{  
  // FIND PLAN
  if (goal_obj != start_obj)
    {
      ROS_INFO("WRONG BLOCK");
      return false;
    }
  else
    {  
      // Create the graphs for Dijkstra
      vector< vector< double > > tri;
      vector< vector< double > > cyl;
      map<int, double> actions;
      triangle_graph(tri, cyl, actions);
      bool suc = false;
      
      if (start_obj==RecObj::BIG_TRIANGLE)
        suc = dijkstra(NUM_TRI_STATES, tri, start_state, goal_state, actions, plan);
      else if (start_obj==RecObj::CYLINDER)
        suc = dijkstra(NUM_CYL_STATES, cyl, start_state, goal_state, actions, plan);      

      if (!suc)
        return false;

      // special case of start_state=goal_state: still must pick and place
      if (plan.size()==0)
  	{
          if ((startV[0]==goal_vec[0]) && 
              (startV[1]==goal_vec[1]) && 
              (startV[2]==goal_vec[2]))
            return true;
          else
            {
              plan.push_back(PLACE);
              plan.push_back(PICK);
            }
  	}

      return true;
    }

}

bool fillPlanParams(RegraspComm regrasp, vector<int> plan, Vec start_vec, Quaternion start_quat, Vec goal_vec, Quaternion goal_quat,vector<regraspComm::Regrasp> &rplan)
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
            rplan[i].droopInFingers.grasp_pose = obj;
          
        }	  
    }
  
  
  printPlan(rplan);
  // bool regrasp_success = regrasp.Execute(rplan);
  // ROS_INFO("regrasp plan success: %d", regrasp_success);
  //vector<int> regrasps;
  // //regrasps.push
  //regrasps.push_back(LONG_EDGE_DROOP);
  //cout << "do it " << LONG_EDGE_DROOP << endl;
  //regrasp.Execute(regrasps);

  return true;
}



bool findState(int vision_obj, int &obj, int &state)
{

  if (vision_obj==1) // flat triangle
    {
      ROS_INFO("flat tri");
      obj = RecObj::BIG_TRIANGLE;
      state = TriangleStates::TRI_WORLD;
    }
  else if (vision_obj==2) 
    {
      ROS_INFO("short tri ");
      obj = RecObj::BIG_TRIANGLE;
      state = TriangleStates::SHORT_WORLD;
    }
  else if (vision_obj==3)
    {
      ROS_INFO("tall cyl");
      obj = RecObj::CYLINDER;
      state = CylinderStates::TALL_WORLD;
    }
  else if (vision_obj==4)
    {
      ROS_INFO("long tri");
      obj = RecObj::BIG_TRIANGLE;
      state = TriangleStates::LONG_WORLD;
    }
  else if (vision_obj==5)
    {
      ROS_INFO("long cyl");
      obj = RecObj::CYLINDER;
      state = CylinderStates::FLAT_WORLD;
    }
  else 
    return false;

  return true;
}

bool findBlockOnTable(TableVisionComm tableVision, int nobj, int &obj, vector<double> &verts, 
		      Vec &blockVec, Quaternion &blockQuat)
{
  cout << "Looking for block on the table" << endl;

  int count = 0;
  bool success = false;

  do 
    {
      success = tableVision.GetObjAndPose(obj, verts, blockVec, blockQuat);
      count++;
    }
  while ((!success) && (count<5));
  
  return success;
}

// bool updatePlan(int obj, int start, int goal, vector<int> &plan)
// {
//   return ret;
  
// }
	      
void triangle_graph(vector< vector< double > > &tri_graph, vector< vector< double > > &cyl_graph, map<int, double> &actions)
{
  actions[0] = 0;
  actions[PICK] = 0.9;
  actions[PLACE] = 1.0;
  actions[ROLL_TO_GROUND] = 0.9;
  actions[THROW_TO_PALM] = 0.8;
  actions[THROW_TO_FINGERTIP] = 0.7;
  actions[THROW_AND_FLIP] = 0.7;
  actions[ROLL_TO_PALM] = 0.95;
  actions[DROOP_IN_FINGERS] = 0.9;
  actions[LONG_EDGE_DROOP] = 0.95;
  actions[SHORT_EDGE_DROOP] = 0.95;
  //actions[TOPPLE] = 0.95;

  vector< vector< double > > tri (NUM_TRI_STATES, vector< double > (NUM_TRI_STATES, 0.0) );
  tri[TriangleStates::TRI_WORLD][TriangleStates::TRI_FINGERTIP] = PICK;
  tri[TriangleStates::LONG_WORLD][TriangleStates::LONG_FINGERTIP] = PICK;
  tri[TriangleStates::SHORT_WORLD][TriangleStates::SHORT_FINGERTIP] = PICK;
  tri[TriangleStates::SHORT_WORLD][TriangleStates::LONG_FLIP_FINGERTIP] = PICK;

  tri[TriangleStates::TRI_FINGERTIP][TriangleStates::TRI_WORLD] = PLACE;
  tri[TriangleStates::LONG_FINGERTIP][TriangleStates::LONG_WORLD] = PLACE;
  tri[TriangleStates::SHORT_FINGERTIP][TriangleStates::SHORT_WORLD] = PLACE;
  tri[TriangleStates::LONG_FLIP_PALM][TriangleStates::SHORT_WORLD] = PLACE;
  tri[TriangleStates::LONG_FLIP_FINGERTIP][TriangleStates::SHORT_WORLD] = PLACE;
  tri[TriangleStates::SHORT_FINGERTIP][TriangleStates::LONG_WORLD] = PLACE;

  tri[TriangleStates::TRI_FINGERTIP][TriangleStates::TRI_PALM] = THROW_TO_PALM;
  tri[TriangleStates::TRI_FINGERTIP][TriangleStates::LONG_WORLD] = ROLL_TO_GROUND;
  tri[TriangleStates::TRI_FINGERTIP][TriangleStates::LONG_FLIP_PALM] = ROLL_TO_PALM;

  tri[TriangleStates::TRI_PALM][TriangleStates::LONG_FLIP_PALM] = THROW_AND_FLIP;
  tri[TriangleStates::TRI_PALM][TriangleStates::TRI_FINGERTIP] = THROW_TO_FINGERTIP;

  tri[TriangleStates::LONG_FLIP_PALM][TriangleStates::LONG_FLIP_FINGERTIP] = THROW_TO_FINGERTIP;
  tri[TriangleStates::LONG_FLIP_FINGERTIP][TriangleStates::LONG_FLIP_PALM] = THROW_TO_PALM;

  tri[TriangleStates::SHORT_PALM][TriangleStates::SHORT_FINGERTIP] = THROW_TO_FINGERTIP;
  tri[TriangleStates::SHORT_FINGERTIP][TriangleStates::SHORT_PALM] = THROW_TO_PALM;

  tri[TriangleStates::SHORT_WORLD][TriangleStates::LONG_WORLD] = SHORT_EDGE_DROOP;
  tri[TriangleStates::LONG_WORLD][TriangleStates::SHORT_WORLD] = LONG_EDGE_DROOP;
  //tri[TriangleStates::LONG_WORLD][TriangleStates::TRI_WORLD] = TOPPLE;
  //tri[TriangleStates::SHORT_WORLD][TriangleStates::TRI_WORLD] = TOPPlE;

  tri_graph = tri;

  vector< vector< double > > cyl (NUM_CYL_STATES, vector< double > (NUM_CYL_STATES, 0.0) );
  cyl[CylinderStates::FLAT_PALM][CylinderStates::FLAT_FINGERTIP] = ROLL_TO_FINGERTIP;
  cyl[CylinderStates::FLAT_FINGERTIP][CylinderStates::FLAT_WORLD] = PLACE;
  cyl[CylinderStates::FLAT_FINGERTIP][CylinderStates::FLAT_PALM] = ROLL_TO_PALM;
  cyl[CylinderStates::TALL_FINGERTIP][CylinderStates::TALL_WORLD] = PLACE;
  cyl[CylinderStates::FLAT_WORLD][CylinderStates::FLAT_FINGERTIP] = PICK;
  cyl[CylinderStates::FLAT_WORLD][CylinderStates::TALL_FINGERTIP] = DROOP_IN_FINGERS;
  cyl[CylinderStates::TALL_WORLD][CylinderStates::TALL_FINGERTIP] = PICK;
  cyl_graph = cyl;

}

/************* DIJKSTRA AND HELPERS******************/
bool dijkstra(int num_states, vector< vector< double > > graph, int src, int goal, 
              map<int, double> actions, vector<int> &path)
{
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
	  double alt = dist[u] + actions[graph[u][v]];
	  
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
  bool ret = shortestPath(src, goal, previous, actions, graph, path);
  return ret;
}
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int num_states, double dist[], bool seen[])
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

// A utility function to print the constructed distance array
void printSolution(double dist[], int previous[], int n)
{
   printf("num_statesertex   Distance from Source    sptSet\n");
   for (int i = 0; i < n; i++)
     {
       cout << TriangleStateNames[i] << " " << dist[i] << " " << TriangleStateNames[previous[i]] << endl;
     }
}

bool shortestPath(int src, int goal, int previous[], map<int, double> actions, 
		  vector< vector< double > > graph, vector<int> &path)
{

  if (src==goal)
    return false;
  cout << "goal " << TriangleStateNames[goal] << endl;
  cout << "src " << TriangleStateNames[src] << endl;

  vector< int > state_path;
  int u = goal;
  int max_len = 5;
  int count = 0;
  while (previous[u] != src) 
    {
      cout << "ff " << previous[u] << " " << TriangleStateNames[u] << endl;
      if ((count >= max_len) || (previous[u]==-1))
	return false;
      // cout << u << " " << TriangleStateNames[u] << endl;
      // cout << previous[u] << " " << TriangleStateNames[previous[u]] << endl;
      // cout << "graph " << graph[previous[u]][u] << endl;
      state_path.push_back(previous[u]);
      path.push_back(graph[previous[u]][u]);
      cout << "adding " << previous[u] << " " << u << " " << graph[previous[u]][u] << endl;
      ROS_INFO("adding a step %f", graph[previous[u]][u]);
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

void printPlan(vector<regraspComm::Regrasp> plan)
{
  map<int, string> act;
  act[PICK] = "pick";
  act[PLACE] = "place";
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
  act[TOPPLE] = "topple";

  cout << "Plan: " << endl;
  for (int i=0; (unsigned)i<plan.size(); i++)
    cout << "Step i: " << act[plan[i].action] << endl;
  
}
void printPlan(vector<int> plan)
{
  map<int, string> act;
  act[PICK] = "pick";
  act[PLACE] = "place";
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

  for (int i=0; (unsigned)i<plan.size(); i++)
    cout << "Step i: " << act[plan[i]] << endl;
  
}
  // bool place_holder = false;
  // for (int i=0; i<num_blocks;i++)
  //   {
  //     bool suc = tryMove(goal_states[i], goal_objs[i], goal_vecs[i], goal_quats[i]);

  //     if (!suc)
  //       {
  //         ROS_INFO("FAILED ON BLOCK %d",i);
  //       }

  //   }

  // // IN TRYMOVE
  // // FIND START
  // int obj, vision_obj, state;
  // robot.SetJoints(25,0,0,0,90,0);
  // findBlockOnTable(tableVision, goal_obj, vision_obj, startV, startQ);
  // double start[7] = {startV[0], startV[1], startV[2], 
  //  		     startQ[0], startQ[1], startQ[2], startQ[3]};
  // findState(vision_obj, obj, state);
  
  // // FIND PLAN
  // if (goal_obj != obj)
  //   {
  //     ROS_INFO("WRONG BLOCK");
  //   }
  // else
  //   {  
  //     ROS_INFO("starting to plan");
  //     vector<int> plan;
  //     //      bool suc = updatePlan(obj, state, goal, plan);
  //     int l = plan.size();
  //     ROS_INFO("got a plan %d, %d", suc, l);

  //     // special case of start_state=goal_state: still must pick and place
  //     if (l==0)
  // 	{
  // 	  plan.push_back(PLACE);
  // 	  plan.push_back(PICK);
  // 	}
  //     l = plan.size();
  //     // printPlan(plan);
      
  //     // reverse the order and get defaults
  //     vector<regraspComm::Regrasp> rplan;
  //     for (int i=(l-1); i>=0; i--)
  // 	rplan.push_back(regrasp.getDefaults(plan[i]));

  //     map<int, double> actionsType;
  //     actionsType[0] = 0;
  //     actionsType[PICK] = ActionType::WORLD_TO_HAND;
  //     actionsType[PLACE] = ActionType::HAND_TO_WORLD;
  //     actionsType[ROLL_TO_GROUND] = ActionType::HAND_TO_WORLD;
  //     actionsType[THROW_TO_PALM] = ActionType::HAND_TO_HAND;
  //     actionsType[THROW_TO_FINGERTIP] = ActionType::HAND_TO_HAND;
  //     actionsType[THROW_AND_FLIP] = ActionType::HAND_TO_HAND;
  //     actionsType[ROLL_TO_PALM] = ActionType::HAND_TO_HAND;
  //     actionsType[DROOP_IN_FINGERS] = ActionType::WORLD_TO_HAND;
  //     actionsType[LONG_EDGE_DROOP] = ActionType::WORLD_TO_WORLD;
  //     actionsType[SHORT_EDGE_DROOP] = ActionType::WORLD_TO_WORLD;

  //     for (int i=0;i<l;i++)
  // 	{
  // 	  if (actionsType[rplan[i].action]==ActionType::WORLD_TO_WORLD)
  // 	    {
  // 	      geometry_msgs::Pose obj;
  // 	      obj.position.x = startV[0];
  // 	      obj.position.y = startV[1];
  // 	      obj.position.z = startV[2];
  // 	      obj.orientation.w = startQ[0];
  // 	      obj.orientation.x = startQ[1];
  // 	      obj.orientation.y = startQ[2];
  // 	      obj.orientation.z = startQ[3];
	      
  // 	      if (rplan[i].action==LONG_EDGE_DROOP)
  // 		rplan[i].longEdgeDroop.pick_pose = obj;
  // 	      if (rplan[i].action==SHORT_EDGE_DROOP)
  // 		rplan[i].shortEdgeDroop.pick_pose = obj;

  // 	    } 
  // 	  else if (actionsType[rplan[i].action]==ActionType::HAND_TO_WORLD)
  // 	    {
  // 	      geometry_msgs::Pose obj;
  // 	      obj.position.x = goalV[0];
  // 	      obj.position.y = goalV[1];
  // 	      obj.position.z = goalV[2];
  // 	      obj.orientation.w = goalQ[0];
  // 	      obj.orientation.x = goalQ[1];
  // 	      obj.orientation.y = goalQ[2];
  // 	      obj.orientation.z = goalQ[3];

  // 	      if (rplan[i].action==PLACE)
  // 		rplan[i].place.pose = obj;
  // 	      if (rplan[i].action==ROLL_TO_GROUND)
  // 		rplan[i].rollToGround.flip_pose = obj; 

  // 	    }
  // 	  else if (actionsType[rplan[i].action]==ActionType::WORLD_TO_HAND)
  // 	    {
  // 	      geometry_msgs::Pose obj;
  // 	      obj.position.x = startV[0];
  // 	      obj.position.y = startV[1];
  // 	      obj.position.z = startV[2];
  // 	      obj.orientation.w = startQ[0];
  // 	      obj.orientation.x = startQ[1];
  // 	      obj.orientation.y = startQ[2];
  // 	      obj.orientation.z = startQ[3];

  // 	      if (rplan[i].action==PICK)
  // 		rplan[i].pick.pose = obj;
  // 	      if (rplan[i].action==DROOP_IN_FINGERS)
  // 		rplan[i].droopInFingers.pose = obj;

  // 	    }	  
  // 	}


  //     printPlan(rplan);
  //     bool regrasp_success = regrasp.Execute(rplan);
  //     ROS_INFO("regrasp plan success: %d", regrasp_success);
  //     //vector<int> regrasps;
  //     // //regrasps.push
  //     //regrasps.push_back(LONG_EDGE_DROOP);
  //     //cout << "do it " << LONG_EDGE_DROOP << endl;
  //     //regrasp.Execute(regrasps);
  //   }
