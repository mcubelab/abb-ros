#include "place_comm.h"

PlaceComm::PlaceComm() 
{
  nodePtr = NULL;
  currentPlaceType = PM_TYPE_NONE;
}

PlaceComm::PlaceComm(ros::NodeHandle *n) 
{
  subscribe(n);
}

PlaceComm::~PlaceComm()
{
  shutdown();
}

void PlaceComm::subscribe(ros::NodeHandle *n)
{
  nodePtr = n;
  currentPlaceType = PM_TYPE_NONE;
  openNodes.clear();
}

void PlaceComm::shutdown()
{
  disable();
  closeNodes();
}

// Opens a node if it is not already opened
bool PlaceComm::openNode(std::string nodeName)
{
  for (vector<string>::size_type i = 0; i < openNodes.size(); i++)
  {
    if (openNodes[i].compare(nodeName) == 0)
      return true;
  }

  char command[MAX_BUFFER];
  sprintf(command,"rosrun %s %s &",nodeName.c_str(),nodeName.c_str());
  if(system(command))
  {
    ROS_ERROR("PLACE_COMM: Could not open node %s with command rosrun.",nodeName.c_str());
    return false;
  }
  else
  {
    openNodes.push_back(nodeName);
    return true;
  }
/*
  int i;

  //We first check wether the corresponding node is running or not
  ros::V_string nodes;
  ros::master::getNodes(nodes);
  bool found = false;
  for(i=0; i<(int)nodes.size(); i++)
  {
    size_t ind=nodes[i].find(nodeName);
    if(ind!=std::string::npos)
      found = true;
  }

  //If node not found in the ROS network we open it
  if(!found)
  {
    char command[MAX_BUFFER];
    sprintf(command,"rosrun %s %s &",nodeName.c_str(),nodeName.c_str());
    if(system(command))
    {
      ROS_ERROR("PLACE_COMM: Could not open node %s with command rosrun.",nodeName.c_str());
      return false;
    }
  }
  return true;
  */
}

bool PlaceComm::closeNodes()
{
  char command[MAX_BUFFER];
  bool success = true;
  for (vector<string>::size_type i=0; i < openNodes.size(); i++)
  {
    sprintf(command,"pkill %s >/dev/null 2>&1",openNodes[i].c_str());
    if (~system(command))
    {
      ROS_ERROR("GRASP_COMM: Could not kill node with command: %s",command);
      success = false;
    }
  }

  return success;
  /*
  //We first check wether the corresponding node is running or not
  ros::V_string nodes;
  ros::master::getNodes(nodes);
  bool found = false;
  for(int i=0; i<(int)nodes.size(); i++)
  {
    size_t ind=nodes[i].find(nodeName);
    if(ind!=std::string::npos)
      found = true;
  }

  //If node found in the ROS network we close it
  if(found)
  {
    char command[MAX_BUFFER];
    sprintf(command,"rosnode kill %s",nodeName.c_str());
    if(system(command))
    {
      ROS_ERROR("PLACE_COMM: Could not kill node %s with command rosnode kill.",nodeName.c_str());
      return false;
    }
  }
  return true;  
  */
}

bool PlaceComm::disable()
{
  switch(currentPlaceType)
  {

    case PM_TYPE_NONE:
      break;

    case PM_TYPE_OPENLOOP:
      // Unsubscribe from service
      handle_place_OpenLoop.shutdown();
      // Close node
      //closeNode(PM_PLACE_OPENLOOP_NAME);
      break;

    case PM_TYPE_DROP:
      // Unsubscribe from service
      handle_drop_OpenLoop.shutdown();
      break;

    case PM_TYPE_INSERT:
      // Unsubscribe from service
      handle_insert_OpenLoop.shutdown();
      break;

    default:
      currentPlaceType = PM_TYPE_NONE;
      ROS_WARN("PLACE: Incorrect place type selected.");
      return false;
  }
  return true;
}

bool PlaceComm::enable(PM_PlaceType placeType)
{
  //Only act if the place type is different than the current one
  if(placeType!=currentPlaceType)
  {
    //First we disable the current place type
    disable();

    // Now, we open the new node and save the service client
    switch(placeType)
    {

      case PM_TYPE_NONE:
        currentPlaceType = PM_TYPE_NONE;
        break;

      case PM_TYPE_OPENLOOP:
        // Open the open loop placing node.
        openNode(PM_PLACE_OPENLOOP_NAME);
        // Connect to service 
        handle_place_OpenLoop = nodePtr->serviceClient<place_comm::place_OpenLoop>("place_OpenLoop");
        // Wait until service is advertised and available.
        while (!handle_place_OpenLoop.exists());
        currentPlaceType = PM_TYPE_OPENLOOP;
        break;

      case PM_TYPE_DROP:
        // Open the open loop placing node.
        openNode(PM_DROP_OPENLOOP_NAME);
        // Connect to service 
        handle_drop_OpenLoop = nodePtr->serviceClient<place_comm::drop_OpenLoop>("drop_OpenLoop");
        // Wait until service is advertised and available.
        while (!handle_drop_OpenLoop.exists());
        currentPlaceType = PM_TYPE_DROP;
        break;

      case PM_TYPE_INSERT:
        // Open the open loop placing node.
        openNode(PM_INSERT_OPENLOOP_NAME);
        // Connect to service 
        handle_insert_OpenLoop = nodePtr->serviceClient<place_comm::insert_OpenLoop>("insert_OpenLoop");
        // Wait until service is advertised and available.
        while (!handle_insert_OpenLoop.exists());
        currentPlaceType = PM_TYPE_INSERT;
        break;

      default:
        currentPlaceType = PM_TYPE_NONE;
        ROS_WARN("PLACE: Incorrect place type selected.");
        return false;
    }
  }
  return true;
}

PM_OutputParams PlaceComm::place(PM_InputParams inParams)
{
  PM_OutputParams outParams;
  if(nodePtr!=NULL)
  {
    switch(inParams.type)
    {

      case PM_TYPE_NONE:
        break;

      case PM_TYPE_OPENLOOP:
        enable(PM_TYPE_OPENLOOP);
        place_OpenLoop_srv.request.angle = inParams.angle;
        place_OpenLoop_srv.request.h_dist = inParams.h_dist;
        place_OpenLoop_srv.request.v1_dist = inParams.v1_dist;
        place_OpenLoop_srv.request.v2_dist = inParams.v2_dist;
        place_OpenLoop_srv.request.test = inParams.test;

        if(!handle_place_OpenLoop.call(place_OpenLoop_srv))
        {
          outParams.type = PM_TYPE_ERROR;
          ROS_WARN("PLACE: %s", place_OpenLoop_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = PM_TYPE_OPENLOOP;
          outParams.logFileName = place_OpenLoop_srv.response.logFileName;
          outParams.picFileName = place_OpenLoop_srv.response.picFileName;
          outParams.success = place_OpenLoop_srv.response.success;
        }
        break;

      case PM_TYPE_DROP:
        enable(PM_TYPE_DROP);
        drop_OpenLoop_srv.request.angle = inParams.angle;
        drop_OpenLoop_srv.request.h_dist = inParams.h_dist;
        drop_OpenLoop_srv.request.test = inParams.test;

        if(!handle_drop_OpenLoop.call(drop_OpenLoop_srv))
        {
          outParams.type = PM_TYPE_ERROR;
          outParams.logFileName = drop_OpenLoop_srv.response.logFileName;
          outParams.picFileName = drop_OpenLoop_srv.response.picFileName;
          outParams.success = drop_OpenLoop_srv.response.success;
          ROS_WARN("PLACE: %s", drop_OpenLoop_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = PM_TYPE_DROP;
          outParams.logFileName = drop_OpenLoop_srv.response.logFileName;
          outParams.picFileName = drop_OpenLoop_srv.response.picFileName;
          outParams.success = drop_OpenLoop_srv.response.success;
        }
        break;        

      case PM_TYPE_INSERT:
        enable(PM_TYPE_INSERT);
        insert_OpenLoop_srv.request.angle = inParams.angle;
        insert_OpenLoop_srv.request.h_dist = inParams.h_dist;
        insert_OpenLoop_srv.request.test = inParams.test;

        if(!handle_insert_OpenLoop.call(insert_OpenLoop_srv))
        {
          outParams.type = PM_TYPE_ERROR;
          outParams.logFileName = insert_OpenLoop_srv.response.logFileName;
          outParams.picFileName = insert_OpenLoop_srv.response.picFileName;
          outParams.success = insert_OpenLoop_srv.response.success;
          outParams.abort = insert_OpenLoop_srv.response.abort;
          ROS_WARN("PLACE: %s", insert_OpenLoop_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = PM_TYPE_INSERT;
          outParams.logFileName = insert_OpenLoop_srv.response.logFileName;
          outParams.picFileName = insert_OpenLoop_srv.response.picFileName;
          outParams.success = insert_OpenLoop_srv.response.success;
          outParams.abort = insert_OpenLoop_srv.response.abort;
        }
        break;    

      default:
        ROS_WARN("PLACE: Incorrect place type.");
        break;
    }
  }
  else
  {
    ROS_WARN("PLACE: Function place called before initializing the ROS node pointer.");
    outParams.type = PM_TYPE_ERROR;
  }
  return outParams;  
}

