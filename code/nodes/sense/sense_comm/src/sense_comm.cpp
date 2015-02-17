#include "sense_comm.h"

SenseComm::SenseComm() 
{
  nodePtr = NULL;
  currentSenseType = SM_TYPE_NONE;
}

SenseComm::SenseComm(ros::NodeHandle *n) 
{
  subscribe(n);
}

SenseComm::~SenseComm()
{
  shutdown();
}

void SenseComm::subscribe(ros::NodeHandle *n)
{
  nodePtr = n;
  currentSenseType = SM_TYPE_NONE;
  openNodes.clear();
}

void SenseComm::shutdown()
{
  disable();
  closeNodes();
}

// Opens a node if it is not already opened
bool SenseComm::openNode(string nodeName)
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
    ROS_ERROR("GRASP_COMM: Could not open node %s with command rosrun.",nodeName.c_str());
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
    if(ind!=string::npos)
      found = true;
  }

  //If node not found in the ROS network we open it
  if(!found)
  {
    char command[MAX_BUFFER];
    sprintf(command,"rosrun %s %s &",nodeName.c_str(),nodeName.c_str());
    if(system(command))
    {
      ROS_ERROR("SENSE_COMM: Could not open node %s wiht command rosrun.",nodeName.c_str());
      return false;
    }
  }
  return true;
  */
}

bool SenseComm::closeNodes()
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
    if(ind!=string::npos)
      found = true;
  }

  //If node found in the ROS network we close it
  if(found)
  {
    char command[MAX_BUFFER];
    sprintf(command,"rosnode kill %s",nodeName.c_str());
    if(system(command))
    {
      ROS_ERROR("SENSE_COMM: Could not kill node %s wiht command rosnode kill.",nodeName.c_str());
      return false;
    }
  }
  return true;
  */
}

bool SenseComm::disable()
{
  switch(currentSenseType)
  {

    case SM_TYPE_NONE:
      break;

    case SM_TYPE_USER:
      // Unsubscribe from service
      handle_sense_User.shutdown();
      // Close node
      //closeNode(SM_USER_NAME);
      break;

    case SM_TYPE_VISION:
      // Unsubscribe from service
      handle_sense_Vision.shutdown();
      // Close node
      //closeNode(SM_VISION_NAME);
      break;

    case SM_TYPE_DATA:
      //Unsubscribe from service
      handle_sense_Data.shutdown();
      //Close node
      //closeNode(SM_DATA_NAME);
      break;

    case SM_TYPE_ABORT:
      //Unsubscribe from service
      handle_sense_Abort.shutdown();
      //Close node
      //closeNode(SM_ABORT_NAME);
      break;

    default:
      currentSenseType = SM_TYPE_NONE;
      ROS_WARN("SENSE_COMM: Incorrect grasp Type selected.");
      return false;
  }
  return true;
}

bool SenseComm::enable(SM_SenseType senseType)
{
  //Only act if the sense type is different than the current one
  if(senseType!=currentSenseType)
  {
    //First we disable the current grasp type
    disable();
    switch(senseType)
    {

      case SM_TYPE_NONE:
        currentSenseType = SM_TYPE_NONE;
        break;

      case SM_TYPE_USER:
        // Open the sense user node.
        openNode(SM_USER_NAME);
        // Connect to service 
        handle_sense_User = nodePtr->serviceClient<sense_comm::sense_User>("sense_User");
        // Wait until service is advertised and available.
        while (!handle_sense_User.exists()) ;
        currentSenseType = SM_TYPE_USER;
        break;

      case SM_TYPE_VISION:
        // Open the sense user node.
        openNode(SM_VISION_NAME);
        // Connect to service 
        handle_sense_Vision = nodePtr->serviceClient<sense_comm::sense_Vision>("sense_Vision");
        // Wait until service is advertised and available.
        while (!handle_sense_Vision.exists()) ;
        currentSenseType = SM_TYPE_VISION;
        break;

      case SM_TYPE_DATA:
        // Open the sense user node.
        openNode(SM_DATA_NAME);
        // Connect to service 
        handle_sense_Data = nodePtr->serviceClient<sense_comm::sense_Data>("sense_Data");
        // Wait until service is advertised and available.
        while (!handle_sense_Data.exists()) ;
        currentSenseType = SM_TYPE_DATA;
        break;

      case SM_TYPE_ABORT:
        // Open the sense user node.
        openNode(SM_ABORT_NAME);
        // Connect to service 
        handle_sense_Abort = nodePtr->serviceClient<sense_comm::sense_Abort>("sense_Abort");
        // Wait until service is advertised and available.
        while (!handle_sense_Abort.exists()) ;
        currentSenseType = SM_TYPE_ABORT;
        break;

      default:
        currentSenseType = SM_TYPE_NONE;
        ROS_WARN("SENSE_COMM: Incorrect sense Type selected.");
        return false;
    }
  }
  return true;
}

SM_OutputParams SenseComm::sense(SM_InputParams inParams)
{
  SM_OutputParams outParams;
  if(nodePtr!=NULL)
  {
    switch(inParams.type)
    {

      case SM_TYPE_NONE:
        break;

      case SM_TYPE_USER:
        enable(SM_TYPE_USER);
        sense_User_srv.request.logFileName = inParams.logFileName;
        if(!handle_sense_User.call(sense_User_srv))
        {
          outParams.type = SM_TYPE_ERROR;
          ROS_WARN("SENSE_COMM: %s",sense_User_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = SM_TYPE_USER;
          outParams.logFileName = sense_User_srv.response.logFileName;
          outParams.singulated = sense_User_srv.response.singulated;
          outParams.nMarkers = sense_User_srv.response.nMarkers;
          outParams.x = sense_User_srv.response.x;
          outParams.y = sense_User_srv.response.y;
          outParams.theta = sense_User_srv.response.theta;
          outParams.alpha = sense_User_srv.response.alpha;
          outParams.distance = sense_User_srv.response.distance;
          outParams.confidence = 1.0; // Set user confidence to 100%
        }
        break;

      case SM_TYPE_VISION:
        enable(SM_TYPE_VISION);
        sense_Vision_srv.request.logFileName = inParams.logFileName;
        if(!handle_sense_Vision.call(sense_Vision_srv))
        {
          outParams.type = SM_TYPE_ERROR;
          ROS_WARN("SENSE_COMM: %s",sense_Vision_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = SM_TYPE_VISION;
          outParams.logFileName = sense_Vision_srv.response.logFileName;
          outParams.singulated = sense_Vision_srv.response.singulated;
          outParams.nMarkers = sense_Vision_srv.response.nMarkers;
          outParams.x = sense_Vision_srv.response.x;
          outParams.y = sense_Vision_srv.response.y;
          outParams.theta = sense_Vision_srv.response.theta;
          outParams.alpha = sense_Vision_srv.response.alpha;
          outParams.distance = sense_Vision_srv.response.distance;
          outParams.picFileName = sense_Vision_srv.response.picFileName;
          outParams.confidence = sense_Vision_srv.response.confidence;
        }
        break;

      case SM_TYPE_DATA:
        enable(SM_TYPE_DATA);
        sense_Data_srv.request.logFileName = inParams.logFileName;
        sense_Data_srv.request.singulationModelName = inParams.singulationModelName;
        sense_Data_srv.request.poseEstimationModelName = inParams.poseEstimationModelName;
        if(!handle_sense_Data.call(sense_Data_srv))
        {
          outParams.type = SM_TYPE_ERROR;
          ROS_WARN("SENSE_COMM: %s",sense_Data_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = SM_TYPE_DATA;
          outParams.logFileName = sense_Data_srv.response.logFileName;
          outParams.singulated = sense_Data_srv.response.singulated;
          outParams.nMarkers = sense_Data_srv.response.nMarkers;
          outParams.x = sense_Data_srv.response.x;
          outParams.y = sense_Data_srv.response.y;
          outParams.theta = sense_Data_srv.response.theta;
          outParams.alpha = sense_Data_srv.response.alpha;
          outParams.distance = sense_Data_srv.response.distance;
          outParams.confidence = sense_Data_srv.response.confidence;
        }
        break;

      case SM_TYPE_ABORT:
        enable(SM_TYPE_ABORT);
        sense_Abort_srv.request.logFileName = inParams.logFileName;
        sense_Abort_srv.request.modelName = inParams.earlyAbortModelName;
        if(!handle_sense_Abort.call(sense_Abort_srv))
        {
          outParams.type = SM_TYPE_ERROR;
          ROS_WARN("SENSE_COMM: %s",sense_Abort_srv.response.msg.c_str());
        }
        else
        {
          outParams.type = SM_TYPE_ABORT;
          outParams.logFileName = sense_Abort_srv.response.logFileName;
          outParams.singulated = sense_Abort_srv.response.singulated;
          outParams.nMarkers = sense_Abort_srv.response.nMarkers;
          outParams.x = sense_Abort_srv.response.x;
          outParams.y = sense_Abort_srv.response.y;
          outParams.theta = sense_Abort_srv.response.theta;
          outParams.alpha = sense_Abort_srv.response.alpha;
          outParams.distance = sense_Abort_srv.response.distance;
          outParams.confidence = sense_Abort_srv.response.confidence;
        }
        break;

      default:
        ROS_WARN("SENSE_COMM: Incorrect sensor type.");
        break;
    }
  }
  else
  {
    ROS_WARN("SENSE_COMM: Function ense called before initializing the ROS node pointer.");
    outParams.type = SM_TYPE_ERROR;
  }
  return outParams;  
}


