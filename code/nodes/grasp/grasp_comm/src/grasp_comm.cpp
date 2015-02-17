#include "grasp_comm.h"

GraspComm::GraspComm() 
{
  nodePtr = NULL;
  currentGraspType = GM_TYPE_NONE;
}

GraspComm::GraspComm(ros::NodeHandle *n) 
{
  subscribe(n);
}

GraspComm::~GraspComm()
{
  shutdown();
}

void GraspComm::subscribe(ros::NodeHandle *n)
{
  nodePtr = n;
  currentGraspType = GM_TYPE_NONE;
  robot.subscribe(nodePtr);
  hand.subscribe(nodePtr);
}

void GraspComm::shutdown()
{
  //For Debugging
  //printf("IN DESTRUCTOR!!!\n");
  //For Debugging
  disable();
  closeNodes();
  robot.shutdown();
  hand.shutdown();
}

// Opens a node if it is not already opened
bool GraspComm::openNode(string nodeName)
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
    ROS_ERROR("GRASP_COMM: Could not open node %s wiht command rosrun.",nodeName.c_str());
    return false;
  }
  else
  {
    openNodes.push_back(nodeName);
    return true;
  }

  /*



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
      ROS_ERROR("GRASP_COMM: Could not open node %s wiht command rosrun.",nodeName.c_str());
      return false;
    }
    else
    {
      openNodes.push(nodeName);
    }
  }
  return true;
  */
}

bool GraspComm::closeNodes()
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
     printf("In close node\n");
  //We first check wether the corresponding node is running or not
  ros::V_string nodes;
  ros::master::getNodes(nodes);
  bool found = false;
  for(int i=0; i<(int)nodes.size(); i++)
  {
  printf("%s\n", nodes[i].c_str());
  size_t ind=nodes[i].find(nodeName);
  if(ind!=string::npos)
  found = true;
  }

  //If node found in the ROS network we close it
  if(found)
  {
  printf("found\n");
  char command[MAX_BUFFER];
  sprintf(command,"rosnode kill %s",nodeName.c_str());
  printf("command: %s\n", command);
  if(system(command))
  {
  ROS_ERROR("GRASP_COMM: Could not kill node %s wiht command rosnode kill.",nodeName.c_str());
  return false;
  }
  }
  return true;  
   */
}

bool GraspComm::disable()
{
  switch(currentGraspType)
  {

    case GM_TYPE_NONE:
      break;

    case GM_TYPE_USER:
      // Unsubscribe from service
      handle_grasp_User.shutdown();
      // Close node
      //closeNode(GM_USER_NAME);
      break;

    case GM_TYPE_OPENLOOP:
      // Unsubscribe from service
      handle_grasp_OpenLoop.shutdown();
      // Close node
      //closeNode(GM_OPENLOOP_NAME);
      break;

    case GM_TYPE_OPENLOOP_EARLYABORT:
      // Unsubscribe from service
      handle_grasp_OpenLoop_EarlyAbort.shutdown();
      // Close node
      //closeNode(GM_OPENLOOP_EARLYABORT_NAME);
      break;

    case GM_TYPE_ABORT:
      // Unsubscribe from service
      handle_grasp_Abort.shutdown();
      // Close node
      //closeNode(GM_GRASP_ABORT_NAME);
      break;
	
		case GM_TYPE_OPENLOOP_DETECT:
			handle_grasp_OpenLoop_Detect.shutdown();
			break; 

    default:
      currentGraspType = GM_TYPE_NONE;
      ROS_WARN("GRASP_COMM: Incorrect grasp Type selected.");
      return false;
  }
  return true;
}

bool GraspComm::enable(GM_GraspType graspType)
{
  //Only act if the grap type is different than the current one
  if(graspType!=currentGraspType)
  {
    //First we disable the current grasp type
    disable();
    switch(graspType)
    {

      case GM_TYPE_NONE:
        currentGraspType = GM_TYPE_NONE;
        break;

      case GM_TYPE_USER:
        // Open the grasp user node.
        openNode(GM_USER_NAME);
        // Connect to service 
        handle_grasp_User = nodePtr->serviceClient<grasp_comm::grasp_User>("grasp_User");
        // Wait until service is advertised and available.
        while (!handle_grasp_User.exists()) ;
        currentGraspType = GM_TYPE_USER;
        break;

      case GM_TYPE_OPENLOOP:
        // Open the grasp user node.
        openNode(GM_OPENLOOP_NAME);
        // Connect to service 
        handle_grasp_OpenLoop = nodePtr->serviceClient<grasp_comm::grasp_OpenLoop>("grasp_OpenLoop");
        // Wait until service is advertised and available.
        while (!handle_grasp_OpenLoop.exists()) ;
        currentGraspType = GM_TYPE_OPENLOOP;
        break;

			case GM_TYPE_OPENLOOP_DETECT:
				openNode(GM_OPENLOOP_DETECT_NAME);
				handle_grasp_OpenLoop_Detect = 
					nodePtr->serviceClient<grasp_comm::grasp_OpenLoop_Detect>
					("grasp_OpenLoop_Detect");
				while (!handle_grasp_OpenLoop_Detect.exists()) ;
				currentGraspType = GM_TYPE_OPENLOOP_DETECT;
				break;

      case GM_TYPE_OPENLOOP_EARLYABORT:
        // Open the grasp user node.
        openNode(GM_OPENLOOP_EARLYABORT_NAME);
        // Connect to service 
        handle_grasp_OpenLoop_EarlyAbort = nodePtr->serviceClient<grasp_comm::grasp_OpenLoop_EarlyAbort>("grasp_OpenLoop_EarlyAbort");
        // Wait until service is advertised and available.
        while (!handle_grasp_OpenLoop_EarlyAbort.exists()) ;
        currentGraspType = GM_TYPE_OPENLOOP_EARLYABORT;
        break;

      case GM_TYPE_ABORT:
        // Open the grasp user node.
        openNode(GM_GRASP_ABORT_NAME);
        // Connect to service 
        handle_grasp_Abort = nodePtr->serviceClient<grasp_comm::grasp_Abort>("grasp_Abort");
        // Wait until service is advertised and available.
        while (!handle_grasp_Abort.exists()) ;
        currentGraspType = GM_TYPE_ABORT;
        break;

      default:
        currentGraspType = GM_TYPE_NONE;
        ROS_WARN("GRASP_COMM: Incorrect grasp Type selected.");
        return false;
    }
  }
  return true;
}

GM_OutputParams GraspComm::grasp(GM_InputParams inParams)
{
  GM_OutputParams outParams;
  outParams.type = inParams.type;
  if(nodePtr!=NULL)
  {
    switch(inParams.type)
    {

      case GM_TYPE_NONE:
        break;

      case GM_TYPE_USER:
        enable(GM_TYPE_USER);
        grasp_User_srv.request.id = inParams.id;
        if(!handle_grasp_User.call(grasp_User_srv))
        {
          outParams.error = (GM_ErrorType)grasp_User_srv.response.error;
          ROS_WARN("GRASP_COMM: %s",grasp_User_srv.response.msg.c_str());
        }
        else
        {
          outParams.error = (GM_ErrorType)grasp_User_srv.response.error;
          outParams.logFileName = grasp_User_srv.response.logFileName;
        }
        break;

      case GM_TYPE_OPENLOOP:
        enable(GM_TYPE_OPENLOOP);
        grasp_OpenLoop_srv.request.x = inParams.x;
        grasp_OpenLoop_srv.request.y = inParams.y;
        grasp_OpenLoop_srv.request.z_lim = inParams.z_lim;
        grasp_OpenLoop_srv.request.up = inParams.up;
        grasp_OpenLoop_srv.request.down = inParams.down;
        grasp_OpenLoop_srv.request.nFlips = inParams.nFlips;
        grasp_OpenLoop_srv.request.oscillationAmplitude = inParams.oscillationAmplitude;
        grasp_OpenLoop_srv.request.id = inParams.id;
        grasp_OpenLoop_srv.request.handSpeed = inParams.handSpeed;
        if(!handle_grasp_OpenLoop.call(grasp_OpenLoop_srv))
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_srv.response.error;
          ROS_WARN("GRASP_COMM: %s",grasp_OpenLoop_srv.response.msg.c_str());
        }
        else
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_srv.response.error;
          outParams.logFileName = grasp_OpenLoop_srv.response.logFileName;
        }
        break;

      case GM_TYPE_OPENLOOP_DETECT:
        enable(GM_TYPE_OPENLOOP_DETECT);
        grasp_OpenLoop_Detect_srv.request.x = inParams.x;
        grasp_OpenLoop_Detect_srv.request.y = inParams.y;
        grasp_OpenLoop_Detect_srv.request.z_lim = inParams.z_lim;
        grasp_OpenLoop_Detect_srv.request.up = inParams.up;
        grasp_OpenLoop_Detect_srv.request.down = inParams.down;
        // grasp_OpenLoop_Detect_srv.request.nFlips = inParams.nFlips;
        // grasp_OpenLoop_Detect_srv.request.oscillationAmplitude = inParams.oscillationAmplitude;
        grasp_OpenLoop_Detect_srv.request.id = inParams.id;
        grasp_OpenLoop_Detect_srv.request.handSpeed = inParams.handSpeed;
        if(!handle_grasp_OpenLoop_Detect.call(grasp_OpenLoop_Detect_srv))
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_Detect_srv.response.error;
          ROS_WARN("GRASP_COMM: %s",grasp_OpenLoop_Detect_srv.response.msg.c_str());
        }
        else
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_Detect_srv.response.error;
          outParams.logFileName = grasp_OpenLoop_Detect_srv.response.logFileName;
        }
				break;

     case GM_TYPE_OPENLOOP_EARLYABORT:
        enable(GM_TYPE_OPENLOOP_EARLYABORT);
        grasp_OpenLoop_EarlyAbort_srv.request.x = inParams.x;
        grasp_OpenLoop_EarlyAbort_srv.request.y = inParams.y;
        grasp_OpenLoop_EarlyAbort_srv.request.z_lim = inParams.z_lim;
        grasp_OpenLoop_EarlyAbort_srv.request.up = inParams.up;
        grasp_OpenLoop_EarlyAbort_srv.request.down = inParams.down;
        grasp_OpenLoop_EarlyAbort_srv.request.nFlips = inParams.nFlips;
        grasp_OpenLoop_EarlyAbort_srv.request.oscillationAmplitude = inParams.oscillationAmplitude;
        grasp_OpenLoop_EarlyAbort_srv.request.id = inParams.id;
        grasp_OpenLoop_EarlyAbort_srv.request.handSpeed = inParams.handSpeed;
        if(!handle_grasp_OpenLoop_EarlyAbort.call(grasp_OpenLoop_EarlyAbort_srv))
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_EarlyAbort_srv.response.error;
          ROS_WARN("GRASP_COMM: %s",grasp_OpenLoop_EarlyAbort_srv.response.msg.c_str());
        }
        else
        {
          outParams.error = (GM_ErrorType)grasp_OpenLoop_EarlyAbort_srv.response.error;
          outParams.logFileName = grasp_OpenLoop_EarlyAbort_srv.response.logFileName;
        }
        break;

      case GM_TYPE_ABORT:
        enable(GM_TYPE_ABORT);
        grasp_Abort_srv.request.x = inParams.x;
        grasp_Abort_srv.request.y = inParams.y;
        grasp_Abort_srv.request.z_lim = inParams.z_lim;
        grasp_Abort_srv.request.up = inParams.up;
        grasp_Abort_srv.request.down = inParams.down;
        grasp_Abort_srv.request.nFlips = inParams.nFlips;
        grasp_Abort_srv.request.oscillationAmplitude = inParams.oscillationAmplitude;
        grasp_Abort_srv.request.id = inParams.id;
        if(!handle_grasp_Abort.call(grasp_Abort_srv))
        {
          outParams.error = (GM_ErrorType)grasp_Abort_srv.response.error;
          ROS_WARN("GRASP: %s",grasp_Abort_srv.response.msg.c_str());
        }
        else
        {
          outParams.error = (GM_ErrorType)grasp_Abort_srv.response.error;
          outParams.logFileName = grasp_Abort_srv.response.logFileName;
        }
        break;
      default:
        ROS_WARN("GRASP_COMM: Incorrect grasp type.");
        break;
    }
  }
  else
  {
    ROS_WARN("GRASP_COMM: Function grasp called before initializing the ROS node pointer.");
    outParams.error = GM_ERROR_UNINITIALIZED;
  }
  return outParams;  
}

