//
// Robot Node
//
// This node handles communication with the ABB robot. It serves 2 main
// functions. First, it communicates with the logging task on the ABB
// robot and broadcasts position and force information on topics. Second,
// it allows the user to control the robot, and allows for both blocking
// and non-blocking moves.
//

#include "robot_node.h"

/////////////////////////////////
// BEGIN RobotController Class //
/////////////////////////////////

RobotController::RobotController(ros::NodeHandle *n) 
{
  node = n;
}

RobotController::~RobotController() {
  /// Shut down services.
  handle_robot_Ping.shutdown();
  handle_robot_SetCartesian.shutdown();
  handle_robot_SetCartesianJ.shutdown();
  handle_robot_GetCartesian.shutdown();
  handle_robot_SetJoints.shutdown();
  handle_robot_GetJoints.shutdown();
  handle_robot_GetIK.shutdown();
  handle_robot_GetFK.shutdown();
  handle_robot_Stop.shutdown();
  handle_robot_SetTool.shutdown();
  handle_robot_SetInertia.shutdown();
  handle_robot_SetWorkObject.shutdown();
  handle_robot_SetSpeed.shutdown();
  handle_robot_SetAcc.shutdown();
  handle_robot_GetState.shutdown();
  handle_robot_SetZone.shutdown();
  handle_robot_SetTrackDist.shutdown();
  handle_robot_SetComm.shutdown();
  handle_robot_SetVacuum.shutdown();
  handle_robot_SpecialCommand.shutdown();
  handle_robot_SetDefaults.shutdown();
  handle_robot_Approach.shutdown();

  // Shut down topics.
  handle_robot_CartesianLog.shutdown();
  handle_robot_JointsLog.shutdown();
  handle_robot_ForceLog.shutdown();
  handle_robot_RRIJointState.shutdown();
  handle_robot_RRICartState.shutdown();
  
  char message[MAX_BUFFER];
  
  //Close RRI connections
  if(RRIConnected) {
    strcpy(message, ABBInterpreter::closeRRI().c_str());
    send(robotMotionSocket, message, strlen(message), 0);
  }

  //Close connections
  strcpy(message, ABBInterpreter::closeConnection().c_str());
  send(robotMotionSocket, message, strlen(message), 0);
  
  ros::Duration(1.0).sleep();
  close(robotMotionSocket);
  close(robotLoggerSocket);
}

// This method initializes the robot controller by connecting to the 
// Robot Motion and Robot Logger servers, setting up internal variables
// and setting the default robot configuration
bool RobotController::init(std::string id)
{
  std::string robotIp;
  int robotMotionPort;
  int robotLoggerPort;
  int hostRRIPort;

  motionConnected = false;
  loggerConnected = false;

  //Connection to Robot Motion server
  ROS_INFO("ROBOT_CONTROLLER: Connecting to the ABB motion server...");
  robotname_sl = "/robot" + id;
  robotname = "robot" + id;
  
  node->getParam(robotname_sl + "/robotIp",robotIp);
  node->getParam(robotname_sl + "/robotMotionPort",robotMotionPort);
  bool useRRI;
  node->param<bool>(robotname_sl + "/useRRI", useRRI, false);
  bool useLogger;
  node->param<bool>(robotname_sl + "/useLogger", useLogger, false);
  
  connectMotionServer(robotIp.c_str(), robotMotionPort);
  if(!motionConnected)
  {
    return false; 
  }
  
  if (useLogger) {
    //Connect to Robot Logger server
    ROS_INFO("ROBOT_CONTROLLER: Connecting to the ABB logger server...");
    node->getParam(robotname_sl + "/robotLoggerPort",robotLoggerPort);
    connectLoggerServer(robotIp.c_str(), robotLoggerPort);
    if(!loggerConnected)
    {
    ROS_INFO("ROBOT_CONTROLLER: Not able to connect to logger server. "
        "Continuing without robot feedback.");
    }
  } 


  
  // Setup our non-blocking variables
  non_blocking = false;
  do_nb_move = false;
  targetChanged = false;
  stopRequest = false;
  stopConfirm = false;
  changing_nb_speed = false;

  // Allocate space for all of our vectors
  curToolP = Vec(3);
  curToolCG = Vec(3);
  curToolI = Vec(3);
  curWorkP = Vec(3);
  curP = Vec(3);
  curGoalP = Vec(3);
  curTargP = Vec(3);

  // Set the Default Robot Configuration
  ROS_INFO("ROBOT_CONTROLLER: Setting robot default configuration...");
  if(!defaultRobotConfiguration())
  {
    ROS_INFO("ROBOT_CONTROLLER: Not able to set the robot to "
        "default configuration.");
    return false;
  }

  if(useRRI){
    //Establish RRI connection
    node->getParam(robotname_sl + "/hostRRIPort",hostRRIPort);
    ROS_INFO("ROBOT_CONTROLLER: Establishing RRI connection on port %d...", hostRRIPort);
    establishRRI(hostRRIPort);
    if(!RRIConnected)
    {
      ROS_INFO("ROBOT_CONTROLLER: Not able to establish RRI. "
          "Continuing without RRI.");
    }
  }
  
  return true;
}

// This function initializes the default configuration of the robot.
// It sets the work object, tool, zone, speed, and vacuum based on
// default parameters from the ROS parameter file
bool RobotController::defaultRobotConfiguration()
{
  double defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz;
  double defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz;
  double defTmass, defTCGx, defTCGy, defTCGz;
  double defTIx, defTIy, defTIz; 

  int zone;
  double speedTCP, speedORI;
 
  //WorkObject
  node->getParam(robotname_sl + "/workobjectX",defWOx);
  node->getParam(robotname_sl + "/workobjectY",defWOy);
  node->getParam(robotname_sl + "/workobjectZ",defWOz);
  node->getParam(robotname_sl + "/workobjectQ0",defWOq0);
  node->getParam(robotname_sl + "/workobjectQX",defWOqx);
  node->getParam(robotname_sl + "/workobjectQY",defWOqy);
  node->getParam(robotname_sl + "/workobjectQZ",defWOqz);
  if (!setWorkObject(defWOx,defWOy,defWOz,defWOq0,defWOqx,defWOqy,defWOqz)){
    return false;
  }

  //Tool
  node->getParam(robotname_sl + "/toolX",defTx);
  node->getParam(robotname_sl + "/toolY",defTy);
  node->getParam(robotname_sl + "/toolZ",defTz);
  node->getParam(robotname_sl + "/toolQ0",defTq0);
  node->getParam(robotname_sl + "/toolQX",defTqx);
  node->getParam(robotname_sl + "/toolQY",defTqy);
  node->getParam(robotname_sl + "/toolQZ",defTqz);
  if (!setTool(defTx,defTy,defTz,defTq0,defTqx,defTqy,defTqz))
      return false;
  
  //Inertia
  node->getParam(robotname_sl + "/toolMass",defTmass);
  node->getParam(robotname_sl + "/toolCGX",defTCGx);
  node->getParam(robotname_sl + "/toolCGY",defTCGy);
  node->getParam(robotname_sl + "/toolCGZ",defTCGz);
  node->getParam(robotname_sl + "/toolIX",defTIx);
  node->getParam(robotname_sl + "/toolIY",defTIy);
  node->getParam(robotname_sl + "/toolIZ",defTIz);
  if (!setInertia(defTmass,defTCGx,defTCGy,defTCGz,defTIx,defTIy,defTIz))
    return false;
  
  //Zone
  node->getParam(robotname_sl + "/zone",zone);
  if (!setZone(zone))
    return false;

  //Speed
  node->getParam(robotname_sl + "/speedTCP",speedTCP);
  node->getParam(robotname_sl + "/speedORI",speedORI);
  if (!setSpeed(speedTCP, speedORI))
    return false;

  //Vacuum
  //node->getParam(robotname_sl + "/vacuum",vacuumMode);
  //if (!setVacuum(vacuumMode))
  //  return false;

  //Communication mode: default is blocking
  if (!setComm(BLOCKING))
    return false;

  // If everything is set, our default configuration has been set up correctly
  return true;
}


//////////////////////////////////////////////////////////////////////////////
// Advertise Services and Topics
//////////////////////////////////////////////////////////////////////////////

// Advertise the topics that the robot node will be broadcasting
void RobotController::advertiseTopics()
{
  handle_robot_RosJointState = 
    node->advertise<sensor_msgs::JointState>(robotname + "_RosJointState", 100);
  handle_robot_CartesianLog = 
    node->advertise<robot_comm::robot_CartesianLog>(robotname + "_CartesianLog", 100);
  handle_robot_JointsLog = 
    node->advertise<robot_comm::robot_JointsLog>(robotname + "_JointsLog", 100);
  handle_robot_ForceLog = 
    node->advertise<robot_comm::robot_ForceLog>(robotname + "_ForceLog", 100);
  handle_robot_RRIJointState = 
    node->advertise<sensor_msgs::JointState>(robotname + "_RRIJointState", 100);
  handle_robot_RRICartState = 
    node->advertise<sensor_msgs::JointState>(robotname + "_RRICartState", 100);
}

// Advertise the services that the robot will be listening for
void RobotController::advertiseServices()
{
  handle_robot_Ping = node->advertiseService(robotname+"_Ping", 
      &RobotController::robot_Ping, this);
  handle_robot_SetCartesian = node->advertiseService(robotname + "_SetCartesian", 
      &RobotController::robot_SetCartesian, this);
  handle_robot_SetCartesianJ = node->advertiseService(robotname + "_SetCartesianJ", 
      &RobotController::robot_SetCartesianJ, this);
  handle_robot_GetCartesian = node->advertiseService(robotname + "_GetCartesian", 
      &RobotController::robot_GetCartesian, this);
  handle_robot_SetJoints = node->advertiseService(robotname + "_SetJoints", 
      &RobotController::robot_SetJoints, this);
  handle_robot_GetJoints = node->advertiseService(robotname + "_GetJoints", 
      &RobotController::robot_GetJoints, this);
  handle_robot_GetIK = node->advertiseService(robotname + "_GetIK", 
      &RobotController::robot_GetIK, this);
  handle_robot_GetFK = node->advertiseService(robotname + "_GetFK", 
      &RobotController::robot_GetFK, this);
  handle_robot_Stop = node->advertiseService(robotname + "_Stop", 
      &RobotController::robot_Stop, this);
  handle_robot_SetTool = node->advertiseService(robotname + "_SetTool", 
      &RobotController::robot_SetTool, this);
  handle_robot_SetInertia = node->advertiseService(robotname + "_SetInertia", 
      &RobotController::robot_SetTool, this);
  handle_robot_SetWorkObject = node->advertiseService(robotname + "_SetWorkObject", 
      &RobotController::robot_SetWorkObject, this);
  handle_robot_SetSpeed = node->advertiseService(robotname + "_SetSpeed", 
      &RobotController::robot_SetSpeed, this);
  handle_robot_SetAcc = node->advertiseService(robotname + "_SetAcc", 
      &RobotController::robot_SetAcc, this);
  handle_robot_GetState = node->advertiseService(robotname + "_GetState", 
      &RobotController::robot_GetState, this);
  handle_robot_SetZone = node->advertiseService(robotname + "_SetZone", 
      &RobotController::robot_SetZone, this);
  handle_robot_SetTrackDist = node->advertiseService(robotname + "_SetTrackDist", 
      &RobotController::robot_SetTrackDist, this);
  handle_robot_SetComm = node->advertiseService(robotname + "_SetComm", 
      &RobotController::robot_SetComm, this);
  handle_robot_SpecialCommand = node->advertiseService(robotname + "_SpecialCommand",
      &RobotController::robot_SpecialCommand, this);
  handle_robot_SetVacuum = node->advertiseService(robotname + "_SetVacuum", 
      &RobotController::robot_SetVacuum, this);
  handle_robot_IsMoving = node->advertiseService(robotname + "_IsMoving", 
      &RobotController::robot_IsMoving, this);
  handle_robot_SetDefaults = node->advertiseService(robotname + "_SetDefaults", 
      &RobotController::robot_SetDefaults, this);
  handle_robot_Approach = node->advertiseService(robotname + "_Approach",
      &RobotController::robot_Approach, this);
      // buffer code
  handle_robot_AddJointPosBuffer = node->advertiseService(robotname+"_AddJointPosBuffer", 
      &RobotController::robot_AddJointPosBuffer, this);
  handle_robot_ExecuteJointPosBuffer = node->advertiseService(robotname+"_ExecuteJointPosBuffer", 
      &RobotController::robot_ExecuteJointPosBuffer, this);
  handle_robot_ClearJointPosBuffer = node->advertiseService(robotname+"_ClearJointPosBuffer", 
      &RobotController::robot_ClearJointPosBuffer, this);
      
  handle_robot_AddBuffer = node->advertiseService(robotname+"_AddBuffer", 
      &RobotController::robot_AddBuffer, this);
  handle_robot_ExecuteBuffer = node->advertiseService(robotname+"_ExecuteBuffer", 
      &RobotController::robot_ExecuteBuffer, this);
  handle_robot_ClearBuffer = node->advertiseService(robotname+"_ClearBuffer", 
      &RobotController::robot_ClearBuffer, this);
}



//////////////////////////////////////////////////////////////////////////////
// Service Callbacks
//////////////////////////////////////////////////////////////////////////////

bool RobotController::robot_Approach(robot_comm::robot_Approach::Request& req, 
    robot_comm::robot_Approach::Response& res)
{
  // double x = req.grasp_pose.position.x;
  // double y = req.grasp_pose.position.y;
  // double z = req.grasp_pose.position.z;
  // double q0 = req.grasp_pose.orientation.w;
  // double q1 = req.grasp_pose.orientation.x;
  // double q2 = req.grasp_pose.orientation.y;
  // double q3 = req.grasp_pose.orientation.z;
  // Vec pos = Vec(x,y,z,3);
  // Quaternion quat = Quaternion(q0,q1,q2,q3);

  // // waypoints
  // SetJoints(0.0,0.0,0.0,0.0,90.0,0.0);
  // SetCartesian(600,200,300,0.0,0.707,0.707,0.0);
  
  // if (req.top_approach)
  //   {
  //     SetCartesian(x,y,z+100,q0,q1,q2,q3);
  //     SetCartesian(x,y,z,q0,q1,q2,q3);
      
  //   }
  // else // side approach
  //   {
  //     matlab.sendVec("quat", quat);
  //     matlab.sendCommand("[grasp, angle] = findLongDroopQuat(quat);");
  //     Quaternion q = matlab.getQuaternion("grasp");
  //     double angle = matlab.getValue("angle");

  //     double offset_x = cos(angle);
  //     double offset_y = sin(angle);

  //     Vec v (3);
  //     v[0] = x+offset_x;
  //     v[1] = y+offset_y;
  //     v[2] = z; 

  //     // check that approach/grasp is possible
  //     HomogTransf approach = HomogTransf(q,v);
  //     double approach_joints[6];
  //     bool suc = robot.GetIK(approach, approach_joints);
      
  //     robot.SetCartesian(v[0], v[1], v[2]+50, quat[0], quat[1], quat[2], quat[3]);

  //   }

  return true;
}

// Simply pings the robot and makes sure we can still communicate with it
bool RobotController::robot_Ping(robot_comm::robot_Ping::Request& req, 
    robot_comm::robot_Ping::Response& res)
{
  if (ping())
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to ping the robot.";
    return false;
  }
}


// Moves the robot to a given cartesian position using a joint move. If we are currently in
// non-blocking mode, then we simply setup the move and let the non-blocking
// thread handle the actual moving. If we are in blocking mode, we then 
// communicate with the robot to execute the move.
bool RobotController::robot_SetCartesianJ(
    robot_comm::robot_SetCartesianJ::Request& req, 
    robot_comm::robot_SetCartesianJ::Response& res)
{
  /*
  ROS_INFO("CMD: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz, 
      sqrt(req.q0*req.q0 + req.qx*req.qx + req.qy*req.qy + req.qz*req.qz));
      */
  // If we are in non-blocking mode
  if (non_blocking)
  {
    // As we are changing some state variables, mutex this block to be sure
    // we don't get unpredictable actions
    pthread_mutex_lock(&nonBlockMutex);
    if (!do_nb_move)
    {
      // If no move is currently being executed, we will now be doing
      // a cartesian move.
      cart_move = true;
      cart_move_j = true;

      // Our new target is simply the target passed by the user
      curTargP[0] = req.x;
      curTargP[1] = req.y;
      curTargP[2] = req.z;
      curTargQ[0] = req.q0;
      curTargQ[1] = req.qx;
      curTargQ[2] = req.qy;
      curTargQ[3] = req.qz;

      // The last goal in the non-blocking move is simply the current position
      getCartesian(curGoalP[0], curGoalP[1], curGoalP[2],
          curGoalQ[0], curGoalQ[1], curGoalQ[2], curGoalQ[3]);

      // We are now ready to execute this non-blocking move
      do_nb_move = true;
    }
    else if (cart_move)
    {
      cart_move_j = true;
      // If we are currently doing a non-blocking cartesian move, we simply
      // need to update our current target
      curTargP[0] = req.x;
      curTargP[1] = req.y;
      curTargP[2] = req.z;
      curTargQ[0] = req.q0;
      curTargQ[1] = req.qx;
      curTargQ[2] = req.qy;
      curTargQ[3] = req.qz;
      
      // Remember that we changed our target, again to maintain concurrency
      targetChanged = true;
    }
    else
    {
      // If we are doing a non-blocking move, but it's a joint move,
      // it would be much too dangerous to switch
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Can't do a cartesian move while doing a ";
      res.msg += "non-blocking joint move!";
      return false;
    }
    pthread_mutex_unlock(&nonBlockMutex);
  }
  else
  {
    // If we are in blocking mode, simply execute the cartesian move
    if (!setCartesianJ(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
    {
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Not able to set cartesian coordinates ";
      res.msg += "of the robot.";
      return false;
    }
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}


// Moves the robot to a given cartesian position. If we are currently in
// non-blocking mode, then we simply setup the move and let the non-blocking
// thread handle the actual moving. If we are in blocking mode, we then 
// communicate with the robot to execute the move.
bool RobotController::robot_SetCartesian(
    robot_comm::robot_SetCartesian::Request& req, 
    robot_comm::robot_SetCartesian::Response& res)
{
  /*
  ROS_INFO("CMD: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz, 
      sqrt(req.q0*req.q0 + req.qx*req.qx + req.qy*req.qy + req.qz*req.qz));
      */
  // If we are in non-blocking mode
  if (non_blocking)
  {
    // As we are changing some state variables, mutex this block to be sure
    // we don't get unpredictable actions
    pthread_mutex_lock(&nonBlockMutex);
    if (!do_nb_move)
    {
      // If no move is currently being executed, we will now be doing
      // a cartesian move.
      cart_move = true;
      cart_move_j = false;

      // Our new target is simply the target passed by the user
      curTargP[0] = req.x;
      curTargP[1] = req.y;
      curTargP[2] = req.z;
      curTargQ[0] = req.q0;
      curTargQ[1] = req.qx;
      curTargQ[2] = req.qy;
      curTargQ[3] = req.qz;

      // The last goal in the non-blocking move is simply the current position
      getCartesian(curGoalP[0], curGoalP[1], curGoalP[2],
          curGoalQ[0], curGoalQ[1], curGoalQ[2], curGoalQ[3]);

      // We are now ready to execute this non-blocking move
      do_nb_move = true;
    }
    else if (cart_move)
    {
      cart_move_j = false;
      // If we are currently doing a non-blocking cartesian move, we simply
      // need to update our current target
      curTargP[0] = req.x;
      curTargP[1] = req.y;
      curTargP[2] = req.z;
      curTargQ[0] = req.q0;
      curTargQ[1] = req.qx;
      curTargQ[2] = req.qy;
      curTargQ[3] = req.qz;
      
      // Remember that we changed our target, again to maintain concurrency
      targetChanged = true;
    }
    else
    {
      // If we are doing a non-blocking move, but it's a joint move,
      // it would be much too dangerous to switch
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Can't do a cartesian move while doing a ";
      res.msg += "non-blocking joint move!";
      return true;
    }
    pthread_mutex_unlock(&nonBlockMutex);
  }
  else
  {
    // If we are in blocking mode, simply execute the cartesian move
    if (!setCartesian(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
    {
      //res.ret = 0;  
      res.ret = errorId;
      res.msg = "ROBOT_CONTROLLER: Not able to set cartesian coordinates ";
      res.msg += "of the robot.";
      return true;
    }
  }

  res.ret = 1;
  res.ret = errorId;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}


// Queries the cartesian position of the robot
bool RobotController::robot_GetCartesian(
    robot_comm::robot_GetCartesian::Request& req, 
    robot_comm::robot_GetCartesian::Response& res)
{
  // Simply call our internal method to communicate with the robot and get
  // the cartesian position
  if (getCartesian(res.x, res.y, res.z, res.q0, res.qx, res.qy, res.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get cartesian coordinates of ";
    res.msg += "the robot.";
    return false;
  }
}


// Moves the robot to a given joint position. If we are currently in
// non-blocking mode, then we simply setup the move and let the non-blocking
// thread handle the actual moving. If we are in blocking mode, we then 
// communicate with the robot to execute the move.
bool RobotController::robot_SetJoints(
    robot_comm::robot_SetJoints::Request& req, 
    robot_comm::robot_SetJoints::Response& res)
{
  // If we are in non-blocking mode
  if (non_blocking)
  {
    // As we are changing state variables that are used in another thread,
    // be sure to mutex this if-block so we don't have any unanticipated
    // results
    pthread_mutex_lock(&nonBlockMutex);
    if (!do_nb_move)
    {
      // If we are not currently doing a move, we are now 
      // going to start a joint move
      cart_move = false;
      
      // Our new target is just the goal specified by the user
      curTargJ[0] = req.j1;
      curTargJ[1] = req.j2;
      curTargJ[2] = req.j3;
      curTargJ[3] = req.j4;
      curTargJ[4] = req.j5;
      curTargJ[5] = req.j6;

      // Our previous goal is simply the current position
      getJoints(curGoalJ[0], curGoalJ[1], curGoalJ[2], 
          curGoalJ[3], curGoalJ[4], curGoalJ[5]);

      // Now that we have set everything up, execute the move
      do_nb_move = true;
    }
    else if (!cart_move)
    {
      // Otherwise, we are currently doing a joint move, and we need to update
      // our target position
      curTargJ[0] = req.j1;
      curTargJ[1] = req.j2;
      curTargJ[2] = req.j3;
      curTargJ[3] = req.j4;
      curTargJ[4] = req.j5;
      curTargJ[5] = req.j6;

      // Remember that we changed our position, again to make sure we don't
      // have unexpected results
      targetChanged = true;
    }
    else
    {
      // If we are in the middle of a non-blocking cartesian move, we 
      // cannot execute a joint move
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Can't do a joint move while doing a ";
      res.msg += "non-blocking cartesian move!";
      return true;
    }
    pthread_mutex_unlock(&nonBlockMutex);
  }
  else
  {
    // If we are in blocking mode, simply execute the entire joint move
    if (!setJoints(req.j1, req.j2, req.j3, req.j4, req.j5, req.j6))
    {
      //res.ret = 0;
      res.ret = errorId;
      res.msg = "ROBOT_CONTROLLER: Not able to set cartesian coordinates ";
      res.msg += "of the robot.";
      return true;
    }
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

// Query the robot for the current position of its joints
bool RobotController::robot_GetJoints(
    robot_comm::robot_GetJoints::Request& req, 
    robot_comm::robot_GetJoints::Response& res)
{
  // Simply call our internal method to get the current position of the robot
  if (getJoints(res.j1, res.j2, res.j3, res.j4, res.j5, res.j6))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get Joint coordinates of ";
    res.msg += "the robot.";
    return false;
  }
}


// Query the robot for the inverse kinematics (joint angles) given a
// cartesian position
bool RobotController::robot_GetIK(
    robot_comm::robot_GetIK::Request& req, 
    robot_comm::robot_GetIK::Response& res)
{
char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, ABBInterpreter::getIK(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz, 
        randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    ABBInterpreter::parseJoints(reply, &res.j1, &res.j2, &res.j3, &res.j4, &res.j5, &res.j6);
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get inverse kinematics!";
    return false;
  }
}


// Query the robot for the forward kinematics (cartesian position) given
// joint angles
bool RobotController::robot_GetFK(
    robot_comm::robot_GetFK::Request& req, 
    robot_comm::robot_GetFK::Response& res)
{
char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, ABBInterpreter::getFK(req.j1, req.j2, req.j3, 
        req.j4, req.j5, req.j6, randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    ABBInterpreter::parseCartesian(reply, &res.x, &res.y, &res.z, 
        &res.q0, &res.qx, &res.qy, &res.qz);
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to get forward kinematics!";
    return false;
  }
}




// If the robot is in non-blocking mode, stop the robot
bool RobotController::robot_Stop(
    robot_comm::robot_Stop::Request& req, 
    robot_comm::robot_Stop::Response& res)
{
  // If we are currently blocking, there's nothing to stop
  if(!non_blocking)
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Only able to stop the robot in ";
    res.msg += "non-blocking mode.";
    ROS_INFO("ROBOT_CONTROLLER: Stop command not sent. "
        "Only allowed in non-blocking mode.");
    return false;
  }
  else
  {
    // Otherwise, call our internal stop method
    stop_nb();
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
}

// Set the tool frame of the robot
bool RobotController::robot_SetTool(
    robot_comm::robot_SetTool::Request& req, 
    robot_comm::robot_SetTool::Response& res)
{
  // Simply call our internal method to set the tool
  if (setTool(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the tool of the robot.";
    return false;
  }
}

// Set the inertia of the tool of the robot
bool RobotController::robot_SetInertia(
    robot_comm::robot_SetInertia::Request& req, 
    robot_comm::robot_SetInertia::Response& res)
{
  // Simply call our internal method to set the tool
  if (setInertia(req.m, req.cgx, req.cgy, req.cgz, req.ix, req.iy, req.iz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the inertia of the tool of the robot.";
    return false;
  }
}

// Set the work object of the robot
bool RobotController::robot_SetWorkObject(
    robot_comm::robot_SetWorkObject::Request& req, 
    robot_comm::robot_SetWorkObject::Response& res)
{
  // Simply call our internal method to set the work object
  if(setWorkObject(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the work ";
    res.msg += "object of the robot.";
    ROS_WARN("ROBOT_NODE: %s",res.msg.c_str());
    return false;
  }
}

// Set the communication mode of our robot
bool RobotController::robot_SetComm(
    robot_comm::robot_SetComm::Request& req, 
    robot_comm::robot_SetComm::Response& res)
{

  if (!setComm(req.mode))
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Invalide communication mode ";
    res.msg += "(0-Nonblocking 1-Blocking).";
    ROS_INFO("ROBOT_CONTROLLER: SetComm command not sent. "
        "Invalid communication mode.");
    return false;
  }
  else
  {
    res.ret = 1;
    if (req.mode == NON_BLOCKING)
      res.msg = "ROBOT_CONTROLLER: Communication set to NON_BLOCKING.";
    else
      res.msg = "ROBOT_CONTROLLER: Communication set to BLOCKING.";
    return true;
  }
}



// Set the speed of our robot. Note that if we are in non-blocking mode, we 
// call a separate method which sets step sizes in addition to speed.
// Otherwise, we just call our generic setSpeed method.
bool RobotController::robot_SetSpeed(
    robot_comm::robot_SetSpeed::Request& req, 
    robot_comm::robot_SetSpeed::Response& res)
{
  if (non_blocking)
  {
    if (!setNonBlockSpeed(req.tcp, req.ori))
    {
      res.ret = 0;
      res.msg = "ROBOT_CONTROLLER: Unable to change non-blocking ";
      res.msg += "speed of the robot.";
      return false;
    }
  }
  else if (!setSpeed(req.tcp, req.ori))
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the speed of the robot.";
    return false;
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

// Set the speed of our robot. Note that if we are in non-blocking mode, we 
// call a separate method which sets step sizes in addition to speed.
// Otherwise, we just call our generic setSpeed method.
bool RobotController::robot_SetAcc(
    robot_comm::robot_SetAcc::Request& req, 
    robot_comm::robot_SetAcc::Response& res)
{
  if (!setAcc(req.acc, req.deacc))
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the speed of the robot.";
    return false;
  }

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}


// Get the current state of the robot. 
bool RobotController::robot_GetState(
    robot_comm::robot_GetState::Request& req, 
    robot_comm::robot_GetState::Response& res)
{
  res.tcp = curSpd[0];
  res.ori = curSpd[1];
  res.zone = curZone;
  res.vacuum = curVacuum;
  res.toolx = curToolP[0];
  res.tooly = curToolP[1];
  res.toolz = curToolP[2];
  res.toolq0 = curToolQ[0];
  res.toolqx = curToolQ[1];
  res.toolqy = curToolQ[2];
  res.toolqz = curToolQ[3];
  res.toolm = curToolM;
  res.toolcgx = curToolCG[0];
  res.toolcgy = curToolCG[1];
  res.toolcgz = curToolCG[2];
  res.toolix = curToolI[0];
  res.tooliy = curToolI[1];
  res.tooliz = curToolI[2];
  res.workx = curWorkP[0];
  res.worky = curWorkP[1];
  res.workz = curWorkP[2];
  res.workq0 = curWorkQ[0];
  res.workqx = curWorkQ[1];
  res.workqy = curWorkQ[2];
  res.workqz = curWorkQ[3];
  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

// Set the zone of the robot. This is the distance before the end of a motion 
// that the server will respond. This enables smooth motions.
bool RobotController::robot_SetZone(
    robot_comm::robot_SetZone::Request& req, 
    robot_comm::robot_SetZone::Response& res)
{
  // Simply call our internal setZone method
  if (setZone(req.mode))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the zone of the robot.";
    return false;
  }
}

bool RobotController::robot_SetTrackDist(
    robot_comm::robot_SetTrackDist::Request& req,
    robot_comm::robot_SetTrackDist::Response& res)
{
  if (!non_blocking)
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: SetTrackDist only applies for non-blocking.";
    return false;
  }
  else if (setTrackDist(req.pos_dist, req.ang_dist))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to change the tracking distance ";
    res.msg += "of the robot.";
    return false;
  }
}

// Execute Special Command
bool RobotController::robot_SpecialCommand(
    robot_comm::robot_SpecialCommand::Request& req, 
    robot_comm::robot_SpecialCommand::Response& res)
{
  // Simply call our internal method
  if (specialCommand(req.command, req.param1, req.param2, req.param3, req.param4, req.param5))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to execute special command.";
    return false;
  }
}

// Turn the Vacuum on or off on the robot
bool RobotController::robot_SetVacuum(
    robot_comm::robot_SetVacuum::Request& req, 
    robot_comm::robot_SetVacuum::Response& res)
{
  // Simply call our internal method
  if (setVacuum(req.vacuum))
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to toggle the vacuum state.";
    return false;
  }
}


bool RobotController::robot_SetDefaults(robot_comm::robot_SetDefaults::Request& req, 
    robot_comm::robot_SetDefaults::Response& res)
{
  if (defaultRobotConfiguration())
  {
    res.ret = 1;
    res.msg = "ROBOT_CONTROLLER: OK.";
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "ROBOT_CONTROLLER: Not able to set default configuration.";
    return false;
  }
}


bool RobotController::setTrackDist(double pos_dist, double ang_dist)
{
  pthread_mutex_lock(&nonBlockMutex);
  
  // We set the tracking distance and adapt the speed of the robot
  // in order to avoid jerkiness.
  double desiredSpeedTCP;
  double desiredSpeedORI;
  
  //TRANSLATION
  //There is a minimum and maximum tracking distance
  if(pos_dist < MINIMUM_TRACK_DIST_TRANS)
    pos_dist = MINIMUM_TRACK_DIST_TRANS;
  if(pos_dist > MAXIMUM_TRACK_DIST_TRANS)
    pos_dist =  MAXIMUM_TRACK_DIST_TRANS;
  curDist[0] = pos_dist;
  curCartStep = curDist[0]/2.0;
  desiredSpeedTCP = 3.0*pos_dist*SAFETY_FACTOR;
  
  //ROTATION
  //There is a minimum and maximum tracking distance
  if(ang_dist < MINIMUM_TRACK_DIST_ORI)
    ang_dist = MINIMUM_TRACK_DIST_ORI;
  if(ang_dist > MAXIMUM_TRACK_DIST_ORI)
    ang_dist =  MAXIMUM_TRACK_DIST_ORI;
  curDist[1] = ang_dist * DEG2RAD;
  curOrientStep = curDist[1]/2.0;
  desiredSpeedORI = 3.0*ang_dist*SAFETY_FACTOR;
  
  //JOINTS
  //There is a minimum and maximum tracking distance
  curDist[2] = ang_dist;
  curJointStep = curDist[2]/2.0;   

  //Change speed in the ABB controller
  changing_nb_speed = true;
  setSpeed(desiredSpeedTCP, desiredSpeedORI);
  changing_nb_speed = false;

  pthread_mutex_unlock(&nonBlockMutex);
  return true;
}

bool RobotController::robot_IsMoving(
    robot_comm::robot_IsMoving::Request& req, 
    robot_comm::robot_IsMoving::Response& res)
{
  res.moving = is_moving();

  res.ret = 1;
  res.msg = "ROBOT_CONTROLLER: OK.";
  return true;
}

bool RobotController::setNonBlockSpeed(double tcp, double ori)
{
  pthread_mutex_lock(&nonBlockMutex);

  double desiredSpeedTCP;
  double desiredSpeedORI;
  
  //TRANSLATION
  //There is a minimum speed
  if(tcp < MINIMUM_NB_SPEED_TCP)
    tcp = MINIMUM_NB_SPEED_TCP;
  desiredSpeedTCP = tcp;
  if(curSpd[0]<60.0)
    curDist[0] = (desiredSpeedTCP/3.0)/SAFETY_FACTOR;
  else
    curDist[0] = INFINITY_TRACK_DIST_TRANS;
  curCartStep = curDist[0]/2.0;
 
  //ROTATION
  if(ori < MINIMUM_NB_SPEED_ORI)
    ori = MINIMUM_NB_SPEED_ORI;
  desiredSpeedORI = ori;
  if(curSpd[1]<20.0)
    curDist[1] = (desiredSpeedORI/3.0)/SAFETY_FACTOR;
  else
    curDist[1] = INFINITY_TRACK_DIST_ORI;
  curOrientStep = curDist[1]/2.0;

  //JOINT
  curDist[2] = curDist[1];
  curJointStep = curOrientStep;

  //Change speed in the ABB controller
  changing_nb_speed = true;
  setSpeed(desiredSpeedTCP, desiredSpeedORI);
  changing_nb_speed = false;

  pthread_mutex_unlock(&nonBlockMutex);
  return true;
}

// TCP Pose buffer commands
bool RobotController::robot_AddBuffer(
    robot_comm::robot_AddBuffer::Request& req, 
    robot_comm::robot_AddBuffer::Response& res)
{
      if (addBuffer(req.x, req.y, req.z, req.q0, req.qx, req.qy, req.qz))
      {
            res.ret = 1;
            res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = 0;
            res.msg = "ROBOT_CONTROLLER: Not able to add TCP pose to buffer ";
            res.msg += "the robot.";
            return false;
      }
      
}

bool RobotController::robot_ExecuteBuffer(
    robot_comm::robot_ExecuteBuffer::Request& req, 
    robot_comm::robot_ExecuteBuffer::Response& res)
{
      if (executeBuffer())
      {
            res.ret = errorId;
             res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = errorId;
             res.msg = "ROBOT_CONTROLLER: Not able to execute buffered TCP pose trajectory ";
            return true;
      }
      
}

bool RobotController::robot_ClearBuffer(
    robot_comm::robot_ClearBuffer::Request& req, 
    robot_comm::robot_ClearBuffer::Response& res)
{
      if (clearBuffer())
      {
            res.ret = 1;
            res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = 0;
            res.msg = "ROBOT_CONTROLLER: Not able to clear buffered TCP pose trajectories ";
            return false;
      }
      
}

// Joint position buffer commands
bool RobotController::robot_AddJointPosBuffer(
    robot_comm::robot_AddJointPosBuffer::Request& req, 
    robot_comm::robot_AddJointPosBuffer::Response& res)
{
      if (addJointPosBuffer(req.j1, req.j2, req.j3, req.j4, req.j5, req.j6))
      {
            res.ret = 1;
            res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = 0;
            res.msg = "ROBOT_CONTROLLER: Not able to add joint position buffer ";
            res.msg += "the robot.";
            return false;
      }
      
}

bool RobotController::robot_ExecuteJointPosBuffer(
    robot_comm::robot_ExecuteJointPosBuffer::Request& req, 
    robot_comm::robot_ExecuteJointPosBuffer::Response& res)
{
      if (executeJointPosBuffer())
      {
            res.ret = errorId;
             res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = errorId;
             res.msg = "ROBOT_CONTROLLER: Not able to execute buffered joint trajectories ";
            return true;
      }
      
}

bool RobotController::robot_ClearJointPosBuffer(
    robot_comm::robot_ClearJointPosBuffer::Request& req, 
    robot_comm::robot_ClearJointPosBuffer::Response& res)
{
      if (clearJointPosBuffer())
      {
            res.ret = 1;
            res.msg = "ROBOT_CONTROLLER: OK.";
            return true;
      }
      else
      {
            res.ret = 0;
            res.msg = "ROBOT_CONTROLLER: Not able to clear buffered joint trajectories ";
            return false;
      }
      
}




//////////////////////////////////////////////////////////////////////////////
// Internal methods that let us execute certain robot functions without using 
// ROS service callbacks
//////////////////////////////////////////////////////////////////////////////

// Pings the robot
bool RobotController::ping()
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, ABBInterpreter::pingRobot(randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
    return true;
  else
    return false;
}

// Command the robot to move to a given cartesian position
bool RobotController::setCartesian(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // We will do some collision checking and sanity checking here
  // some time in the future
  
  /*
  // For debugging purposes
  ROS_INFO("ACTION: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      x, y, z, q0, qx, qy, qz, sqrt(q0*q0+qx*qx+qy*qy+qz*qz));*/

  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, ABBInterpreter::setCartesian(x, y, z, q0, qx, qy, qz, 
        randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // If this was successful, keep track of the last commanded position
    curGoalP[0] = x;
    curGoalP[1] = y;
    curGoalP[2] = z;
    curGoalQ[0] = q0;
    curGoalQ[1] = qx;
    curGoalQ[2] = qy;
    curGoalQ[3] = qz;
    return true;
  }
  else
    return false;
}

// Command the robot to move to a given cartesian position using a joint
// move
bool RobotController::setCartesianJ(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // We will do some collision checking and sanity checking here
  // some time in the future
  
  /*
  // For debugging purposes
  ROS_INFO("ACTION: %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, %1.2f, mag = %f", 
      x, y, z, q0, qx, qy, qz, sqrt(q0*q0+qx*qx+qy*qy+qz*qz));*/

  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, ABBInterpreter::setCartesianJ(x, y, z, q0, qx, qy, qz, 
        randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // If this was successful, keep track of the last commanded position
    curGoalP[0] = x;
    curGoalP[1] = y;
    curGoalP[2] = z;
    curGoalQ[0] = q0;
    curGoalQ[1] = qx;
    curGoalQ[2] = qy;
    curGoalQ[3] = qz;
    return true;
  }
  else
    return false;
}


// Query the robot for the current position of the robot
bool RobotController::getCartesian(double &x, double &y, double &z, 
    double &q0, double &qx, double &qy, double &qz)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, ABBInterpreter::getCartesian(randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // Parse the reply to get the cartesian coordinates
    ABBInterpreter::parseCartesian(reply, &x, &y, &z, &q0, &qx, &qy, &qz);
    return true;
  }
  else 
    return false;
}

// Command the robot to move to a given joint configuration
bool RobotController::setJoints(double j1, double j2, double j3, double j4,
    double j5, double j6)
{
  // We will do some collision and sanity checks here

  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

  strcpy(message, ABBInterpreter::setJoints(j1, j2, j3, j4, j5, j6, 
        randNumber).c_str());

  if (sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // If the move was successful, keep track of the last commanded position
    curGoalJ[0] = j1;
    curGoalJ[1] = j2;
    curGoalJ[2] = j3;
    curGoalJ[3] = j4;
    curGoalJ[4] = j5;
    curGoalJ[5] = j6;
    return true;
  }
  else
    return false;
}

// Query the robot for the current joint positions
bool RobotController::getJoints(double &j1, double &j2, double &j3,
    double &j4, double &j5, double &j6)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, ABBInterpreter::getJoints(randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
  {
    // Parse the reply to get the joint angles
    ABBInterpreter::parseJoints(reply, &j1, &j2, &j3, &j4, &j5, &j6);
    return true;
  }
  else
    return false;
}

// Stop the robot while it is in non_blocking mode
bool RobotController::stop_nb()
{
  // This function doesn't apply when we are in blocking mode
  if (!non_blocking)
    return false;

  // If the robot is currently not moving, we're done.
  if (!do_nb_move)
    return true;

  ros::Rate stop_check(STOP_CHECK_FREQ);

  // If we are currently non-blocking, get out of non-blocking move and wait
  // to return until we're sure that the robot has stopped moving.
  do_nb_move = false;

  // We wait for the non-blocking thread to set stopConfirm to 
  // true after we set stopRequest to true
  stopConfirm = false;
  stopRequest = true;
  while (!stopConfirm)
  {
    stop_check.sleep();
  }
  stopRequest = false; 

  return true;
}

// Set the tool frame of the robot
bool RobotController::setTool(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // Only take action if the required values are different than the actual ones
  if(x!=curToolP[0] || y!=curToolP[1] || y!=curToolP[2] || 
     q0!=curToolQ[0] || qx!=curToolQ[1] || qy!=curToolQ[2] || qz!=curToolQ[3])
    {
      // This is dangerous if we are currently executing a non-blocking move
      if (do_nb_move)
      return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::setTool(x, y, z, q0, qx, qy, qz, 
                                    randNumber).c_str());

      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // If the command was successful, remember our new tool frame
        curToolP[0] = x;
        curToolP[1] = y;
        curToolP[2] = z;
        curToolQ[0] = q0;
        curToolQ[1] = qx;
        curToolQ[2] = qy;
        curToolQ[3] = qz;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/toolX",x);
        node->setParam(robotname_sl + "/toolY",y);
        node->setParam(robotname_sl + "/toolZ",z);
        node->setParam(robotname_sl + "/toolQ0",q0);
        node->setParam(robotname_sl + "/toolQX",qx);
        node->setParam(robotname_sl + "/toolQY",qy);
        node->setParam(robotname_sl + "/toolQZ",qz);
        return true;
      }
      else
      return false;
    }
  return true;
}

// Set the tool frame of the robot
bool RobotController::setInertia(double m, double cgx, double cgy, 
    double cgz, double ix, double iy, double iz)
{
  // Only take action if the required values are different than the actual ones
  if(m!=curToolM || cgx!=curToolCG[0] || cgy!=curToolCG[1] || cgz!=curToolCG[2] ||
     ix!=curToolI[0] || iy!=curToolI[1] || iz!=curToolI[2])
    {
      // This is dangerous if we are currently executing a non-blocking move
      if (do_nb_move)
      return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::setInertia(m, cgx, cgy, cgz, ix, iy, iz, 
                                    randNumber).c_str());

      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // If the command was successful, remember our new inertia
        curToolM = m;
        curToolCG[0] = cgx;
        curToolCG[1] = cgy;
        curToolCG[2] = cgz;
        curToolI[0] = ix;
        curToolI[1] = iy;
        curToolI[2] = iz;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/toolMass",m);
        node->setParam(robotname_sl + "/toolCGX",cgx);
        node->setParam(robotname_sl + "/toolCGY",cgy);
        node->setParam(robotname_sl + "/toolCGZ",cgz);
        node->setParam(robotname_sl + "/toolIX",ix);
        node->setParam(robotname_sl + "/toolIY",iy);
        node->setParam(robotname_sl + "/toolIZ",iz);
        return true;
      }
      else
      return false;
    }
  return true;
}

// Set the work object frame of the robot
bool RobotController::setWorkObject(double x, double y, double z, 
    double q0, double qx, double qy, double qz)
{
  // Only take action if the required values are different than the actual ones
  if(x!=curWorkP[0] || y!=curWorkP[1] || y!=curWorkP[2] || 
     q0!=curWorkQ[0] || qx!=curWorkQ[1] || qy!=curWorkQ[2] || qz!=curWorkQ[3])
    {
      // This is dangerous if we are in the middle of a non-blocking move
      if (do_nb_move)
      return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::setWorkObject(x, y, z, q0, qx, qy, qz, 
                                        randNumber).c_str());
      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // If the command was successful, remember our new work object
        curWorkP[0] = x;
        curWorkP[1] = y;
        curWorkP[2] = z;
        curWorkQ[0] = q0;
        curWorkQ[1] = qx;
        curWorkQ[2] = qy;
        curWorkQ[3] = qz;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/workobjectX",x);
        node->setParam(robotname_sl + "/workobjectY",y);
        node->setParam(robotname_sl + "/workobjectZ",z);
        node->setParam(robotname_sl + "/workobjectQ0",q0);
        node->setParam(robotname_sl + "/workobjectQX",qx);
        node->setParam(robotname_sl + "/workobjectQY",qy);
        node->setParam(robotname_sl + "/workobjectQZ",qz);
        return true;
      }
      else
      return false;
    }
  return true;
}

// Set the speed of the robot
bool RobotController::setSpeed(double tcp, double ori)
{
  // Only take action if the required values are different than the actual ones
  if(tcp!=curSpd[0] || ori!=curSpd[1])
    {
      // This is dangerous if we are currently executing a non-blocking move
      // (unless we're sure and set changing_nb_speed to true)
      if (!changing_nb_speed && do_nb_move)
      return false;

      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::setSpeed(tcp, ori, 
                                     randNumber).c_str());
      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // If we successfully changed the speed, remember our new speed values
        curSpd[0] = tcp;
        curSpd[1] = ori;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/speedTCP",tcp);
        node->setParam(robotname_sl + "/speedORI",ori);
        return true;
      }
      else
      return false;
    }
  return true;
}

// Set the acceleration of the robot
bool RobotController::setAcc(double acc, double deacc)
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::setAcc(acc, deacc, 
                                     randNumber).c_str());
      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        return true;
      }
      else
      return false;
}


// Set the zone of our robot
bool RobotController::setZone(int z)
{
 // Only take action if the required values are different than the actual ones
  if(z!=curZone)
    {
      // This is dangerous if we are in the middle of a non-blocking move
      if (do_nb_move)
      return false;
      
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      
      // Make sure the specified zone number exists
      if (z < 0 || z > NUM_ZONES)
      {
        ROS_INFO("ROBOT_CONTROLLER: SetZone command not sent. Invalide zone mode.");
        return false;
      }
      
      strcpy(message, ABBInterpreter::setZone((z == ZONE_FINE), 
                                    zone_data[z].p_tcp, zone_data[z].p_ori, zone_data[z].ori, 
                                    randNumber).c_str());
      
      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // If we set the zone successfully, remember our new zone
        curZone = z;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/zone",z);
        return true;
      }
      else
      return false;
    }
  return true;
}

// Set the communication of our robot
bool RobotController::setComm(int mode)
{
  if (mode == NON_BLOCKING)
  {
    // If the user wants to set non-blocking mode, we 
    // simply set our state variables appropriately
    if (!non_blocking)
    {
      // our mode is now non-blocking, but we are not yet executing a motion
      do_nb_move = false;
      non_blocking = true;

      // Set up step sizes based on the current speed
      setNonBlockSpeed(curSpd[0], curSpd[1]);
    }
    return true;
  }
  else if (mode == BLOCKING)
  {
    // If the user wants to set blocking mode, make sure we stop any 
    // non-blocking move, and then set our state variable appropriately.
    if (non_blocking)
    {
      stop_nb();
      non_blocking = false;
    }
    return true;
  }
  else
  {
    return false;
  }
}


// Call Special Command
bool RobotController::specialCommand(int command, double param1, double param2, double param3, double param4, double param5)
{
  char message[MAX_BUFFER];
  char reply[MAX_BUFFER];
  int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
  strcpy(message, ABBInterpreter::specialCommand(command, param1, param2, param3, param4, param5, randNumber).c_str());

  if(sendAndReceive(message, strlen(message), reply, randNumber))
    return true;
  else
    return false;
}

// Open or close the vacuum 0 is open, 1 is closed
bool RobotController::setVacuum(int v)
{
  // Only take action if the required values are different than the actual ones
  if(v!=curVacuum)
    {
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      
      // Make sure we are either opening or closing the vacuum
      if((v != VACUUM_OPEN) && (v != VACUUM_CLOSE))
      {
        ROS_INFO("ROBOT_CONTROLLER: SetVacuum command not sent. "
               "Invalid communication mode.");
        return false;
      }
      
      strcpy(message, ABBInterpreter::setVacuum(v, randNumber).c_str());
      
      if(sendAndReceive(message, strlen(message), reply, randNumber))
      {
        // Remember the current state of the vacuum if the command sent
        curVacuum = v;
        // We also save it to the parameter server
        node->setParam(robotname_sl + "/vacuum",v);
        return true;
      }
      else
      return false;
    }
  return true;
}

// If we are in non-blocking mode, then we have a variable that keeps 
//  track of whether we are moving or not. If we are in blocking mode,
//  reaching this function means the robot is not moving
bool RobotController::is_moving()
{
  if (non_blocking)
    return do_nb_move;
  else
    return false;
}

// Send TCP pose trajectories to a buffer 1 at a time
bool RobotController::addBuffer(double x, double y, double z, double q0, double qx, double qy, double qz)
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::addBuffer(x, y, z, q0, qx, qy, qz, randNumber).c_str());
      if(sendAndReceive(message,strlen(message), reply, randNumber))
        return true;
    else
      return false;
}

// Execute the TCP poses added to the buffer previously
bool RobotController::executeBuffer()
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::executeBuffer(randNumber).c_str());
      if(sendAndReceive(message,strlen(message), reply, randNumber))
      return true;
    else
      return false;
}

// Clear the TCP pose buffer
bool RobotController::clearBuffer()
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::clearBuffer(randNumber).c_str());
      if(sendAndReceive(message,strlen(message), reply, randNumber))
      return true;
    else
      return false;
}

// Set adds joint positions 1 configuration at a time to the joint position buffer
bool RobotController::addJointPosBuffer(double j1, double j2, double j3, double j4, double j5, double j6)
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::addJointPosBuffer(j1, j2, j3, j4, j5, j6, randNumber).c_str());
      if(sendAndReceive(message,strlen(message), reply, randNumber))
        return true;
    else
      return false;
      
      /* Template
      // Command the robot to move to a given joint configuration
    bool RobotController::setJoints(double j1, double j2, double j3, double j4,
    double j5, double j6)
      {
    // We will do some collision and sanity checks here

    char message[MAX_BUFFER];
    char reply[MAX_BUFFER];
    int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));

    strcpy(message, ABBInterpreter::setJoints(j1, j2, j3, j4, j5, j6, 
        randNumber).c_str());

    if (sendAndReceive(message, strlen(message), reply, randNumber))
    {
    // If the move was successful, keep track of the last commanded position
    curGoalJ[0] = j1;
    curGoalJ[1] = j2;
    curGoalJ[2] = j3;
    curGoalJ[3] = j4;
    curGoalJ[4] = j5;
    curGoalJ[5] = j6;
    return true;
    }
    else
    return false;
    }      
      */
      
}

// Execute the joint configurations added to the buffer previously
bool RobotController::executeJointPosBuffer()
{
      char message[MAX_BUFFER];
      char reply[MAX_BUFFER];
      int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
      strcpy(message, ABBInterpreter::executeJointPosBuffer(randNumber).c_str());
      if(sendAndReceive(message,strlen(message), reply, randNumber))
      return true;
    else
      return false;
}

// Clear the joint position buffer
bool RobotController::clearJointPosBuffer()
{
    char message[MAX_BUFFER];
    char reply[MAX_BUFFER];
    int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
    strcpy(message, ABBInterpreter::clearJointPosBuffer(randNumber).c_str());
    if(sendAndReceive(message,strlen(message), reply, randNumber))
      return true;
    else
      return false;
}

//////////////////////////////////////////////////////////////////////////////
// Connect to Servers on Robot
//////////////////////////////////////////////////////////////////////////////


// Establish RRI connection
bool RobotController::establishRRI(int port){
  if(RRIConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: RRI already connected.");
    return false;
  }
  
  try {
    // Create an RRI UDP socket
    RRIsock = new UDPSocket(port);
    
    char message[MAX_BUFFER];
    char reply[MAX_BUFFER];
    int randNumber = (int)(ID_CODE_MAX*(double)rand()/(double)(RAND_MAX));
    strcpy(message, ABBInterpreter::connectRRI(randNumber).c_str());
    if(sendAndReceive(message,strlen(message), reply, randNumber)){
      RRIConnected = true;
      return true;
    }
    else
      return false;
  } catch (SocketException &e) {
    ROS_INFO("ROBOT_CONTROLLER: %s", e.what());
    return false;
  }
}

// Connect to the logger server
bool RobotController::connectLoggerServer(const char* ip, int port)
{
  if(loggerConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: Logger already connected.");
    return false;
  }

  // First, create a logger socket
  if ((robotLoggerSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
  {
    ROS_INFO("ROBOT_CONTROLLER: Problem creating the logger socket. "
        "Error number: %d.",errno);
  }
  else
  {
    // Try connecting to the logger server
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
    if (connect(robotLoggerSocket, (sockaddr*)&remoteSocket,
          sizeof(remoteSocket)) == -1)
    {
      ROS_INFO("ROBOT_CONTROLLER: Could not connect to the ABB logger "
          "server. Error number: %d.",errno);
    }
    else
    {
      //Set socket as non-blocking
      int flags;
      flags = fcntl(robotLoggerSocket,F_GETFL,0);
      if(flags != -1)
      {
        if(fcntl(robotLoggerSocket, F_SETFL, flags | O_NONBLOCK) != -1)
        {
          loggerConnected = true;
          return true;
        }
        else
        {
          ROS_INFO("ROBOT_CONTROLLER: Could not set the logger socket "
              "as non-blocking");
        }
      }
      else
      {
        ROS_INFO("ROBOT_CONTROLLER: Could not set the logger socket "
            "as non-blocking");
      }
    }
  }
  return false;
}

// Connect to the robot motion server
bool RobotController::connectMotionServer(const char* ip, int port)
{
  if(motionConnected)
  {
    ROS_INFO("ROBOT_CONTROLLER: Robot controller already connected.");
    return false;
  }

  // Create a socket for robot motion
  if ((robotMotionSocket = socket(PF_INET, SOCK_STREAM, 0)) == -1)
    ROS_INFO("ROBOT_CONTROLLER: Problem creating the socket. %d",errno);
  else
  {
    // Now try to connect to the robot motion server
    struct sockaddr_in remoteSocket;
    remoteSocket.sin_family = AF_INET;
    remoteSocket.sin_port = htons(port);
    inet_pton(AF_INET, ip, &remoteSocket.sin_addr.s_addr);
    if(connect(robotMotionSocket, (sockaddr*)&remoteSocket,
          sizeof(remoteSocket)) == -1)
    {
      ROS_INFO("ROBOT_CONTROLLER: Could not connect to the "
          "ABB robot. Error number %d.",errno);
    }
    else
    {
      motionConnected = true;
      return true;
    }
  }
  return false;
}

// Helper function to send a command to the robot, 
// wait for an answer and check for correctness.
bool RobotController::sendAndReceive(char *message, int messageLength, 
    char* reply, int idCode)
{
  pthread_mutex_lock(&sendRecvMutex);
  if (send(robotMotionSocket, message, messageLength, 0) == -1)
  {
    ROS_WARN("ROBOT_CONTROLLER: Failed to send command to ABB robot:"
        " Error number %d.", errno);
  }
  else
  {
    // Read the reply to the message we just sent, and make sure
    // it's not corrupt, and the command was executed successfully
    int t;
    if ((t=recv(robotMotionSocket, reply, MAX_BUFFER-1, 0)) > 0)
    {
      reply[t] = '\0';
      sprintf(errorReply, "%s",reply);
      int ok, rcvIdCode;
      sscanf(reply,"%*d %d %d", &rcvIdCode, &ok);
      errorId = ok;
      //printf("Error Id %d\n",errorId);
 
      if(idCode!=-1)
      {  
        if ((ok == SERVER_OK) && (rcvIdCode == idCode))
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return true;
        }
        else if ((ok == SERVER_COLLISION) && (rcvIdCode == idCode))
        {
          ROS_WARN("ROBOT_CONTROLLER: Collision Detected.");
          pthread_mutex_unlock(&sendRecvMutex);
          return false;
        }
        else if ((ok == SERVER_BAD_IK || ok == SERVER_BAD_FK) && (rcvIdCode == idCode))
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return false;
        }
        else
        {
          ROS_WARN("ROBOT_CONTROLLER: Corrupt message.");
          ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
        }
      }
      else
      {
        if (ok == SERVER_OK)
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return true;
        }
        else if (ok == SERVER_COLLISION)
        {
          ROS_WARN("ROBOT_CONTROLLER: Collision Detected.");      
          pthread_mutex_unlock(&sendRecvMutex);
          return false;
  
        }
        else if ((ok == SERVER_BAD_IK || ok == SERVER_BAD_FK) && (rcvIdCode == idCode))
        {
          pthread_mutex_unlock(&sendRecvMutex);
          return false;
        }
        else
        {
          ROS_WARN("ROBOT_CONTROLLER: Corrupt message.");
          ROS_WARN("msg = %s, reply = %s, idCode = %d, ok = %d, rcvCode = %d", message, reply, idCode, ok, rcvIdCode);
        }
      }
    }
    else
      ROS_WARN("ROBOT_CONTROLLER: Failed to receive answer from ABB robot.");
  }

  pthread_mutex_unlock(&sendRecvMutex);
  return false;
}

// Compute the distance between the current position and the goal
double RobotController::posDistFromGoal()
{
  pthread_mutex_lock(&cartUpdateMutex);
  double pos_mag = (curGoalP - curP).norm();
  pthread_mutex_unlock(&cartUpdateMutex);
  return pos_mag;
}

// Compute the distance between the current orientation and the goal
double RobotController::orientDistFromGoal()
{
  pthread_mutex_lock(&cartUpdateMutex);
  //Quaternion diff = Quaternion("1 0 0 0") - (curGoalQ - curQ);
  //diff /= diff.norm();
  Quaternion diff = curGoalQ^(curQ.inverse());
  pthread_mutex_unlock(&cartUpdateMutex);
  double ang_mag = fabs(diff.getAngle());
  return ang_mag;
}

// Compute the distance between the current joint angles and the goal angles
double RobotController::jointDistFromGoal()
{
  double joint_mag = 0;
  pthread_mutex_lock(&jointUpdateMutex);
  for (int i=0; i < 6; i++)
    joint_mag += (curGoalJ[i] - curJ[i]) * (curGoalJ[i] - curJ[i]);
  pthread_mutex_unlock(&jointUpdateMutex);

  return sqrt(joint_mag);
}


//////////////////////////////////////////////////////////////////////////////
// Log Call Back
//
// This function communicates with the robot to see if any new position
// or force information has been transmitted by tcp/ip. If any data has been
// sent, it reads all of it, saves the newest of each message, and publishes
// any new position, joint, or force information it gets. This function is
// called every time there is a timer event.
//////////////////////////////////////////////////////////////////////////////
void RobotController::logCallback(const ros::TimerEvent&)
{
  int t;
  int code;
  char buffer[MAX_BUFFER];
  char * partialBuffer;
  bool cartesianModif = false;
  bool jointsModif = false;
  bool forceModif = false;
  robot_comm::robot_JointsLog msgJoints;
  robot_comm::robot_CartesianLog msgCartesian;
  robot_comm::robot_ForceLog msgForce;

  // Read all information from the tcp/ip socket
  if ((t=recv(robotLoggerSocket, buffer, MAX_BUFFER-1, 0)) > 0)
  {
    // Add an end character to form our string
    buffer[t] = '\0';
    partialBuffer = buffer;

    // Each message starts with a '#' character. Read messages one at a time
    while((partialBuffer = strchr(partialBuffer,'#'))!=NULL)            
    {
      // The number after the start character is the type of message
      sscanf(partialBuffer,"# %d", &code);
      switch(code)
      {
        // Cartesian Message
        case 0:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %lf %lf %lf %lf %lf %lf %lf %lf",
                date,
                time,
                &msgCartesian.timeStamp,
                &msgCartesian.x,
                &msgCartesian.y,
                &msgCartesian.z,
                &msgCartesian.q0,
                &msgCartesian.qx,
                &msgCartesian.qy,
                &msgCartesian.qz);
            if(nParams == 10)
            {
              // If we read in the correct number of parameters, save this message
              msgCartesian.date.assign(date);
              msgCartesian.time.assign(time);
              msgCartesian.timeStamp = ros::Time::now().toSec();
              cartesianModif = true;
            }
            break;
          }

        // Joint Message
        case 1:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %lf %lf %lf %lf %lf %lf %lf",
                date,
                time,
                &msgJoints.timeStamp,
                &msgJoints.j1,
                &msgJoints.j2,
                &msgJoints.j3,
                &msgJoints.j4,
                &msgJoints.j5,
                &msgJoints.j6);
            if (nParams == 9)
            {
              // If we read in the correct number of parameters, save this message
              msgJoints.date.assign(date);
              msgJoints.time.assign(time);
              jointsModif = true;
            }
            break;
          }

        // Force Message
        case 2:
          {
            char date[MAX_BUFFER];
            char time[MAX_BUFFER];
            int nParams = sscanf(partialBuffer,"# %*d %s %s %lf %lf %lf %lf %lf %lf %lf",
                date,
                time,
                &msgForce.timeStamp,
                &msgForce.fx,
                &msgForce.fy,
                &msgForce.fz,
                &msgForce.tx,
                &msgForce.ty,
                &msgForce.tz);
            if (nParams == 9)
            {
              // If we read in the correct number of parameters, save this message.
              msgForce.date.assign(date);
              msgForce.time.assign(time);
              forceModif = true;
            }
            break;
          }
      }
      // Increment partialBuffer, so we don't look at the same message again
      partialBuffer++;
    }

    // Now, only publish on a given topic if a new message was received.
    // Also, save this data internally so it can be used for other functions
    if(cartesianModif)
    {
      handle_robot_CartesianLog.publish(msgCartesian);
      pthread_mutex_lock(&cartUpdateMutex);
      curP[0] = msgCartesian.x;
      curP[1] = msgCartesian.y;
      curP[2] = msgCartesian.z;
      curQ[0] = msgCartesian.q0;
      curQ[1] = msgCartesian.qx;
      curQ[2] = msgCartesian.qy;
      curQ[3] = msgCartesian.qz;
      pthread_mutex_unlock(&cartUpdateMutex); 
    }
    if(jointsModif)
    {
      handle_robot_JointsLog.publish(msgJoints);
      pthread_mutex_lock(&jointUpdateMutex);
      curJ[0] = msgJoints.j1;
      curJ[1] = msgJoints.j2;
      curJ[2] = msgJoints.j3;
      curJ[3] = msgJoints.j4;
      curJ[4] = msgJoints.j5;
      curJ[5] = msgJoints.j6;
      pthread_mutex_unlock(&jointUpdateMutex);
    }
    if(forceModif)
    {
      handle_robot_ForceLog.publish(msgForce);
      pthread_mutex_lock(&forceUpdateMutex);
      curForce[0] = msgForce.fx;
      curForce[1] = msgForce.fy;
      curForce[2] = msgForce.fz;
      curForce[3] = msgForce.tx;
      curForce[4] = msgForce.ty;
      curForce[5] = msgForce.tz;
      pthread_mutex_unlock(&forceUpdateMutex);
    }
    
    //prepare joint state message
    if(jointsModif){
      sensor_msgs::JointState js;
      js.position.push_back(msgJoints.j1*DEG2RAD);
      js.position.push_back(msgJoints.j2*DEG2RAD);
      js.position.push_back(msgJoints.j3*DEG2RAD);
      js.position.push_back(msgJoints.j4*DEG2RAD);
      js.position.push_back(msgJoints.j5*DEG2RAD);
      js.position.push_back(msgJoints.j6*DEG2RAD);
        
      js.name.push_back("joint1");
      js.name.push_back("joint2");
      js.name.push_back("joint3");
      js.name.push_back("joint4");
      js.name.push_back("joint5");
      js.name.push_back("joint6");
      js.header.stamp = ros::Time::now();
      handle_robot_RosJointState.publish(js);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// RRI Call Back
//
// This function receives robot position data to see if any new position
// or force information has been transmitted by udp. If any data has been
// sent, it reads all of it, saves the newest of each message, and publishes
// any new position, joint, or force information it gets. This function is
// called every time there is a timer event.
//////////////////////////////////////////////////////////////////////////////


void RobotController::rriCallback(const ros::TimerEvent&)
{
  char buffer[MAX_BUFFER];
  int recvMsgSize;
  string sourceAddress;             // Address of datagram source
  unsigned short sourcePort;        // Port of datagram source
  
  // Read all information from the tcp/ip socket
  
  try{
    if ((recvMsgSize = RRIsock->recvFrom(buffer, MAX_BUFFER-1, sourceAddress, sourcePort)) > 0)
    {
      // Prepare the msgs
      sensor_msgs::JointState pos;
      pos.header.stamp = ros::Time::now();
      sensor_msgs::JointState js;
      js.header.stamp = pos.header.stamp;
      
      // Parse it
      buffer[recvMsgSize] = '\0';
      xmldoc.Clear();
      xmldoc.Parse( buffer );
      TiXmlElement* element = xmldoc.FirstChildElement( "RobData" ); 
      if (element){
        
        // For TCP position 
        TiXmlElement* Pact = element->FirstChildElement("P_act");
        double P_act[6];  // X, Y, Z, Rx, Ry, Rz
        string P_element_names[] = {"X", "Y", "Z", "Rx", "Ry", "Rz"};
        
        for (int i=0;i<6;i++)
          Pact->QueryDoubleAttribute( P_element_names[i].c_str(), &P_act[i]);
        
        // publish it
        for (int i=0;i<6;i++){
          if (i<3)  pos.position.push_back(P_act[i]);
          else      pos.position.push_back(P_act[i]);
          pos.name.push_back(P_element_names[i]);
        }
        handle_robot_RRICartState.publish(pos);
          
        // For joints position
        TiXmlElement* Jact = element->FirstChildElement("J_act");
        double J_act[6];  // J1, J2, J3, J4, J5, J6
        string J_element_names[] = {"J1", "J2", "J3", "J4", "J5", "J6"};
        for (int i=0;i<6;i++)
          Jact->QueryDoubleAttribute( J_element_names[i].c_str(), &J_act[i]);
          
        // publish it
        for (int i=0;i<6;i++)
          js.position.push_back(J_act[i]);
          
        js.name.push_back("joint1");
        js.name.push_back("joint2");
        js.name.push_back("joint3");
        js.name.push_back("joint4");
        js.name.push_back("joint5");
        js.name.push_back("joint6");
        handle_robot_RRIJointState.publish(js);
      }
    }
  } catch (SocketException &e) {
    // no data
    //cerr << e.what() << endl;
  }
}

//////////////////////////////////////////////////////////////////////////////
// Main Thread for Logger
//
// This is the main function for our logger thread. We simply start a ROS 
// timer event and get it to call our loggerCallback function at a 
// specified interval. This exits when ROS shuts down.
//////////////////////////////////////////////////////////////////////////////
void *loggerMain(void *args)
{
  //Recover the pointer to the main node
  RobotController* ABBrobot;
  ABBrobot = (RobotController*) args;

  // Create a timer to look at the log data
  ros::Timer loggerTimer;
  loggerTimer = ABBrobot->node->createTimer(ros::Duration(0.003), 
      &RobotController::logCallback, ABBrobot);
  // Now simply wait until the program is shut down
  ros::waitForShutdown();
  loggerTimer.stop();

  pthread_exit((void*) 0);
}

//////////////////////////////////////////////////////////////////////////////
// Main Thread for RRI
//
// This is the main function for our logger thread. We simply start a ROS 
// timer event and get it to call our loggerCallback function at a 
// specified interval. This exits when ROS shuts down.
//////////////////////////////////////////////////////////////////////////////
void *rriMain(void *args)
{
  //Recover the pointer to the main node
  RobotController* ABBrobot;
  ABBrobot = (RobotController*) args;

  // Create a timer to look at the log data
  ros::Timer rriTimer;
  rriTimer = ABBrobot->node->createTimer(ros::Duration(0.001), 
      &RobotController::rriCallback, ABBrobot);

  // Now simply wait until the program is shut down
  ros::waitForShutdown();
  rriTimer.stop();

  pthread_exit((void*) 0);
}


//////////////////////////////////////////////////////////////////////////////
// Main Thread for Non-Blocking
//
// This is the main function of the non-blocking thread. When we are in
// non-blocking mode, it updates it's current target position, figures
// out what step to take, and moves there. 
//////////////////////////////////////////////////////////////////////////////
void *nonBlockMain(void *args)
{
  // Recover the pointer to the main node
  RobotController* robot;
  robot = (RobotController*) args;

  // Initiate our loop frequency variable
  ros::Rate nb_rate(NB_FREQ);

  // We will stay in this loop until ROS shuts down
  while(ros::ok())
  {
    // While we are inside this loop, the robot is moving
    while (robot->non_blocking && robot->do_nb_move)
    {
      // If we are doing a cartesian move, execute the sequence below
      if (robot->cart_move)
      {
        // Read in the current cartesian target, and don't use it again
        // for this iteration in case it changed in between
        pthread_mutex_lock (&nonBlockMutex);
        HomogTransf target(robot->curTargQ.getRotMat(), robot->curTargP);
        robot->targetChanged = false;
        /*ROS_INFO("Target: %f, %f, %f", robot->curTargP[0], 
            robot->curTargP[1], robot->curTargP[2]);*/
        pthread_mutex_unlock(&nonBlockMutex);

        // Get the last goal position, and find the 
        // difference between here and our target
        HomogTransf pos(robot->curGoalQ.getRotMat(), robot->curGoalP);
        HomogTransf diff = (pos.inv())*target;
        
        // Get the orientation and translational change
        Vec diffV = diff.getTranslation();
        Quaternion diffQ = diff.getRotation().getQuaternion();
        diffQ /= diffQ.norm();

        // Compute the magnitude of each change
        double transMag = diffV.norm();
        double rotMag = diffQ.getAngle();

        // Compute the number of loops it will take us to reach our 
        // translation and orientation goals. Note that since our speed and
        // step size is variable, we make sure that nothing funny happens while
        pthread_mutex_lock(&nonBlockMutex);
        double linSteps = transMag / robot->curCartStep;
        double angSteps = rotMag / robot->curOrientStep;
        pthread_mutex_unlock(&nonBlockMutex);

        // This will hold the distance to translate 
        // and rotate for this iteration
        double transDist = 0;
        double angDist = 0;

        // Keep track of whether this is our last step
        bool reachedGoal = false;

        // If we have more linear steps than angular steps left, make sure 
        // we go the full distance for the linear step, but only a scaled 
        // distance for the angular step
        if (linSteps >= angSteps)
        {
          // If we have more than a step left, make sure we only move 
          // 1 step's worth in both rotation and translation
          if (linSteps > 1.0)
          {
            transDist = transMag / linSteps;
            angDist = rotMag / linSteps;
          }
          else
          {
            // Otherwise, we are less than a step away from our goal
            reachedGoal = true;
          }
        }
        else
        {
          // If there are more angular steps than linear steps remaining, 
          // make sure we scale everything by the number of angular 
          // steps left
          if (angSteps > 1.0)
          {
            // If we have more than a step left, only move 1 steps's worth
            angDist = rotMag / angSteps;
            transDist = transMag / angSteps;
          }
          else
          {
            // Otherwise, we are less than a step away from our goal
            reachedGoal = true;
          }
        }

        // Now that we have computed the magnitude of our steps, compute
        // the actual translation and rotation to do for this step
        Vec incTrans(3);
        Quaternion incRot("1 0 0 0");

        if (!reachedGoal)
        {
          // Simply scale the total difference by the current magnitude's step
          // to get the translation for this step
          if (transMag > 0)
            incTrans = diffV * transDist / transMag;

          // Interpolate between not rotating (unit quaternion) to the full
          // rotation, and scale by the magnitude for the current step
          if (rotMag > 0)
          {
            incRot = Quaternion("1 0 0 0") + 
              (diffQ - Quaternion("1 0 0 0")) * angDist / rotMag;

            // Make sure that we renormalize our quaternion
            incRot /= incRot.norm();
          }
        }
        else
        {
          // If we have less than a step to go, the translation and rotation 
          // is simply the changed we calculated above
          incTrans = diffV;
          incRot = diffQ;
        }

        // Now form homogeneous matrices to calculate the resulting 
        // position and orientation from this step
        HomogTransf incStep(incRot.getRotMat(), incTrans);
        HomogTransf newGoal = pos * incStep;

        Vec newGoalV = newGoal.getTranslation();
        Quaternion newGoalQ = newGoal.getRotation().getQuaternion();

        // Wait here until we're within range to last commanded goal
        ros::Rate check_rate(DIST_CHECK_FREQ);

        while ((robot->posDistFromGoal() > robot->curDist[0]) ||
            (robot->orientDistFromGoal() > robot->curDist[1]))
          check_rate.sleep();

        /*
           ROS_INFO("New Goal: %f, %f, %f", newGoalV[0], newGoalV[1], newGoalV[2]);
           ROS_INFO("Old Goal: %f, %f, %f", robot->curGoalP[0], 
           robot->curGoalP[1], robot->curGoalP[2]);
           pthread_mutex_lock (&cartUpdateMutex);
           ROS_INFO("Cur Pos: %f, %f, %f", 
           robot->curP[0], robot->curP[1], robot->curP[2]); 
           pthread_mutex_unlock(&cartUpdateMutex);
         */

        if (robot->cart_move_j)
        {
          // Now do the cartesian move towards our new goal
          if (!robot->setCartesianJ(newGoalV[0], newGoalV[1], newGoalV[2],
                newGoalQ[0], newGoalQ[1], newGoalQ[2], newGoalQ[3]))
          {
            ROS_INFO("Non-Blocking move error!");
            pthread_mutex_lock (&nonBlockMutex);
            robot->do_nb_move = false;
            pthread_mutex_unlock(&nonBlockMutex);
          }
        }
        else
        {
          // Now do the cartesian move towards our new goal
          if (!robot->setCartesian(newGoalV[0], newGoalV[1], newGoalV[2],
                newGoalQ[0], newGoalQ[1], newGoalQ[2], newGoalQ[3]))
          {
            ROS_INFO("Non-Blocking move error!");
            pthread_mutex_lock (&nonBlockMutex);
            robot->do_nb_move = false;
            pthread_mutex_unlock(&nonBlockMutex);
          }
        }

        // If we have reached our goal, and the target hasn't been changed 
        // while we were doing the last move, we're done.
        pthread_mutex_lock (&nonBlockMutex);
        if (reachedGoal && !robot->targetChanged)
          robot->do_nb_move = false;
        pthread_mutex_unlock(&nonBlockMutex);
      }
      // Otherwise, we are executing a non-blocking joint move
      else
      {
        int i;
        double diffJ[NUM_JOINTS];
        double newGoalJ[NUM_JOINTS];
        bool reachedJ = false;
        double maxNumSteps = 0.0;
        double jointStepSize;

        // Compute the difference between the current goal and joint target 
        // for each joint. Note that this is the only time we access the 
        // current joint target, as it's possible that it could change 
        // while we are executing this iteration.
        pthread_mutex_lock (&nonBlockMutex);
        for (i=0; i<NUM_JOINTS; i++)
        {
          diffJ[i] = robot->curTargJ[i] - robot->curGoalJ[i];
        }
        robot->targetChanged = false;
        jointStepSize = robot->curJointStep;

        // Find the joint with the furthest to go, and compute the number
        // of iterations it will take to get there
        for (i=0; i<NUM_JOINTS; i++)
        {
          double numSteps = fabs(diffJ[i]) / jointStepSize;
          if (numSteps > maxNumSteps)
            maxNumSteps = numSteps;
        }
        pthread_mutex_unlock(&nonBlockMutex);

        // If we have more than one iteration to go, scale the total 
        // difference by the magnitude of the current step and add it 
        // to the current position to get our new goal
        if (maxNumSteps > 1.0)
        {
          for (i=0; i<NUM_JOINTS; i++)
            newGoalJ[i] = robot->curGoalJ[i] + diffJ[i] / maxNumSteps;
        }
        else
        {
          // Otherwise, we will reach our goal during this step, so simply
          // add the entire difference to the current position to get our goal
          for (i=0; i<NUM_JOINTS; i++)
            newGoalJ[i] = robot->curGoalJ[i] + diffJ[i];
          reachedJ = true;
        }

        // Wait here until we're within range to last commanded goal
        ros::Rate check_rate(DIST_CHECK_FREQ);
        while (robot->jointDistFromGoal() > robot->curDist[2])
          check_rate.sleep();

        // Now do the joint move towards our new goal
        if (!robot->setJoints(newGoalJ[0], newGoalJ[1], newGoalJ[2],
              newGoalJ[3], newGoalJ[4], newGoalJ[5]))
        {
          ROS_INFO("Non-Blocking move error!");
          pthread_mutex_lock (&nonBlockMutex);
          robot->do_nb_move = false;
          pthread_mutex_unlock(&nonBlockMutex);
        }

        // If we have reached our goal, and the target hasn't been changed 
        // while we were doing the last move, we're done.
        pthread_mutex_lock (&nonBlockMutex);
        if (reachedJ && !robot->targetChanged)
          robot->do_nb_move = false;
        pthread_mutex_unlock(&nonBlockMutex);
      }

      // Make sure we don't hog all the CPU by running too fast
      nb_rate.sleep();
    }

    // If we get a stop request, confirm that we've stopped
    if (robot->stopRequest)
      robot->stopConfirm = true;

    // Make sure we don't hog all the CPU by running too fast
    nb_rate.sleep();
  }

  pthread_exit((void*) 0);
}



//////////////////////////////////////////////////////////////////////////////
// Main Loop for Robot Node
//
// This is what is executed when the robot node is run in ROS. We initialize
// the robot, start our logging and non-blocking threads, and start listening
// for messages. The program exits when ROS is shutdown.
//////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_controller");
  ros::NodeHandle node;
  RobotController ABBrobot(&node);

  // Initialize the Robot Node
  
  if (argc == 2){
    if (!ABBrobot.init(argv[1]))
      exit(-1);
  }
  else{
    if (!ABBrobot.init())
      exit(-1);
  } 
  

  // Initialize the mutex's we will be using in our threads
  pthread_mutex_init(&nonBlockMutex, NULL);
  pthread_mutex_init(&jointUpdateMutex, NULL);
  pthread_mutex_init(&cartUpdateMutex, NULL);
  pthread_mutex_init(&forceUpdateMutex, NULL);
  pthread_mutex_init(&sendRecvMutex, NULL);

  bool useLogger;
  node.param<bool>(ABBrobot.robotname_sl + "/useLogger", useLogger, true);

  // Create a dedicated thread for logger broadcasts
  pthread_t loggerThread;
  pthread_attr_t attrL;
  pthread_attr_init(&attrL);
  pthread_attr_setdetachstate(&attrL, PTHREAD_CREATE_JOINABLE);


  if (pthread_create(&loggerThread, &attrL, 
        loggerMain, (void*)&ABBrobot) != 0)
  {
    ROS_INFO("ROBOT_CONTROLLER: Unable to create logger thread. "
        "Error number: %d.",errno);
  }
  
  
  
  bool useRRI;
  node.param<bool>(ABBrobot.robotname_sl + "/useRRI", useRRI, false);
  
  // Create a dedicated thread for rri broadcasts
  pthread_t rriThread;
  pthread_attr_t attrR;
  pthread_attr_init(&attrR);
  pthread_attr_setdetachstate(&attrR, PTHREAD_CREATE_JOINABLE);
  if (useRRI){
    if (pthread_create(&rriThread, &attrR, 
        rriMain, (void*)&ABBrobot) != 0)
    {
    ROS_INFO("ROBOT_CONTROLLER: Unable to create rri thread. "
        "Error number: %d.",errno);
    }
  }
  
  // Create a dedicated thread for non-blocking moves
  pthread_t nonBlockThread;
  pthread_attr_t attrB;
  pthread_attr_init(&attrB);
  pthread_attr_setdetachstate(&attrB, PTHREAD_CREATE_JOINABLE);

  if(pthread_create(&nonBlockThread,  &attrB, 
        nonBlockMain, (void*)&ABBrobot) != 0)
  {
    ROS_INFO("ROBOT_CONTROLLER: Unable to create non-blocking thread."
        " Error number: %d.",errno);
  }

  //Advertise ROS services
  ROS_INFO("ROBOT_CONTROLLER: Advertising ROS services...");
  ABBrobot.advertiseServices();

  //Advertise ROS topics
  ROS_INFO("ROBOT_CONTROLLER: Advertising ROS topics...");
  ABBrobot.advertiseTopics();

  //Main ROS loop
  ROS_INFO("ROBOT_CONTROLLER: Running node /robot_controller...");
  // Multithreaded spinner so that callbacks 
  // can be handled on separate threads.
  ros::MultiThreadedSpinner spinner(4); // We have 4 total threads
  spinner.spin();
  ROS_INFO("ROBOT_CONTROLLER: Shutting down node /robot_controller...");

  //End threads
  void *statusL, *statusB, *statusR;
  if(useLogger) pthread_attr_destroy(&attrL);
  if(useRRI) pthread_attr_destroy(&attrR);
  pthread_attr_destroy(&attrB);
  pthread_mutex_destroy(&nonBlockMutex);
  pthread_mutex_destroy(&jointUpdateMutex);
  pthread_mutex_destroy(&cartUpdateMutex);
  pthread_mutex_destroy(&forceUpdateMutex);
  pthread_mutex_destroy(&sendRecvMutex);
  if(useLogger) pthread_join(loggerThread, &statusL);
  if(useRRI) pthread_join(rriThread, &statusR);
  pthread_join(nonBlockThread, &statusB);

  ROS_INFO("ROBOT_CONTROLLER: Done.");
  return 0;
}
