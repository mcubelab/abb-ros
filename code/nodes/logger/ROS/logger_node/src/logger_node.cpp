#include "logger_node.h"


#include "ros/ros.h"

void robot_CartesianLog_Callback(const robot_comm::robot_CartesianLog& msg)
{
  pthread_mutex_lock(&cartesianLogMutex);
  robot_CartesianLog_values[0] = msg.x;
  robot_CartesianLog_values[1] = msg.y;
  robot_CartesianLog_values[2] = msg.z;
  robot_CartesianLog_values[3] = msg.q0;
  robot_CartesianLog_values[4] = msg.qx;
  robot_CartesianLog_values[5] = msg.qy;
  robot_CartesianLog_values[6] = msg.qz;
  pthread_mutex_unlock(&cartesianLogMutex);
}

void robot_JointsLog_Callback(const robot_comm::robot_JointsLog& msg)
{
  pthread_mutex_lock(&jointsLogMutex);
  robot_JointsLog_values[0] = msg.j1;
  robot_JointsLog_values[1] = msg.j2;
  robot_JointsLog_values[2] = msg.j3;
  robot_JointsLog_values[3] = msg.j4;
  robot_JointsLog_values[4] = msg.j5;
  robot_JointsLog_values[5] = msg.j6;
  pthread_mutex_unlock(&jointsLogMutex);
}

void robot_ForceLog_Callback(const robot_comm::robot_ForceLog& msg)
{
  pthread_mutex_lock(&forceLogMutex);
  robot_ForceLog_values[0] = msg.fx;
  robot_ForceLog_values[1] = msg.fy;
  robot_ForceLog_values[2] = msg.fz;
  robot_ForceLog_values[3] = msg.tz;
  robot_ForceLog_values[4] = msg.ty;
  robot_ForceLog_values[5] = msg.tz;
  pthread_mutex_unlock(&forceLogMutex);
}

void hand_EncodersLog_Callback(const hand_comm::hand_EncodersLog& msg)
{
  pthread_mutex_lock(&encodersLogMutex);
  hand_EncodersLog_values[0] = msg.encMotor;
  for (int i=0; i < NUM_FINGERS; i++)
    hand_EncodersLog_values[i+1] = msg.encFinger[i];
  pthread_mutex_unlock(&encodersLogMutex);
}

void hand_AnglesLog_Callback(const hand_comm::hand_AnglesLog& msg)
{
  pthread_mutex_lock(&anglesLogMutex);
  hand_AnglesLog_values[0] = msg.angleMotor;
  for (int i=0; i < NUM_FINGERS; i++)
    hand_AnglesLog_values[i+1] = msg.angle[i];
  pthread_mutex_unlock(&anglesLogMutex);
}

void hand_RawForcesLog_Callback(const hand_comm::hand_RawForcesLog& msg)
{
  pthread_mutex_lock(&handRawForcesLogMutex);
  for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
    hand_RawForcesLog_values[i] = msg.forces[i];
  pthread_mutex_unlock(&handRawForcesLogMutex);
}

void hand_ForcesLog_Callback(const hand_comm::hand_ForcesLog& msg)
{
  pthread_mutex_lock(&handForcesLogMutex);
  for (int i=0; i < NUM_HAND_FORCES; i++)
    hand_ForcesLog_values[i] = msg.forces[i];
  pthread_mutex_unlock(&handForcesLogMutex);
}

void scanAndSubscribeTopics()
{
  ros::master::V_TopicInfo topics;
  ros::master::getTopics(topics);
  for(int i=0; i<(int)topics.size(); i++)
  {
    if(!robot_CartesianLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/robot_CartesianLog"))
      {
        handle_robot_CartesianLog = nodePtr->subscribe("robot_CartesianLog", 100, robot_CartesianLog_Callback);
        robot_CartesianLog_present = true;
        ROS_INFO("LOGGER: Topic /robot_CartesianLog found.");
      }
    }
    if(!robot_JointsLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/robot_JointsLog"))
      {
        handle_robot_JointsLog = nodePtr->subscribe("robot_JointsLog", 100, robot_JointsLog_Callback);
        robot_JointsLog_present = true;
        ROS_INFO("LOGGER: Topic /robot_JointsLog found.");
      }
    }
    if(!robot_ForceLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/robot_ForceLog"))
      {
        handle_robot_ForceLog = nodePtr->subscribe("robot_ForceLog", 100, robot_ForceLog_Callback);
        robot_ForceLog_present = true;
        ROS_INFO("LOGGER: Topic /robot_ForceLog found.");
      }
    }
    if(!hand_EncodersLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/hand_EncodersLog"))
      {
        handle_hand_EncodersLog = nodePtr->subscribe("hand_EncodersLog", 100, hand_EncodersLog_Callback);
        hand_EncodersLog_present = true;
        ROS_INFO("LOGGER: Topic /hand_EncodersLog found.");
      }
    }
    if(!hand_AnglesLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/hand_AnglesLog"))
      {
        handle_hand_AnglesLog = nodePtr->subscribe("hand_AnglesLog", 100, hand_AnglesLog_Callback);
        hand_AnglesLog_present = true;
        ROS_INFO("LOGGER: Topic /hand_AnglesLog found.");
      }
    }
    if(!hand_RawForcesLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/hand_RawForcesLog"))
      {
        handle_hand_RawForcesLog = nodePtr->subscribe("hand_RawForcesLog", 100, hand_RawForcesLog_Callback);
        hand_RawForcesLog_present = true;
        ROS_INFO("LOGGER: Topic /hand_RawForcesLog found.");
      }
    }
    if(!hand_ForcesLog_present)
    {
      if(!strcmp(topics[i].name.c_str(),"/hand_ForcesLog"))
      {
        handle_hand_ForcesLog = nodePtr->subscribe("hand_ForcesLog", 100, hand_ForcesLog_Callback);
        hand_ForcesLog_present = true;
        ROS_INFO("LOGGER: Topic /hand_ForcesLog found.");
      }
    }
  }
}

void advertiseServices()
{
  handle_logger_Start = nodePtr->advertiseService("logger_Start", logger_Start);
  handle_logger_Stop = nodePtr->advertiseService("logger_Stop", logger_Stop);
  handle_logger_Append = nodePtr->advertiseService("logger_Append",logger_Append);
  handle_logger_Create = nodePtr->advertiseService("logger_Create",logger_Create);
  handle_logger_Copy = nodePtr->advertiseService("logger_Copy",logger_Copy);
}

void advertiseTopics()
{
  handle_logger_SystemLog = nodePtr->advertise<logger_comm::logger_SystemLog>("logger_SystemLog", 100);
}


bool logger_Start(logger_comm::logger_Start::Request& req, logger_comm::logger_Start::Response& res)
{
  if (logging)
  {
    ROS_WARN("Logger already in logging state. Deleting previous file and starting new one...");
    stopLog();
    remove(currentFileName);
    logging = false;
  }

  if(!robot_CartesianLog_present &&
      !robot_JointsLog_present &&
      !robot_ForceLog_present &&
      !hand_EncodersLog_present &&
      !hand_AnglesLog_present)
  {
    res.ret = 0;
    res.msg = "LOGGER: Not available topics to log.";
    return false;     
  }
  //Set what topic to log to file based on what is present at this same moment
  robot_CartesianLog_logging = robot_CartesianLog_present;
  robot_JointsLog_logging = robot_JointsLog_present; 
  robot_ForceLog_logging = robot_ForceLog_present;
  hand_EncodersLog_logging = hand_EncodersLog_present;
  hand_AnglesLog_logging = hand_AnglesLog_present;
  hand_RawForcesLog_logging = hand_RawForcesLog_present;
  hand_ForcesLog_logging = hand_ForcesLog_present;

  char fileName[LOG_BUFFER];
  char folderName[LOG_BUFFER];
  char completeFileName[LOG_BUFFER];
  time_t timer;

  // Name of the file where to save it
  timer=time(NULL);
  tm* today;
  today = localtime(&timer);
  sprintf(fileName,"Log__%d:%02d:%02d__%02d:%02d:%02d.txt",
      today->tm_year+1900,
      today->tm_mon+1,
      today->tm_mday,
      today->tm_hour,
      today->tm_min,
      today->tm_sec);

  // Folder where to save it
  std::string folder;
  nodePtr->getParam("/logger/folder", folder);
  sprintf(folderName,"%s/%s/%s",
      folder.c_str(),
      COMMON_FOLDER,
      req.folder.c_str());

  //If it does not exist we create it
  struct stat st;
  if(stat(folderName,&st) < 0)
  {
    //We create the folder
    if((mkdir(folderName,00777))==-1)
    {
      res.ret = 0;
      res.msg = "LOGGER: Impossible to create folder.";
      ROS_WARN("%s : %s",res.msg.c_str(),folderName);
      return false;	
    }
  }

  // Complete filename
  sprintf(completeFileName,"%s/%s",folderName,fileName);

  //Open file
  fLog = fopen(completeFileName,"w");
  if(!fLog)
  {
    res.ret = 0;
    res.msg = "LOGGER: Could not create the log file.";
    ROS_INFO("%s : %s",res.msg.c_str(),completeFileName);
    //fclose(fLog);
    return false;
  }
  else
  {
    sprintf(currentFileName, "%s", completeFileName);
  }

  //File Header
  id = req.id;
  fprintf(fLog,"%d\n",(int)req.id);
  fprintf(fLog,"timeStamp,");
  if(robot_CartesianLog_logging)
  {
    fprintf(fLog,"x,");
    fprintf(fLog,"y,");
    fprintf(fLog,"z,");
    fprintf(fLog,"q0,");
    fprintf(fLog,"qx,");
    fprintf(fLog,"qy,");
    fprintf(fLog,"qz,");
  }
  if(robot_JointsLog_logging)
  {
    fprintf(fLog,"j1,");
    fprintf(fLog,"j2,");
    fprintf(fLog,"j3,");
    fprintf(fLog,"j4,");
    fprintf(fLog,"j5,");
    fprintf(fLog,"j6,");
  }
  if(robot_ForceLog_logging)
  {
    fprintf(fLog,"fx,");
    fprintf(fLog,"fy,");
    fprintf(fLog,"fz,");
    fprintf(fLog,"tx,");
    fprintf(fLog,"ty,");
    fprintf(fLog,"tz,");
  }
  if(hand_EncodersLog_logging)
  {
    fprintf(fLog,"encMotor,");
    for (int i=0; i < NUM_FINGERS; i++)
      fprintf(fLog,"encFinger%d,", i+1);
  }
  if(hand_AnglesLog_logging)
  {
    fprintf(fLog,"angleMotor,");
    for (int i=0; i < NUM_FINGERS; i++)
      fprintf(fLog, "angle%d,", i+1);
  }
  if(hand_RawForcesLog_logging)
  {
    for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
      fprintf(fLog, "rawForce%d,", i);
  }
  if(hand_ForcesLog_logging)
  {
    for (int i=0; i < NUM_HAND_FORCES; i++)
      fprintf(fLog, "force%d,", i);
  }

  //Start logging
  ROS_INFO("LOGGER: Start logging to file %s.",fileName);
  logging = true;
  logStartTime = ros::Time::now();

  char returnFileName[LOG_BUFFER];
  sprintf(returnFileName, "%s/%s/%s",
      COMMON_FOLDER,
      req.folder.c_str(),
      fileName);
  res.filename = returnFileName;
  res.ret = 1;
  return true;
}

bool stopLog()
{
  if(logging)
  {
    fclose(fLog);
    logging = false;
    ros::Time logTime = ros::Time::now();
    ros::Duration timeStamp = logTime - logStartTime;
    ROS_INFO("LOGGER: Stop logging. %.2lf seconds.",timeStamp.toSec());
    return true;
  }
  else
  {
    ROS_WARN("LOGGER: Not in logging state. Can't stop log.");
    return false;
  }
}

bool logger_Stop(logger_comm::logger_Stop::Request& req, logger_comm::logger_Stop::Response& res)
{
  if (stopLog())
  {
    res.ret = 1;
    return true;
  }
  else
  {
    res.ret = 0;
    res.msg = "LOGGER: Not in logging state.";
    return false;
  }
}

bool logger_Append(logger_comm::logger_Append::Request& req, logger_comm::logger_Append::Response& res)
{
  FILE * f;
  char filename[LOG_BUFFER];

  if(!logging)
  {
    std::string folder;
    nodePtr->getParam("/logger/folder", folder);
    sprintf(filename,"%s/%s",
        folder.c_str(),
        req.filename.c_str());
    //First we check if the file exists
    struct stat st;
    if(stat(filename,&st) < 0)
    {
      res.ret = 0;
      res.msg = "LOGGER: Filename not valid.";
      return false;	
    }
    //We open the file
    f = fopen(filename,"a");
    if(!f)
    {
      res.ret = 0;
      res.msg = "LOGGER: Could not open the specified file.";
      ROS_INFO("%s : %s",res.msg.c_str(),filename);
      //fclose(f);
      return false;
    }
    //Append info
    fprintf(f,"\n%s",req.info.c_str());
    fclose(f);
    res.ret = 1;
    res.msg = "LOGGER: OK";
    return true;
  }
  res.ret = 0;
  res.msg = "LOGGER: Cannot append to file while logging.";
  return false;	 
}

bool logger_Create(logger_comm::logger_Create::Request& req, logger_comm::logger_Create::Response& res)
{
  FILE *fLog;
  std::string logFolder;
  char fileName[LOG_BUFFER];
  char completeFileName[LOG_BUFFER];

  time_t timer;
  timer=time(NULL);
  tm* today;
  today = localtime(&timer);

  nodePtr->getParam("/logger/folder", logFolder);
  sprintf(fileName,"&MLog__%d:%02d:%02d__%02d:%02d:%02d.txt",
      today->tm_year+1900,
      today->tm_mon+1,
      today->tm_mday,
      today->tm_hour,
      today->tm_min,
      today->tm_sec);

  sprintf(completeFileName,"%s/%s/%s",logFolder.c_str(),COMMON_FOLDER,fileName);
  fLog = fopen(completeFileName,"w");
  if(!fLog)
  {
    res.ret = 0;
    res.msg = "LOGGER: Could not create the log file.";
    ROS_WARN("%s : %s",res.msg.c_str(),completeFileName);
    //fclose(fLog);
    return false;
  }
  fclose(fLog);

  char returnFileName[LOG_BUFFER];
  sprintf(returnFileName,"%s/%s",COMMON_FOLDER,fileName);
  res.filename = returnFileName;
  res.ret = 1;
  res.msg = "LOGGER: OK.";
  return true;
}    

bool logger_Copy(logger_comm::logger_Copy::Request& req, logger_comm::logger_Copy::Response& res)
{
  std::string logFolder;
  char fileName[LOG_BUFFER];
  char newFileName[LOG_BUFFER];
  char folderName[LOG_BUFFER];
  char command[LOG_BUFFER];

  nodePtr->getParam("/logger/folder", logFolder);

  //Check that the file exists
  sprintf(fileName,"%s/%s",
      logFolder.c_str(),
      req.filename.c_str());
  struct stat st;
  if(stat(fileName,&st) < 0)
  {
    res.ret = 0;
    res.msg = "LOGGER: Filename not valid.";
    ROS_WARN("%s : %s",res.msg.c_str(),fileName);
    return false;	
  }

  //Check if the folder exists
  sprintf(folderName,"%s/%s",
      logFolder.c_str(),
      req.folder.c_str());
  if(stat(folderName,&st) < 0)
  {
    //We create the folder
    if((mkdir(folderName,00777))==-1)
    {
      res.ret = 0;
      res.msg = "LOGGER: Impossible to create folder.";
      ROS_WARN("%s : %s",res.msg.c_str(),folderName);
      return false;	
    }
  }

  // Extract the folder from the fileName
  char file[LOG_BUFFER];
  char fileMinusFolder[LOG_BUFFER];
  strcpy(file, req.filename.c_str());
  char *ptr = strrchr ( file, '/');
  if(ptr == NULL)
    strcpy(fileMinusFolder, file);
  else
    {
      ptr++;
      strcpy(fileMinusFolder, ptr);
    }
  
  //Copy file
  sprintf(newFileName,"%s/%s/%s",
      logFolder.c_str(),
      req.folder.c_str(),
      fileMinusFolder);
  sprintf(command,"cp %s %s",fileName, newFileName);
  if(system(command)!=0)
  {
    res.ret = 0;
    res.msg = "LOGGER: There was an error while copying the file.";
    ROS_WARN("%s : %s",res.msg.c_str(),newFileName);
    return false;	
  }
  res.ret = 1;
  res.msg = "LOGGER: OK.";
  return true;
}    


void scannerCallBack(const ros::TimerEvent& event)
{
  //We should probably mutex this
  if(!logging)
    scanAndSubscribeTopics();
}

void logCallback(const ros::TimerEvent& event)
{
  if(logging)
    fprintf(fLog,"\n");

  //Add timer
  ros::Time logTime = ros::Time::now();
  ros::Duration timeStamp = logTime - logStartTime;

  logger_comm::logger_SystemLog msg;
  msg.timeStamp = logTime.toSec();
  if(logging)
    msg.logtimeStamp = timeStamp.toSec();
  else
    msg.logtimeStamp = 0;

  if(logging)
    fprintf(fLog,"%09.3lf,",timeStamp.toSec());

  msg.id = id;

  pthread_mutex_lock(&cartesianLogMutex);
  msg.x = robot_CartesianLog_values[0];
  msg.y = robot_CartesianLog_values[1];
  msg.z = robot_CartesianLog_values[2];
  msg.q0 = robot_CartesianLog_values[3];
  msg.qx = robot_CartesianLog_values[4];
  msg.qy = robot_CartesianLog_values[5];
  msg.qz = robot_CartesianLog_values[6];
  if(logging && robot_CartesianLog_logging)
  {	  
    fprintf(fLog,"%.1lf,",robot_CartesianLog_values[0]);
    fprintf(fLog,"%.1lf,",robot_CartesianLog_values[1]);
    fprintf(fLog,"%.1lf,",robot_CartesianLog_values[2]);
    fprintf(fLog,"%.4lf,",robot_CartesianLog_values[3]);
    fprintf(fLog,"%.4lf,",robot_CartesianLog_values[4]);
    fprintf(fLog,"%.4lf,",robot_CartesianLog_values[5]);
    fprintf(fLog,"%.4lf,",robot_CartesianLog_values[6]);
  }
  pthread_mutex_unlock(&cartesianLogMutex);

  pthread_mutex_lock(&jointsLogMutex);
  msg.j1 = robot_JointsLog_values[0];
  msg.j2 = robot_JointsLog_values[1];
  msg.j3 = robot_JointsLog_values[2];
  msg.j4 = robot_JointsLog_values[3];
  msg.j5 = robot_JointsLog_values[4];
  msg.j6 = robot_JointsLog_values[5];
  if(logging && robot_JointsLog_logging)
  {
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[0]);
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[1]);
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[2]);
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[3]);
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[4]);
    fprintf(fLog,"%.2lf,",robot_JointsLog_values[5]);
  }
  pthread_mutex_unlock(&jointsLogMutex);

  pthread_mutex_lock(&forceLogMutex);
  msg.fx = robot_ForceLog_values[0];
  msg.fy = robot_ForceLog_values[1];
  msg.fz = robot_ForceLog_values[2];
  msg.tx = robot_ForceLog_values[3];
  msg.ty = robot_ForceLog_values[4];
  msg.tz = robot_ForceLog_values[5];
  if(logging && robot_ForceLog_logging)
  {
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[0]);
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[1]);
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[2]);  
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[3]);
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[4]);
    fprintf(fLog,"%.3lf,",robot_ForceLog_values[5]);
  }
  pthread_mutex_unlock(&forceLogMutex);

  pthread_mutex_lock(&encodersLogMutex);
  msg.encMotor = hand_EncodersLog_values[0];
  msg.encFinger.resize(NUM_FINGERS);
  for (int i=0; i < NUM_FINGERS; i++)
    msg.encFinger[i] = hand_EncodersLog_values[i+1];
  if(logging && hand_EncodersLog_logging)
  {
    for (int i=0; i < NUM_FINGERS+1; i++)
      fprintf(fLog,"%d,",hand_EncodersLog_values[i]);
  }
  pthread_mutex_unlock(&encodersLogMutex);

  pthread_mutex_lock(&anglesLogMutex);
  msg.angleMotor = hand_AnglesLog_values[0]; 
  msg.angle.resize(NUM_FINGERS);
  for (int i=0; i < NUM_FINGERS; i++)
    msg.angle[i] = hand_AnglesLog_values[i+1];
  if(logging && hand_AnglesLog_logging)
  {
    for (int i=0; i < NUM_FINGERS+1; i++)
      fprintf(fLog,"%.2lf,",hand_AnglesLog_values[i]);
  }
  pthread_mutex_unlock(&anglesLogMutex);

  pthread_mutex_lock(&handRawForcesLogMutex);
  msg.rawForces.resize(NUM_RAW_HAND_FORCES);
  for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
    msg.rawForces[i] = hand_RawForcesLog_values[i];
  if(logging && hand_RawForcesLog_logging)
  {
    for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
      fprintf(fLog,"%d,",hand_RawForcesLog_values[i]);
  }
  pthread_mutex_unlock(&handRawForcesLogMutex);
  
  pthread_mutex_lock(&handForcesLogMutex);
  msg.forces.resize(NUM_HAND_FORCES);
  for (int i=0; i < NUM_HAND_FORCES; i++)
    msg.forces[i] = hand_ForcesLog_values[i];
  if(logging && hand_ForcesLog_logging)
  {
    for (int i=0; i < NUM_HAND_FORCES; i++)
      fprintf(fLog,"%.2lf,",hand_ForcesLog_values[i]);
  }
  pthread_mutex_unlock(&handForcesLogMutex);

  handle_logger_SystemLog.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "logger");
  ros::NodeHandle node;
  nodePtr = &node;
  logging = false;
  robot_CartesianLog_values[0] = 0.0;
  robot_CartesianLog_values[1] = 0.0;
  robot_CartesianLog_values[2] = 0.0;
  robot_CartesianLog_values[3] = 0.0;
  robot_CartesianLog_values[4] = 0.0;
  robot_CartesianLog_values[5] = 0.0;
  robot_CartesianLog_values[6] = 0.0;
  robot_JointsLog_values[0] = 0.0;
  robot_JointsLog_values[1] = 0.0;
  robot_JointsLog_values[2] = 0.0;
  robot_JointsLog_values[3] = 0.0;
  robot_JointsLog_values[4] = 0.0;
  robot_JointsLog_values[5] = 0.0;
  robot_ForceLog_values[0] = 0.0;
  robot_ForceLog_values[1] = 0.0;
  robot_ForceLog_values[2] = 0.0;
  robot_ForceLog_values[3] = 0.0;
  robot_ForceLog_values[4] = 0.0;
  robot_ForceLog_values[5] = 0.0;
  for (int i=0; i < NUM_FINGERS+1; i++)
  {
    hand_EncodersLog_values[i] = 0;
    hand_AnglesLog_values[i] = 0.0;
  }
  for (int i=0; i < NUM_RAW_HAND_FORCES; i++)
  {
    hand_RawForcesLog_values[i] = 0;
  }
  for (int i=0; i < NUM_HAND_FORCES; i++)
  {
    hand_ForcesLog_values[i] = 0.0;
  }

  // Initialize Mutexes
  pthread_mutex_init(&cartesianLogMutex, NULL);
  pthread_mutex_init(&jointsLogMutex, NULL);
  pthread_mutex_init(&forceLogMutex, NULL);
  pthread_mutex_init(&encodersLogMutex, NULL);
  pthread_mutex_init(&anglesLogMutex, NULL);
  pthread_mutex_init(&handRawForcesLogMutex, NULL);
  pthread_mutex_init(&handForcesLogMutex, NULL);

  //Subscribing to topics
  ROS_INFO("LOGGER: Subscribing to available topics...");
  robot_CartesianLog_present = false;
  robot_JointsLog_present = false;
  robot_ForceLog_present = false;
  hand_EncodersLog_present = false;
  hand_AnglesLog_present = false;
  hand_RawForcesLog_present = false;
  hand_ForcesLog_present = false;
  scanAndSubscribeTopics();
  robot_CartesianLog_logging = robot_CartesianLog_present;
  robot_JointsLog_logging = robot_JointsLog_present;
  robot_ForceLog_logging = robot_ForceLog_present;
  hand_EncodersLog_logging = hand_EncodersLog_present;
  hand_AnglesLog_logging = hand_AnglesLog_present;
  hand_RawForcesLog_logging = hand_RawForcesLog_present;
  hand_ForcesLog_logging = hand_ForcesLog_present;

  //Log Timer and Callback
  double samplingRate;
  nodePtr->getParam("/logger/samplingRate", samplingRate);
  loggerTimer = nodePtr->createTimer(ros::Duration(1.0/samplingRate), logCallback);

  //Do periodic scans of available topics every 2.0 seconds
  scannerTimer = nodePtr->createTimer(ros::Duration(2.0), scannerCallBack);

  //Advertising ROS services
  ROS_INFO("LOGGER: Advertising ROS services...");
  advertiseServices();

  //Advertising ROS topics
  ROS_INFO("LOGGER: Advertising ROS topics...");
  advertiseTopics();

  //Main ROS loop
  ROS_INFO("LOGGER: Running node /logger at %.1lf Hz ...",samplingRate);
  ros::spin();
  ROS_INFO("LOGGER: Shutting down node /logger...");

  // Destroy our mutexes
  pthread_mutex_destroy(&cartesianLogMutex);
  pthread_mutex_destroy(&jointsLogMutex);
  pthread_mutex_destroy(&forceLogMutex);
  pthread_mutex_destroy(&encodersLogMutex);
  pthread_mutex_destroy(&anglesLogMutex);
  pthread_mutex_destroy(&handRawForcesLogMutex);
  pthread_mutex_destroy(&handForcesLogMutex);

  loggerTimer.stop();

  return 0;
}
