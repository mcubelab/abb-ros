#include "matlab_node.h"

// Evaluate the given matlab string. If echo is true, also print out anything
//  from the console (assuming there are no errors). Note that this function
//  also returns true or false based on whether the command was executed
//  successfully.
bool evalString(char *buffer, bool echo)
{
  if (engEvalString(matEng, buffer)!=0)
    return false;
  else if (echo)
  {
    if (echo_buffer[2] != '\0')
    {
      ROS_INFO("%s", echo_buffer+2);
      echo_buffer[2] = '\0';
    }
  }

  return true;
}

bool matlab_Ping(matlab_comm::matlab_Ping::Request& req, matlab_comm::matlab_Ping::Response& res)
{
  if(ready)
    {
      res.ret = 1;
      res.msg = "MATLAB_INTERFACE: OK.";
      return true;
    }
  res.ret = 0;
  res.msg = "MATLAB_INTERFACE: MATLAB not yet ready.";
  return false;
}

bool matlab_SendCommand(matlab_comm::matlab_SendCommand::Request& req, matlab_comm::matlab_SendCommand::Response& res)
{
  char buffer[MAX_BUFFER];

  //Send command to MATLAB  
  sprintf(buffer,"%s",req.command.c_str());
  if(!evalString(buffer, true))
    {
      
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Matlab encountered a problem with the command: \"" + req.command + "\"";
      ROS_WARN("%s",res.msg.c_str());
      return false;
    }
  res.ret = 1;
  res.msg = "MATLAB_INTERFACE: OK.";
  return true;
}

bool matlab_PutArray(matlab_comm::matlab_PutArray::Request& req, matlab_comm::matlab_PutArray::Response& res)
{
  //Array size checking
  if(req.nrows*req.ncols != (int)req.data.size())
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Size of data does not correspond to n x m.";
      ROS_WARN("%s",res.msg.c_str());
      return false;
    }

  //Transform into matlab format (column major order)
  mwSize dims[] = { req.nrows, req.ncols };
  mxArray *T = mxCreateNumericArray(2, dims, mxDOUBLE_CLASS, mxREAL);
  double *p=mxGetPr(T);
  for (int i=0;i<req.ncols;i++) 
      for (int j=0;j<req.nrows;j++) 
	  *p++ = req.data[i + j*req.ncols];

  //Send variable to matlab
  if(engPutVariable(matEng, req.name.c_str(), T)!=0)
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Error trying to put the variable \"" + req.name + "\" into the MATLAB workspace.";
      ROS_WARN("%s",res.msg.c_str());
      mxDestroyArray(T);
      return false;
    }
  res.ret = 1;
  res.msg = "MATLAB_INTERFACE: OK.";
  mxDestroyArray(T);
  return true;
}

bool matlab_GetArray(matlab_comm::matlab_GetArray::Request& req, matlab_comm::matlab_GetArray::Response& res)
{
  mxArray *T;
  //Get variable to matlab
  if((T=engGetVariable(matEng, req.name.c_str()))==NULL)
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Error trying to get the variable \"" + req.name + "\" from the MATLAB workspace.";
      ROS_WARN("%s",res.msg.c_str());
      mxDestroyArray(T);
      return false;
    }

  //Transform into ROS format
  size_t nDims = mxGetNumberOfDimensions(T);
  if (nDims>2)
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Error - The variable is an array of dimension higher than 2.";
      ROS_WARN("%s",res.msg.c_str());
      mxDestroyArray(T);
      return false;
    }
  res.nrows = mxGetM(T);
  res.ncols = mxGetN(T);
  res.data.resize(res.nrows*res.ncols);
  double *p=mxGetPr(T);
  for (int i=0;i<res.ncols;i++)
    for (int j=0;j<res.nrows;j++)
      res.data[i + j*res.ncols] = *p++;
  
  res.ret = 1;
  res.msg = "MATLAB_INTERFACE: OK.";
  mxDestroyArray(T);
  return true;
}

bool matlab_PutString(matlab_comm::matlab_PutString::Request& req, matlab_comm::matlab_PutString::Response&res)
{
  char buffer[MAX_BUFFER];
  
  //Put string
  sprintf(buffer,"%s = '%s'",req.name.c_str(),req.data.c_str());
  if(!evalString(buffer, false))
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Error trying to put the string \"" + req.data + "\" in the MATLAB workspace" ;
      ROS_WARN("%s",res.msg.c_str());
      return false;
    }
  res.ret = 1;
  res.msg = "MATLAB_INTERFACE: OK.";
  return true;
}

bool matlab_GetString(matlab_comm::matlab_GetString::Request& req, matlab_comm::matlab_GetString::Response&res)
{
  char buffer[MAX_BUFFER];
  mxArray *T = NULL;

  //Recover string
  if((T = engGetVariable(matEng,req.name.c_str()))==NULL)
    {
      res.ret = 0;
      res.msg = "MATLAB_INTERFACE: Error trying to recover the string \"" + req.name + "\" from the MATLAB workspace.";
      ROS_WARN("%s",res.msg.c_str());
      mxDestroyArray(T);
      return false;
    }

  //Parse reply
  if(mxGetString(T, buffer, MAX_BUFFER)!=0)
    {
      res.data = "";
      res.msg = "MATLAB_INTERFACE: The variable \"" + req.name + "\" is not a string.";
    }
  else
    {
      res.data = buffer;
      res.msg = "MATLAB_INTERFACE: OK.";
    }
  res.ret = 1;
  mxDestroyArray(T);
  return true;
}

bool matlab_AddPath(matlab_comm::matlab_AddPath::Request& req, matlab_comm::matlab_AddPath::Response&res)
{
  // Add requested folder and all subfolders to path
  char buffer[MAX_BUFFER];
  sprintf(buffer,"addpath(genpath('%s'));",req.folder.c_str());
  ROS_INFO("%s", buffer);
  return evalString(buffer, false);
}

int main(int argc, char** argv)
{
  ready = false;
  ros::init(argc, argv, "matlab_node");
  ros::NodeHandle node;
  nodePtr = &node;
  //Initialize Matlab Engine
  ROS_INFO("MATLAB_INTERFACE: Starting MATLAB...");
  if (!(matEng = engOpen(NULL)))
    {
      ROS_ERROR("MATLAB_INTERFACE: Cannot start MATLAB.");
      return -1;
    }
  
  // This enables us to store matlab output that 
  //  gets printed when executing commands
  echo_buffer[MAX_MATLAB_BUFFER] = '\0';
  echo_buffer[2] = '\0';
  engOutputBuffer(matEng, echo_buffer, MAX_MATLAB_BUFFER);

  //Clear Matlab Workspace
  ROS_INFO("MATLAB_INTERFACE: Clearing MATLAB Workspace...");
  evalString((char*)"clear", false);
  
  //Load path with matlab scripts
  ROS_INFO("MATLAB_INTERFACE: Adding scripts folder and subfolders to MATLAB path...");
  std::string folder;
  nodePtr->getParam("/matlab/folder", folder);
  char buffer[MAX_BUFFER];
  sprintf(buffer,"addpath(genpath('%s'));",folder.c_str());
  ROS_INFO("%s", buffer);
  evalString(buffer, false);

  //Advertising ROS services
  ROS_INFO("MATLAB_INTERFACE: Advertising ROS services...");
  handle_matlab_Ping = nodePtr->advertiseService("matlab_Ping",matlab_Ping);
  handle_matlab_SendCommand = nodePtr->advertiseService("matlab_SendCommand",matlab_SendCommand);
  handle_matlab_PutArray = nodePtr->advertiseService("matlab_PutArray",matlab_PutArray);
  handle_matlab_GetArray = nodePtr->advertiseService("matlab_GetArray",matlab_GetArray);
  handle_matlab_PutString = nodePtr->advertiseService("matlab_PutString",matlab_PutString);
  handle_matlab_GetString = nodePtr->advertiseService("matlab_GetString",matlab_GetString);
  handle_matlab_AddPath = nodePtr->advertiseService("matlab_AddPath",matlab_AddPath);

  //Main ROS loop
  ready = true;
  ROS_INFO("MATLAB_INTERFACE: Running node /matlab_node...");
  ros::spin();
  ROS_INFO("MATLAB_INTERFACE: Shutting down node /matlab_node...");
  
  ROS_INFO("MATLAB_INTERFACE: Shutting down services ...");
  handle_matlab_Ping.shutdown();
  handle_matlab_SendCommand.shutdown();
  handle_matlab_PutArray.shutdown();
  handle_matlab_GetArray.shutdown();
  handle_matlab_PutString.shutdown();
  handle_matlab_GetString.shutdown();
  handle_matlab_AddPath.shutdown();
  ready= false;

  //Close the MATLAB engine
  engClose(matEng);
  printf("MATLAB_INTERFACE: MATLAB closed.\n");
}
