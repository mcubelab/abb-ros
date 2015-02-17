#include "matlab_comm.h"

MatlabComm::MatlabComm()
{
}

MatlabComm::MatlabComm(ros::NodeHandle* np)
{
  subscribe(np);
}

MatlabComm::~MatlabComm()
{
  shutdown();
}


void MatlabComm::subscribe(ros::NodeHandle* np)
{
  handle_matlab_Ping = 
    np->serviceClient<matlab_comm::matlab_Ping>("matlab_Ping");
  handle_matlab_SendCommand = 
    np->serviceClient<matlab_comm::matlab_SendCommand>("matlab_SendCommand");
  handle_matlab_PutArray = 
    np->serviceClient<matlab_comm::matlab_PutArray>("matlab_PutArray");
  handle_matlab_GetArray = 
    np->serviceClient<matlab_comm::matlab_GetArray>("matlab_GetArray");
  handle_matlab_PutString = 
    np->serviceClient<matlab_comm::matlab_PutString>("matlab_PutString");
  handle_matlab_GetString = 
    np->serviceClient<matlab_comm::matlab_GetString>("matlab_GetString");
  handle_matlab_AddPath = 
    np->serviceClient<matlab_comm::matlab_AddPath>("matlab_AddPath");
}

void MatlabComm::shutdown()
{
  handle_matlab_Ping.shutdown();
  handle_matlab_SendCommand.shutdown();
  handle_matlab_PutArray.shutdown();
  handle_matlab_GetArray.shutdown();
  handle_matlab_PutString.shutdown();
  handle_matlab_GetString.shutdown();
  handle_matlab_AddPath.shutdown();
}

//Client functions to simplify calling Matlab ROS services
bool MatlabComm::Ping()
{
  return handle_matlab_Ping.call(matlab_Ping_srv);
}

bool MatlabComm::sendCommand(const char *command)
{
  matlab_SendCommand_srv.request.command = command;
  return handle_matlab_SendCommand.call(matlab_SendCommand_srv);
}

bool MatlabComm::sendMat(const char *name, int n, int m, double* data)
{
  matlab_PutArray_srv.request.nrows = n;
  matlab_PutArray_srv.request.ncols = m;
  matlab_PutArray_srv.request.name = name;
  matlab_PutArray_srv.request.data.resize(n*m);
  memcpy(&matlab_PutArray_srv.request.data[0],data,n*m*sizeof(double));
  return handle_matlab_PutArray.call(matlab_PutArray_srv);
}


bool MatlabComm::sendMat(const char *name, const Mat &m)
{
  matlab_PutArray_srv.request.nrows = m.nn;
  matlab_PutArray_srv.request.ncols = m.mm;
  matlab_PutArray_srv.request.name = name;
  matlab_PutArray_srv.request.data.resize(m.nn*m.mm);
  memcpy(&matlab_PutArray_srv.request.data[0],&m[0][0],m.nn*m.mm*sizeof(double));
  return handle_matlab_PutArray.call(matlab_PutArray_srv);
}


bool MatlabComm::sendVec(const char *name, int n, double* data)
{
  matlab_PutArray_srv.request.nrows = 1;
  matlab_PutArray_srv.request.ncols = n;
  matlab_PutArray_srv.request.name = name;
  matlab_PutArray_srv.request.data.resize(n);
  memcpy(&matlab_PutArray_srv.request.data[0],data,n*sizeof(double));
  return handle_matlab_PutArray.call(matlab_PutArray_srv);
}

bool MatlabComm::sendVec(const char *name, const Vec &v)
{
  matlab_PutArray_srv.request.nrows = 1;
  matlab_PutArray_srv.request.ncols = v.nn;
  matlab_PutArray_srv.request.name = name;
  matlab_PutArray_srv.request.data.resize(v.nn);
  memcpy(&matlab_PutArray_srv.request.data[0],&v[0],v.nn*sizeof(double));
  return handle_matlab_PutArray.call(matlab_PutArray_srv);
}

bool MatlabComm::sendValue(const char *name, double v)
{
  matlab_PutArray_srv.request.nrows = 1;
  matlab_PutArray_srv.request.ncols = 1;
  matlab_PutArray_srv.request.name = name;
  matlab_PutArray_srv.request.data.resize(1);
  matlab_PutArray_srv.request.data[0] = v;
  return handle_matlab_PutArray.call(matlab_PutArray_srv);
}

bool MatlabComm::sendString(const char *name, char *str)
{
  matlab_PutString_srv.request.name = name;
  matlab_PutString_srv.request.data = str;
  return handle_matlab_PutString.call(matlab_PutString_srv);
}

bool MatlabComm::addPath(const char* folder)
{
  matlab_AddPath_srv.request.folder = folder;
  return handle_matlab_AddPath.call(matlab_AddPath_srv);
}

//receive
Mat MatlabComm::getMat(const char *name)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    Mat m(matlab_GetArray_srv.response.nrows,matlab_GetArray_srv.response.ncols);
    if(memcpy(&m[0][0],&matlab_GetArray_srv.response.data[0],m.nn * m.mm *sizeof(double)))
      return m;
  }
  return Mat();
}

bool MatlabComm::getMat(const char *name, Mat &m)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    m = Mat(matlab_GetArray_srv.response.nrows,matlab_GetArray_srv.response.ncols);
    if(memcpy(&m[0][0],&matlab_GetArray_srv.response.data[0],m.nn * m.mm * sizeof(double)))
      return true;
  }
  return false;
}

Vec MatlabComm::getVec(const char *name)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    if((matlab_GetArray_srv.response.nrows != 1) && (matlab_GetArray_srv.response.ncols != 1))
      return Vec();

    int dim;
    if(matlab_GetArray_srv.response.nrows == 1)
      dim = matlab_GetArray_srv.response.ncols;
    else
      dim = matlab_GetArray_srv.response.nrows;
    Vec v(dim);
    if(memcpy(&v[0],&matlab_GetArray_srv.response.data[0],v.nn * sizeof(double)))
      return v;
  }
  return Vec();
}

bool MatlabComm::getVec(const char *name, Vec &v)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    if((matlab_GetArray_srv.response.nrows != 1) && (matlab_GetArray_srv.response.ncols != 1))
      return false;
    if(matlab_GetArray_srv.response.nrows == 1)
      v = Vec(matlab_GetArray_srv.response.ncols);
    else
      v = Vec(matlab_GetArray_srv.response.nrows);
    if(memcpy(&v[0],&matlab_GetArray_srv.response.data[0],v.nn * sizeof(double)))
      return true;
  }
  return false;

}

Quaternion MatlabComm::getQuaternion(const char *name)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    if((matlab_GetArray_srv.response.nrows != 1) && (matlab_GetArray_srv.response.ncols != 1))
      return Quaternion();

    int dim;
    if(matlab_GetArray_srv.response.nrows == 1)
      dim = matlab_GetArray_srv.response.ncols;
    else
      dim = matlab_GetArray_srv.response.nrows;
    Quaternion q(dim);
    if(memcpy(&q[0],&matlab_GetArray_srv.response.data[0],q.nn * sizeof(double)))
      return q;
  }
  return Quaternion();
}

double MatlabComm::getValue(const char *name)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    if((matlab_GetArray_srv.response.nrows != 1) || (matlab_GetArray_srv.response.ncols != 1))
    {
      ROS_WARN("matlab_comm::getValue(): Tried to get a value, got a matrix instead.");
      return 0;
    }
    return matlab_GetArray_srv.response.data[0];
  }
  ROS_WARN("matlab_comm::getValue(): Couldn't call service");
  return 0;
}

bool MatlabComm::getValue(const char *name, double *v)
{
  matlab_GetArray_srv.request.name = name;
  if(handle_matlab_GetArray.call(matlab_GetArray_srv))
  {
    if((matlab_GetArray_srv.response.nrows != 1) || (matlab_GetArray_srv.response.ncols != 1))
      return false;
    *v=matlab_GetArray_srv.response.data[0];
    return true;
  }
  return false;
}


std::string MatlabComm::getString(const char *name)
{
  matlab_GetString_srv.request.name = name;
  if(handle_matlab_GetString.call(matlab_GetString_srv))
    return matlab_GetString_srv.response.data;
  return "";
}

bool MatlabComm::getString(const char *name, char *str, int strLength)
{
  matlab_GetString_srv.request.name = name;
  if(handle_matlab_GetString.call(matlab_GetString_srv))
  {
    strcpy(str,matlab_GetString_srv.response.data.c_str());
    return true;
  }
  return false;
}
