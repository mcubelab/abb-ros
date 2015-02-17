#include "logger_comm.h"

LoggerComm::LoggerComm()
{
}

LoggerComm::LoggerComm(ros::NodeHandle* np)
{
  subscribe(np);
}

LoggerComm::~LoggerComm()
{
  shutdown();
}

void LoggerComm::subscribe(ros::NodeHandle* np)
{
  handle_logger_Start = 
    np->serviceClient<logger_comm::logger_Start>("logger_Start");
  handle_logger_Stop = 
    np->serviceClient<logger_comm::logger_Stop>("logger_Stop");
  handle_logger_Append = 
    np->serviceClient<logger_comm::logger_Append>("logger_Append");
  handle_logger_Create = 
    np->serviceClient<logger_comm::logger_Create>("logger_Create");
  handle_logger_Copy = 
    np->serviceClient<logger_comm::logger_Copy>("logger_Copy");

}

void LoggerComm::subscribeSystem(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const logger_comm::logger_SystemLogConstPtr&))
{
  logger_system_sub = np->subscribe("logger_SystemLog", q_len, funcPtr);  
}

void LoggerComm::shutdown()
{
  handle_logger_Start.shutdown();
  handle_logger_Stop.shutdown();
  handle_logger_Append.shutdown();
  handle_logger_Create.shutdown();
  handle_logger_Copy.shutdown();
}

bool LoggerComm::Start(string &filename, string folder, int id)
{
  logger_Start_srv.request.id = id;
  logger_Start_srv.request.folder = folder;
  int ret = handle_logger_Start.call(logger_Start_srv);
  filename = logger_Start_srv.response.filename;
  return ret;
}

bool LoggerComm::Start(string &filename, int id)
{
  return (Start(filename, ".", id));
}

bool LoggerComm::StartGrasp(string &filename, int id)
{
  return (Start(filename, "logGrasp", id));
}

bool LoggerComm::StartPlace(string &filename, int id)
{
  return (Start(filename, "logPlace", id));
}

bool LoggerComm::Start(int id)
{
  std::string aux;
  return(Start(aux,".", id));
}

bool LoggerComm::StartGrasp(int id)
{
  std::string aux;
  return(Start(aux,"logGrasp", id));
}

bool LoggerComm::StartPlace(int id)
{
  std::string aux;
  return(Start(aux,"logPlace", id));
}

bool LoggerComm::Stop()
{
  return handle_logger_Stop.call(logger_Stop_srv);
}

bool LoggerComm::Append(string filename, string info)
{
  logger_Append_srv.request.filename = filename;
  logger_Append_srv.request.info = info;
  return handle_logger_Append.call(logger_Append_srv);
}

bool LoggerComm::Create(string &filename)
{
  int ret = handle_logger_Create.call(logger_Create_srv);
  filename = logger_Create_srv.response.filename;
  return ret;
}

bool LoggerComm::Copy(string filename, string folder)
{
  logger_Copy_srv.request.filename = filename;
  logger_Copy_srv.request.folder = folder;
  return handle_logger_Copy.call(logger_Copy_srv);
}
