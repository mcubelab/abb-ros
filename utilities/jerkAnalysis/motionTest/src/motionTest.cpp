#include <time.h>
#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>
#include <SFML/System.hpp>

#include <ABBInterpreter.h>
using namespace ABBInterpreter;

#define MAX_STRING 80
#define MAX_BUFFER 256
#define LOG_FOLDER "../../data"

#define ROBOT_PORT   5000
#define LOGGER_PORT   5001
#define ROBOT_IP "192.168.1.99"

//Work object
#define P_WORK_X 400.0
#define P_WORK_Y 0.0
#define P_WORK_Z 250.0
#define P_WORK_Q0 0.0
#define P_WORK_QX 0.0
#define P_WORK_QY 1.0
#define P_WORK_QZ 0.0

//Tool
#define P_TOOL_X  0.0
#define P_TOOL_Y  0.0
#define P_TOOL_Z  105.0
#define P_TOOL_Q0 1.0
#define P_TOOL_QX 0.0
#define P_TOOL_QY 0.0
#define P_TOOL_QZ 0.0

#define FLY_SPEED_TCP 75.0
#define FLY_SPEED_ORI 50.0 

#define SPEED_TCP 200.0
#define SPEED_ORI 50.0 

#define RATE 100.0 //Hz

sf::SocketTCP RobotClient;
double x;
double comx;
FILE *fLog;
bool logging;

#define N_INC 15
#define N_SPEED 15
#define N_DIST 15

const double INC[N_INC]={
  0.5000,
  0.6507,
  0.8469,
  1.1022,
  1.4345,
  1.8670,
  2.4298,
  3.1623,
  4.1156,
  5.3563,
  6.9711,
  9.0726,
  11.8077,
  15.3673,
  20.0000};

const double SPEED[N_SPEED]={
  3.0000,
  4.0495,
  5.4661,
  7.3783,
  9.9595,
  13.4437,
  18.1466,
  24.4949,
  33.0640,
  44.6307,
  60.2439,
  81.3191,
  109.7670,
  148.1668,
  200.0000};

const double DIST[N_DIST]={
  0.5000,
  0.6641,
  0.8819,
  1.1713,
  1.5556,
  2.0661,
  2.7440,
  3.6443,
  4.8401,
  6.4282,
  8.5374,
  11.3386,
  15.0589,
  20.0000,
  1000.0};
  
void feedbackLog(void* UserData)
{
  char strReceived[MAX_STRING];
  char buffer[MAX_STRING]; 

  std::size_t sizeReceived;
  sf::IPAddress loggerIP(ROBOT_IP); 
  sf::SocketTCP loggerClient;
  double time;

  int code;
  loggerClient.SetBlocking(true); 
  if (loggerClient.Connect(LOGGER_PORT, loggerIP) != sf::Socket::Done)
    {
      printf("There has been an error while connecting to the log server.\n");
      loggerClient.Close();
      exit(-1);
    }

  while(1)
    {
      if(loggerClient.Receive(strReceived,sizeof(strReceived),sizeReceived) == sf::Socket::Done)
	{
	  strReceived[sizeReceived]='\0';
	  sscanf(strReceived,"# %d",&code);
	  if(code==0) //We are only interested in messages with cartesian coordinates
	    {
	      sscanf(strReceived,"# %*d %*s %*s %lf %lf",&time, &x);
	      if(logging)
		{
		  sprintf(buffer,"%.2lf %.1lf %.1lf",time,x, comx);
		  fprintf(fLog,"\n%s",buffer);
		  //printf("%s\n",buffer);
		}
	    }
	}
      else
	printf("Problem with the Logger socket.\n");
    }
  loggerClient.Close();
}

int main(int argc, char** argv)
{
 //local variables
  char strReceived[MAX_STRING];
  char fileName[MAX_STRING];
  std::size_t sizeReceived;
  logging = false;

  ///////////////
  //Launch logger
  sf::Thread Log(&feedbackLog);
  Log.Launch();

  //////////////////////////////////////////////////////
  //Robot configuration
  RobotClient.SetBlocking(true); //hold the execution until we receive ACK from the server 	
  sf::IPAddress robotIp(ROBOT_IP); 
  if (RobotClient.Connect(ROBOT_PORT, robotIp) != sf::Socket::Done)
    {
      printf("There has been an error while connecting to the robot controller.\n");
      RobotClient.Close();
      exit(-1);
    }
	
  //Set the WorkObject
  RobotClient.Send(setWorkObject(P_WORK_X,
				 P_WORK_Y,
				 P_WORK_Z,
				 P_WORK_Q0,
				 P_WORK_QX,
				 P_WORK_QY,
				 P_WORK_QZ).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	
  //Set the tool
  RobotClient.Send(setTool(P_TOOL_X,
			   P_TOOL_Y,
			   P_TOOL_Z,
			   P_TOOL_Q0,
			   P_TOOL_QX,
			   P_TOOL_QY,
			   P_TOOL_QZ).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	
  //Set speed
  RobotClient.Send(setSpeed(FLY_SPEED_TCP,
			    FLY_SPEED_ORI).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  //Set Zone
  RobotClient.Send(setZone(1,0.0,0.0,.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);

  for (int i=1; i<2; i++)
    {
      for (int j=0; j<N_INC; j++)
	{
	  for (int k=0; k<N_DIST; k++)
	    {
	      //Set speed
	      RobotClient.Send(setSpeed(SPEED_TCP,SPEED_ORI).c_str(),MAX_STRING);
	      RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	      //Go Home
	      RobotClient.Send(setCartesian(0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
	      RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	      sf::Sleep(0.5);  

	      //Set Zone
	      RobotClient.Send(setZone(0,0.3,0.3,0.03).c_str(),MAX_STRING);
	      RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	      //Set speed
	      RobotClient.Send(setSpeed(SPEED[i],SPEED_ORI).c_str(),MAX_STRING);
	      RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);

	      double increment = INC[j];
	      double checkDist = DIST[k];
	      
	      comx = 0.0;
	      double xmin = -150.0;

	      //Open logging file
	      sprintf(fileName,"%s/Log_%02d_%02d_%02d.txt",LOG_FOLDER,i,j,k);
	      fLog = fopen(fileName,"w");
	      fprintf(fLog,"%.3lf %.3lf %.3lf",SPEED[i],INC[j],DIST[k]);

	      logging = true;
	      sf::Sleep(0.25);
	      while(x>xmin)
		{
		  //Wait until we are close enough to the commanded position
		  double dist = fabs(x-comx);
		  while(dist>checkDist)
		    {
		      dist = fabs(x-comx);
		      sf::Sleep(0.001);  
		    }
		  
		  //New commanded position
		  comx = comx - increment;
		  if(comx<xmin)
		    comx = xmin;
		  RobotClient.Send(setCartesian(comx,0,0,1,0,0,0).c_str(),MAX_STRING);
		  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
		  sf::Sleep(1.0/(double)RATE);  
		}
	      sf::Sleep(0.25);
	      logging = false;
	      sf::Sleep(0.25);
	      fclose(fLog);
	      printf("%.1lf - %.1lf - %.1lf DONE.\n",SPEED[i],INC[j],DIST[k]);
	      fflush(stdout);
	    }
	  
	}
    } 
  RobotClient.Close();
  Log.Terminate();
 
  return 0;
}
