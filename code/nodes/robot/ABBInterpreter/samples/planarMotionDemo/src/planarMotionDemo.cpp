#include <iostream>
#include <string>
#include <math.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/Network.hpp>
#include <SFML/System.hpp>
#include <matVec/matVec.h>
#include "../../../include/ABBInterpreter.h"

using namespace ABBInterpreter;

#define ROBOT_PORT   5000
#define LOGGER_PORT   5001
#define ROBOT_IP "192.168.1.99"
//#define ROBOT_IP "127.0.0.1"
//#define ROBOT_IP "192.168.180.128" //Virtual Machine

#define MAX_STRING 80
#define MAX_BUFFER 256

//Mouse interface
#define STEP              0.025 //interface Responsiveness
#define INTERFACE_WIDTH   600
#define INTERFACE_HEIGHT  600
#define MAX_DISPLACEMENT  150
sf::RenderWindow window;
sf::Sprite background;
void drawWorld();
bool handleWindowEvent(sf::Event Event, const sf::Input& Input);
double currentX;
double currentY;

double goToX = 0.0;
double goToY = 0.0;


//Robot connection
sf::SocketTCP RobotClient;

void feedbackLog(void* UserData)
{
  bool log;
  char strReceived[MAX_STRING];
  std::size_t sizeReceived;
  sf::IPAddress loggerIP(ROBOT_IP); 
  sf::SocketTCP loggerClient;
	
  int code;
  double x,y;
  loggerClient.SetBlocking(true); 
    if (loggerClient.Connect(LOGGER_PORT, loggerIP) != sf::Socket::Done)
      {
	printf("There has been an error while connecting to the log server.\n");
	loggerClient.Close();
	exit(-1);
      }

    log=true;
    while(log)
      {
	if(loggerClient.Receive(strReceived,sizeof(strReceived),sizeReceived) == sf::Socket::Done)
	  {
	    strReceived[sizeReceived]='\0';
	    sscanf(strReceived,"%d %*s %*s %*f %lf %lf",&code,&x,&y);
	    if(code==0) //We are only interested in messages with cartesian coordinates
	      {
		currentX = x;
		currentY = y;
	      }
	    if(code==1)
	      printf("%s\n",strReceived);
	  }
	else
	  printf("Problem with the Logger socket.\n");
      }
    loggerClient.Close();
}

int main(int argc, char **argv)
{
  //local variables
  char strReceived[MAX_STRING];
  std::size_t sizeReceived;

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
	
  //Definitions
  HomogTransf WorkObject(Quaternion("0.0 0.0 1.0 0.0").getRotMat(),Vec("500.0 0.0 500.0",3));
  HomogTransf Tool(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec("0.0 0.0 0.0",3));
  Vec Speed("100 30",2);

  //Set the WorkObject
  RobotClient.Send(setWorkObject(
				 WorkObject.getTranslation()[0],
				 WorkObject.getTranslation()[1],
				 WorkObject.getTranslation()[2],
				 WorkObject.getRotation().getQuaternion()[0],
				 WorkObject.getRotation().getQuaternion()[1],
				 WorkObject.getRotation().getQuaternion()[2],
				 WorkObject.getRotation().getQuaternion()[3]).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	
  //Set the tool
  RobotClient.Send(setTool(
			   Tool.getTranslation()[0],
			   Tool.getTranslation()[1],
			   Tool.getTranslation()[2],
			   Tool.getRotation().getQuaternion()[0],
			   Tool.getRotation().getQuaternion()[1],
			   Tool.getRotation().getQuaternion()[2],
			   Tool.getRotation().getQuaternion()[3]).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	
  //Set speed
  RobotClient.Send(setSpeed(Speed[0],Speed[1]).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	
  //Set  fly zone
  RobotClient.Send(setZone(0,0.3,0.03).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
  
  //Initialization of the Window Display
  window.Create(sf::VideoMode(INTERFACE_WIDTH, INTERFACE_HEIGHT, 32), "Robot Control Interface");
  window.SetPosition(10,10);
  const sf::Input& Input =  window.GetInput();
  sf::Event Event;
  background.Resize (INTERFACE_WIDTH,INTERFACE_HEIGHT);
  background.SetColor(sf::Color(255,255,255));
  //main loop
  RobotClient.Send(setCartesian(goToX, goToY, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
  RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);

  while(window.IsOpened())
    {
      while (window.GetEvent(Event))
	handleWindowEvent(Event, Input);
      drawWorld();
      window.Display();
      sf::Sleep(STEP);  
    }
 
  RobotClient.Close();
  Log.Terminate();
  return 0;
}

void drawWorld()
{
  int xPixel,yPixel;
  window.Draw(background);

  //current location of the Robot	
  yPixel = (int)((double)INTERFACE_WIDTH*(-currentX + MAX_DISPLACEMENT)/(2.0*MAX_DISPLACEMENT));
  xPixel = (int)((double)INTERFACE_HEIGHT*(currentY + MAX_DISPLACEMENT)/(2.0*MAX_DISPLACEMENT));
  sf::Shape Marker = sf::Shape::Circle(xPixel, yPixel, 5.0, sf::Color(255, 0, 0, 255));
  window.Draw(Marker);

  //target location of the Robot
  yPixel = (int)((double)INTERFACE_WIDTH*(-goToX + MAX_DISPLACEMENT)/(2.0*MAX_DISPLACEMENT));
  xPixel = (int)((double)INTERFACE_HEIGHT*(goToY + MAX_DISPLACEMENT)/(2.0*MAX_DISPLACEMENT));
 
  sf::Shape goToMarker = sf::Shape::Circle(xPixel, yPixel, 5.0, sf::Color(0, 0, 0, 255));
  window.Draw(goToMarker);
}

bool handleWindowEvent(sf::Event Event, const sf::Input& Input)
{
  Vec NonBlockingSpeed("500 150",2);
  Vec BlockingSpeed("100 30",2);

  char strReceived[MAX_STRING];
  std::size_t sizeReceived;
  switch (Event.Type)
    {
    case sf::Event::Closed:
      {
	window.Close();
	return 1;
      }
    case sf::Event::MouseButtonPressed:
      {
	if(Event.MouseButton.Button == sf::Mouse::Left)
	  {
	    int mouseX = Input.GetMouseX();
	    int mouseY = Input.GetMouseY();
	    goToY = 2.0*MAX_DISPLACEMENT*((double)mouseX/(double)INTERFACE_WIDTH)-MAX_DISPLACEMENT;
	    goToX = -(2.0*MAX_DISPLACEMENT*((double)mouseY/(double)INTERFACE_HEIGHT)-MAX_DISPLACEMENT);
	    RobotClient.Send(setCartesian(goToX, goToY, 0.0, 1.0, 0.0, 0.0, 0.0).c_str(),MAX_STRING);
	    RobotClient.Receive(strReceived,sizeof(strReceived),sizeReceived);
	  }
	return 1;
      }
    default:
      break;
    };
  return 0;
}
