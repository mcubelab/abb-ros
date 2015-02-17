#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

#include <matVec.h>

//Defines
#define MAX_BUFFER 256

//Global variables
sf::RenderWindow window;

bool handleWindowEvent(sf::Event Event, const sf::Input& Input);

int main()
{
  char filename[MAX_BUFFER];
  char line[MAX_BUFFER];
  char file1[MAX_BUFFER];
  char file2[MAX_BUFFER];

  sf::Image captureImage;
  sf::Sprite capture;
  sf::Event Event;
  sf::Shape Circle;
  int pen1X, pen2X;
  int pen1Y, pen2Y;
  double angle;

  Vec pen(2);
  FILE* f;
  FILE* fComplete;
  int expId;
  int nMarkers;
  bool repeat;
  bool done;
  
  int motPos;
  int enc[4];
  
  f=fopen("../graspingMarkersP2_2010_05_28","r");
  fComplete=fopen("../graspingMarkersP2_2010_05_28_complete","w");
  window.Create(sf::VideoMode(1280, 960, 32), "Input Pen Pose");

  fgets(line, MAX_BUFFER,f);
  fgets(line, MAX_BUFFER,f);
  while (fgets(line, MAX_BUFFER,f)!=NULL)
    {
      repeat=true;
      while(repeat)
	{
      	  sscanf(line,"%d :: %d %d %d %d %d %d %s %s",&expId, &nMarkers,&motPos,&enc[0], &enc[1], &enc[2], &enc[3],file1, file2);
	  printf("%d %d\n",expId, nMarkers);
	  if(nMarkers == 1)
	    {
	      sprintf(filename,"../captures/graspingMarkersP2_2010_05_28_palm%03d.jpg",expId);
	      captureImage.LoadFromFile(filename);
	      capture.SetImage(captureImage);
	    
	      window.Draw(capture);
	      window.Display();	  
	      const sf::Input& Input =  window.GetInput();
	      
	      //We capture the first point of the pen
	      do
		  window.GetEvent(Event);
	      while(Event.Type!=sf::Event::MouseButtonReleased);
	      pen1X = Input.GetMouseX();
	      pen1Y = Input.GetMouseY();    
	      Circle = sf::Shape::Circle(pen1X, pen1Y, 2, sf::Color(255, 0, 0, 100), 1,sf::Color(255, 0, 0, 255));
	      Circle.SetBlendMode(sf::Blend::Alpha);
	      window.Draw(Circle);
	      window.Display();
	      sf::Sleep(0.5);
	      
	      //We capture the second point of the pen
	      do
		window.GetEvent(Event);
	      while(Event.Type!=sf::Event::MouseButtonReleased);
	      pen2X = Input.GetMouseX();
	      pen2Y = Input.GetMouseY();
	      Circle = sf::Shape::Circle(pen2X, pen2Y, 2, sf::Color(0, 255, 0, 100), 1,sf::Color(0, 255, 0, 255));
	      Circle.SetBlendMode(sf::Blend::Alpha);
	      window.Draw(Circle);
	      window.Display();
	      sf::Sleep(0.5);
	      
	      pen[0] = pen2X-pen1X;
	      pen[1] = pen2Y-pen1Y;	      
	      pen.normalize();
	      
	      
	      angle = 360.0*atan2(pen[1],pen[0])/(2*PI);
	      char text[100];
	      sprintf(text,"Pose = (%d, %d)   Angle = %5.1lf degrees. Next?", pen1X, pen1Y, angle);
	      sf::String infoText(text);
	      infoText.SetPosition(10.0f,0.0f);
	      infoText.SetSize(18);
	      infoText.SetColor(sf::Color(0, 0, 255));
	      window.Draw(infoText);
	      window.Display();
	      sf::Sleep(0.5);
	      done=false;
	      do
		{
		  window.GetEvent(Event);
		  if(Event.Type==sf::Event::KeyReleased)
		    {
		      if (Event.Key.Code==110)
			{
			  repeat=true;
			  done=true;
			}
		      if (Event.Key.Code==121)
			{
			  repeat=false;
			  done=true;
			}
		    }
		}
	      while(!done);
	    }
	  else
	    {
	      pen1X = 0;
	      pen1Y = 0;
	      angle = 0.0;
	      repeat=false;
	    }
	}
      fprintf(fComplete,"%03d %d %06d %04d %04d %04d %04d %03d %03d %6.2lf %s %s\n",expId,nMarkers,motPos,enc[0],enc[1],enc[2],enc[3],pen1X, pen1Y,angle,file1,file2);
    }
  fclose(f);
  fclose(fComplete);
  return 0;
}
