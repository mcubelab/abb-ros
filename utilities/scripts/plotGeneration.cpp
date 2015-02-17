#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SFML/System.hpp>

#define MAX_BUFFER 256

#define SX          640
#define SY          480

sf::RenderWindow window;
	
void drawPlot(char *fileName, int expNumber);

int main()
{
  char fileName[MAX_BUFFER];
  window.Create(sf::VideoMode(SX, SY, 32), "Grasp Signature");

  for (int i=1;i<=200; i++)
    {
      sprintf(fileName,"../signatures/graspingMarkersP2_2010_05_28_signature%03d.txt",i);
      //Screenshot
      drawPlot(fileName,i);
      sf::Image Screen = window.Capture();
      sprintf(fileName,"../plots/graspingMarkersP2_2010_05_28_signature%03d.jpg",i);
      Screen.SaveToFile(fileName); 
      window.Display();
      //sf::Sleep(1.0);
    }
  return 1;
}

void drawPlot(char *fileName,int expNumber)
{
  char buffer[MAX_BUFFER];

  window.Clear(sf::Color(255, 255, 255));		
  sf::Shape rect = sf::Shape::Rectangle(25.0, 50.0, SX - 25.0, SY - 25.0, sf::Color(0,0,0,0), 3.0, sf::Color(0,0,0,255));
  window.Draw(rect);
  sprintf(buffer,"Grasping Markers 2010/05/08 -- grasp signature experiment %03d",expNumber);
  sf::String infoText(buffer);
  infoText.SetPosition(50.0f,15.0f);
  infoText.SetSize(18);
  infoText.SetColor(sf::Color(0, 0, 0));
  window.Draw(infoText);

  char line[MAX_BUFFER];
 
  FILE *f;			
  f = fopen(fileName,"r");
  bool begin= true;
  double oldTime, newTime;
  int oldMotor, newMotor;
  int oldEnc[4], newEnc[4];
  sf::Shape Line;
  while(fgets(line,MAX_BUFFER,f))
    {
      sscanf(line, "%lf %d %d %d %d %d",&newTime, &newMotor, &newEnc[0], &newEnc[1], &newEnc[2], &newEnc[3]);
      //rescaling of values
      newTime = 25.0 + (double)newTime*(SX - 2*25.0)/15.0;
      newMotor = 475 + (int)(newMotor/50.0);
      newEnc[0] = 450 + 30 - (int)(newEnc[0]/10.0);
      newEnc[1] = 450 + 25 - (int)(newEnc[1]/10.0);
      newEnc[2] = 450 - 20 -(int)(newEnc[2]/10.0);
      newEnc[3] = 450 - 40 - (int)(newEnc[3]/10.0);
      if(begin)
	begin = false;
      else
	{
	  Line = sf::Shape::Line(oldTime, oldMotor, newTime, newMotor, 2.0, sf::Color(0,0,0,255));
	  window.Draw(Line);
	  Line = sf::Shape::Line(oldTime, oldEnc[0], newTime, newEnc[0], 2.0, sf::Color(255,0,0,255));
	  window.Draw(Line);
	  Line = sf::Shape::Line(oldTime, oldEnc[1], newTime, newEnc[1], 2.0, sf::Color(255,0,0,255));
	  window.Draw(Line);
	  Line = sf::Shape::Line(oldTime, oldEnc[2], newTime, newEnc[2], 2.0, sf::Color(255,0,0,255));
	  window.Draw(Line);
	  Line = sf::Shape::Line(oldTime, oldEnc[3], newTime, newEnc[3], 2.0, sf::Color(255,0,0,255));
	  window.Draw(Line);
	}
      oldTime = newTime;
      oldMotor = newMotor;
      oldEnc[0] = newEnc[0];
      oldEnc[1] = newEnc[1];
      oldEnc[2] = newEnc[2];
      oldEnc[3] = newEnc[3];
    }
  fclose(f);
}
