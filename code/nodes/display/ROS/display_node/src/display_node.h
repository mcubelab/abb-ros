#include <stdio.h>
#include <time.h>
#include <vector>

#include <ros/ros.h>
#include <matVec/matVec.h>
#include <display_comm/display_comm.h>
#include <logger_comm/logger_comm.h>
#include <robot_comm/robot_comm.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <cv_bridge/cv_bridge.h>
#include <rosgraph_msgs/Log.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pqp/PQP.h>

#include <SFML/Window.hpp>
#include <SFML/System.hpp>
#include <SFML/Network.hpp>
#include <SFML/Graphics.hpp>

#include "fileio.h"

using namespace std;
//namespace enc = sensor_msgs::image_encodings;

#define MAX_BUFFER                 512
#define NUM_AUX_CONTAINERS         3     //Number of auxiliary containers 
#define NUM_TOPICS                 8     //Number of topics to display in auxiliary containers
#define HAND_CAM_SX                640   //Expected width in pixels of the images captures by the hand camera
#define HAND_CAM_SY                480   //Expected height in pixels of the images captures by the hand camera
#define PLACE_CAM_SX               640   //Expected width in pixels of the images captures by the place camera
#define PLACE_CAM_SY               480   //Expected height in pixels of the images captures by the place camera
#define KINECT_RGB_SX              640   //Expected width in pixels of the images captures by the kinect rgb
#define KINECT_RGB_SY              480   //Expected height in pixels of the images captures by the kinect rgb
#define KINECT_DEPTH_SX            640   //Expected width in pixels of the images captures by the kinect depth
#define KINECT_DEPTH_SY            480   //Expected height in pixels of the images captures by the kinect depth
#define TEXT_CONTAINER_WIDTH       600   //Fixed width of text container
#define TEXT_CONTAINER_MAX_LINES   13    //Fixed maximum number of lines to be used in text container 
#define ROSOUT_CONTAINER_MAX_LINES 34    //Fixed maximum number of lines to be used in the ROSout container
#define AUX_CONTAINER_X2Y_RATIO    1.33f //Fixed width to height ratio of auxiliary containers 
#define FIELD_OF_VIEW              40    //Aperture of OpenGL camera.
#define SKEW                       1.778 //Width to Height skew of OpenGl camera (optimzed for resolution of Display computer)
#define ZNEAR                      10    //Zmin of OpenGL camera (mm)
#define ZFAR                       10000 //Zmax of OpenGL camera (mm)
#define REFRESH_RATE               20    //Maximum refresh rate of Visualizer
#define TRAJ_TIME                  3.0   //Amount of trajectory to be displayed(s) 
#define NORMAL                     0000  //Object display normal mode
#define HIGHLIGHT                  0001  //Object display highlighted mode
#define INVISIBLE                  0002  //Object display invisible mode


//////////////////////
// Structs
//////////////////////
struct Container
{
 int x;         //x coordinate of the top left corner
 int y;         //y coordinate of the top left corner
 int width;     //width of the container.
 int height;    //height of the container.
 int topic;     //Topic currently displayed.
 bool isActive; //True when active.
};

struct RobotLocation
{
  Vec robotT;        //Position of TCP
  Quaternion robotQ; //Orientaion of TCP
  double time;       //Time when location drawn
};

struct Body 
{
  char fileName[MAX_BUFFER];   //Full path to WRL file with triangle mesh
  Vec pos;                     //Initial position of body. 
  Quaternion quat;             //Initial orientation of body.
  HomogTransf currPose;        //Current pose of body.
  double scale;                //Scale of body.
  Vec color;                   //Color
  int mode;                    //Mode to choose between NORMAL, HIGHLIGHT and INVISIBLE
  bool frame;                  //True if want to draw a local reference frame on the object.   
  Mat Vert;                    //Structure containing all vertices of triangle mesh 
  Mat Tri;                     //Structure containing indices to vertices composing all triangles of the tringle mesh.
  int glList;                  //OpenGL drawing list for fast drawing.
  PQP_Model pqp;               //PQP Model for collision detection.
  bool interact;               //True if object to be detected by mouse pointer. 
  int link;                    //Link it is attached to.
};

struct Link
{
  int linkId;            //Id of link.
  vector<int> idBodies;   //List of bodies attached to the link.
  Vec trans;             //Translation from origin to its nominal location in the workspace when angle = 0.0
  Vec rotAxis;           //Rotation from origin to its nominal location in the workspace when angle = 0.0.
  double angle;          //Current angle value.
  int prevLink;          //Previous link where attached.
};

struct World
{
  int nbodies;                  //Number of bodies
  int nLinks;                   //Number of links
  vector<int> idStaticBodies;           //List of ids of static bodies
  vector<Link> links;          //List of links
  int selectedBody;             //Body selected by mouse pointer. 
  int selectedTriangle;         //Id of triangle in "selectedBody" selected by mouse pointer.
  Vec mouseCamera;              //Position of mouse in camera coordinates.
  Vec mouseWorld;               //Position of mouse as projected to the bodies in the workspace.
  HomogTransf cameraMat;        //openGL camera.
  vector<Body> bodies;          //List of ll bodies in the world
};


World w;
vector<RobotLocation> robotTrajectory;
double robot_CartesianLog_values[7];
double robot_JointsLog_values[6];
double robot_ForceLog_values[6];
int hand_EncodersLog_values[4];
double hand_AnglesLog_values[4];
vector<string> rosoutContainerMsg;
vector<string> textContainerMsg;
bool readConfigFile(const char* configFile, const char* configFolder);
void createPQPModels();
double pointLineDistance(Vec p0, Vec p1, Vec p2);
Vec lineLineProjection(Vec p1, Vec p2, Vec p3, Vec p4);
Vec pointPlaneProjection(Vec p0, Vec p1, Vec n);
Vec linePlaneIntersection(Vec p1, Vec p2, Vec p3, Vec n);


/////////////////////////////
// Apearance and GUI Feedback
/////////////////////////////
double windowWidth2HeightRatio;
int currentWidth;
int currentHeight;
int originalWidth;
int originalHeight;
int previousMouseX;
int previousMouseY;
Vec motionCenter;
bool pressed;
HomogTransf initialOriginToCamera;
PQP_Model pointer;


//////////////////
// SFML Specific
//////////////////
sf::RenderWindow openGLwindow;
sf::Mutex worldMutex;
sf::Mutex loggerMutex;
sf::Mutex handCamMutex;
sf::Mutex placeCamMutex;
sf::Mutex kinectRGBMutex;
sf::Mutex kinectDepthMutex;
sf::Font font;
bool handleWindowEvent(sf::Event Event, const sf::Input& Input);
sf::Image logoRI;
  
//////////////////
// OPENGL Specific
//////////////////
void openGLInit();
void createDisplayLists();
void updateMouseOnCamera();
void updateMouseOnWorld();


///////////////////
// Drawing methods
///////////////////
void drawWorld();
void drawLinkHelper(HomogTransf Hom, int currLinkId);
void drawAxis(double radius, double length, double alpha=1.0);
void drawTrajectory();
void drawFloor();
void drawROSout();
void drawText();
bool drawHandCam(int containerId);
bool drawPlaceCam(int containerId);
bool drawMatLab(int containerId);
bool drawVision(int containerId);
bool drawKinectRGB(int containerId);
bool drawKinectDepth(int containerId);
bool drawHandState(int containerId);
bool drawInfo(int containerId);
bool drawContainer(int containerId);
void drawMouse();

//////////////////
// Camera capture 
//////////////////
bool handCamStarted;
bool placeCamStarted;
bool kinectRGBStarted;
bool kinectDepthStarted;
unsigned char handCamCapturedFrame[4*HAND_CAM_SX*HAND_CAM_SY];
unsigned char placeCamCapturedFrame[4*PLACE_CAM_SX*PLACE_CAM_SY];
unsigned char kinectRGBCapturedFrame[4*KINECT_RGB_SX*KINECT_DEPTH_SY];
unsigned char kinectDepthCapturedFrame[4*KINECT_DEPTH_SX*KINECT_DEPTH_SY];

//////////////////////
// Handling containers
//////////////////////
Container containerOpenGL;
Container containerROSout;
Container containerText;
Container containerAux[NUM_AUX_CONTAINERS];

void initContainers(int windowWidth, int windowHeight);
bool setContainer(int containerId, int topicId);
bool emptyContainer(int containerId);
bool toggleContainer(int containerId);


///////////////
// ROS Specific
///////////////
ros::NodeHandle* nodePtr;
ros::Subscriber handle_logger_SystemLog;

image_transport::Subscriber handle_hand_camera;
image_transport::Subscriber handle_place_camera;
image_transport::Subscriber handle_kinect_rgb;
ros::Subscriber handle_kinect_depth;
ros::Subscriber handle_rosout;
ros::ServiceServer handle_display_SetContainer;
ros::ServiceServer handle_display_Write;

void subscribeTopics();
void advertiseServices();
void logger_SystemLog_Callback(const logger_comm::logger_SystemLog& msg);
void handCam_Callback(const sensor_msgs::ImageConstPtr& msg);
void placeCam_Callback(const sensor_msgs::ImageConstPtr& msg);
void kinectRGB_Callback(const sensor_msgs::ImageConstPtr& msg);
void kinectDepth_Callback(const sensor_msgs::ImageConstPtr& msg);
//void kinectDepth_Callback(const stereo_msgs::DisparityImageConstPtr& msg);
void rosout_Callback(const rosgraph_msgs::Log::ConstPtr& msg);
bool display_Write(display_comm::display_Write::Request& req, display_comm::display_Write::Response& res);
bool display_SetContainer(display_comm::display_SetContainer::Request& req, display_comm::display_SetContainer::Response& res);
