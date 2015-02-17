#include "display_node.h"

/**
  * Method to advertise ROS services offered by display_node.
  */
void advertiseServices()
{
  handle_display_SetContainer = nodePtr->advertiseService("display_SetContainer",display_SetContainer);
  handle_display_Write = nodePtr->advertiseService("display_Write",display_Write);
}


/**
  * Method to subscribe to ROS topics needed by display_node.
  */
void subscribeTopics()
{
  //Subscription to logger_SystemLog
  handle_logger_SystemLog = nodePtr -> subscribe("logger_SystemLog", 100, logger_SystemLog_Callback);
  //Subscription to rosout
  handle_rosout = nodePtr -> subscribe("/rosout", 1000, rosout_Callback);
  //Subscription to hand and place cameras
  image_transport::ImageTransport it_(*nodePtr);
  handle_hand_camera = it_.subscribe("/vis_hand/camera/image_raw", 1, handCam_Callback);
  handle_place_camera = it_.subscribe("/vis_place/camera/image_raw", 1, placeCam_Callback);
  handle_kinect_rgb = it_.subscribe("/camera/rgb/image_color", 1, kinectRGB_Callback);
  handle_kinect_depth = nodePtr->subscribe("/camera/depth_registered/image_raw", 1, kinectDepth_Callback);
}


/**
  * Callback to ROS service display_Write.
  * Sets input text to the specified line in text container.
  * @param req Service request.
  * @param res Service response.
  * @return True when acomplished.
  */
bool display_Write(display_comm::display_Write::Request& req,
                   display_comm::display_Write::Response& res)
{
  //Check input parameters
  if(req.line < 0 || req.line >= TEXT_CONTAINER_MAX_LINES)
  {  
    res.ret = 0;
    char aux[MAX_BUFFER];
    sprintf(aux,"DISPLAY_NODE: Wrong line number (0-%d).",TEXT_CONTAINER_MAX_LINES-1);
    res.msg = aux;
    return false;
  }
  else
  {
    if((int)textContainerMsg.size()<=(int)req.line)
      textContainerMsg.resize(req.line + 1);
    textContainerMsg[req.line] = req.s;
    res.ret = 1;
    char aux[MAX_BUFFER];
    sprintf(aux,"DISPLAY_NODE: Modified line %d.",(int)req.line);
    res.msg = aux;
    return true;
  }
}


/**
  * Callback to ROS service display_SetContainer.
  * Modifies the content of a container.
  * @param req Service request.
  * @param res Service response.
  * @return True when acomplished.
  */
bool display_SetContainer(display_comm::display_SetContainer::Request& req, 	
			  display_comm::display_SetContainer::Response& res)
{
  if(req.containerId < 0 || req.containerId >= NUM_AUX_CONTAINERS) 
  {
    res.ret = 0;
    char aux[MAX_BUFFER];
    sprintf(aux,"DISPLAY_NODE: Wrong container number (0-%d). Only auxiliary containers can be modified.", NUM_AUX_CONTAINERS-1);
    res.msg = aux;
    return false;
  }

  if(req.topicId < 0 || req.topicId >= NUM_TOPICS)
  {
    res.ret = 0;
    char aux[MAX_BUFFER];
    sprintf(aux,"DISPLAY_NODE: Wrong topic id (0-%d).",NUM_TOPICS-1);
    res.msg = aux;
    return false;
  }

  if(setContainer(req.containerId, req.topicId))
  {
    res.ret = 1;
    char aux[MAX_BUFFER];
    sprintf(aux,"DISPLAY_NODE: Auxiliary container %d set to topic %d.",(int)req.containerId, (int)req.topicId);
    res.msg = aux;
    return true;
  }
  res.ret = 0;
  res.msg = "DISPLAY_NODE: Problem setting the container.";
  return false;
}

bool setContainer(int containerId, int topicId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  if(topicId < 0 || topicId >= NUM_TOPICS)
    return false;
  containerAux[containerId].topic = topicId; 
  return true;
}

/**
*Assigns values for each of these properties from the logger_node's messages
*/
void logger_SystemLog_Callback(const logger_comm::logger_SystemLog& msg)
{
  loggerMutex.Lock();
  //Robot cartesian values
  robot_CartesianLog_values[0] = msg.x;
  robot_CartesianLog_values[1] = msg.y;
  robot_CartesianLog_values[2] = msg.z;
  robot_CartesianLog_values[3] = msg.q0;
  robot_CartesianLog_values[4] = msg.qx;
  robot_CartesianLog_values[5] = msg.qy;
  robot_CartesianLog_values[6] = msg.qz;

  //Robot joint values
  robot_JointsLog_values[0] = msg.j1;
  robot_JointsLog_values[1] = msg.j2;
  robot_JointsLog_values[2] = msg.j3;
  robot_JointsLog_values[3] = msg.j4;
  robot_JointsLog_values[4] = msg.j5;
  robot_JointsLog_values[5] = msg.j6;

  //Robot force values
  robot_ForceLog_values[0] = msg.fx;
  robot_ForceLog_values[1] = msg.fy;
  robot_ForceLog_values[2] = msg.fz;
  robot_ForceLog_values[3] = msg.tz;
  robot_ForceLog_values[4] = msg.ty;
  robot_ForceLog_values[5] = msg.tz;

  //Hand encoder values
  hand_EncodersLog_values[0] = msg.encMotor;
  hand_EncodersLog_values[1] = msg.encFinger[0];
  hand_EncodersLog_values[2] = msg.encFinger[1];
  hand_EncodersLog_values[3] = msg.encFinger[2];

  //Hand Angle values
  hand_AnglesLog_values[0] = msg.angleMotor;
  hand_AnglesLog_values[1] = msg.angle[0];
  hand_AnglesLog_values[2] = msg.angle[1];
  hand_AnglesLog_values[3] = msg.angle[2];
  loggerMutex.Unlock();

  //Set link angles
  worldMutex.Lock();
  w.links[0].angle = msg.j1;
  w.links[1].angle = msg.j2;
  w.links[2].angle = msg.j3;
  w.links[3].angle = msg.j4;
  w.links[4].angle = msg.j5;
  w.links[5].angle = msg.j6;
  w.links[6].angle = msg.angle[0];
  w.links[7].angle = msg.angle[1];
  w.links[8].angle = msg.angle[2];
  worldMutex.Unlock();
}

/**
  * Callback for subscription to ROS topic rosout.
  * Stores messages published to rosout.
  * @param msg Message published.
  */
void rosout_Callback(const rosgraph_msgs::Log::ConstPtr& msg)
{
    std::stringstream ss;
    switch (msg->level)
    {
    case rosgraph_msgs::Log::FATAL:
      ss << "FATAL ";
      break;
    case rosgraph_msgs::Log::ERROR:
      ss << "ERROR ";
      break;
    case rosgraph_msgs::Log::WARN:
      ss << "WARN ";
      break;
    case rosgraph_msgs::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rosgraph_msgs::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << msg->level << " ";
    }
    /*
    rosmsg.name = msg->name;
    rosmsg.line = msg->line;
    rosmsg.function = msg->function;
    rosmsg.file = msg->file;
    rosmsg.msg = msg->msg;*/
    //ss << "[" << msg->file << ":" << msg->line << "(" << msg->function << ") ";
    ss << "[" << msg->name << ":" << msg->line << "]: ";
    
    //ss << "[topics: ";
    /*
    std::vector<std::string>::const_iterator it = msg->topics.begin();
    std::vector<std::string>::const_iterator end = msg->topics.end();
    for ( ; it != end; ++it )
    {
      const std::string& topic = *it;

      if ( it != msg->topics.begin() )
      {
        rosmsg.topics.append( ", ");
      }

      rosmsg.topics.append(topic);
    }*/
    //ss << "] ";
    
    ss << msg->msg << "\n";

    rosoutContainerMsg.push_back(ss.str());
    //char* out = new char[ss.str().size()+1];
    //out[ss.str().size()] = 0;
    //memcpy(rosoutContainerMsg, ss.str().c_str(),ss.str().size());
}

/**
  * Callback for subscription to ROS topic encapsulating the hand camera.
  * Captures and stores the lastest frame.
  * @param msg Message published.
  */
void handCam_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!handCamStarted)
    handCamStarted = true;
  
  handCamMutex.Lock();
  unsigned char* ptrImage;
  unsigned char* ptrData;
  ptrImage = handCamCapturedFrame;
  ptrData = (unsigned char*)&msg->data[0];
  for (int i=0; i<HAND_CAM_SY; i++)
  {
    for(int j=0; j<HAND_CAM_SX; j++)
    {
      *ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = 255;
    }
  }
  handCamMutex.Unlock();
}

/**
  * Callback for subscription to ROS topic encapsulating the place camera.
  * Captures and stores the lastest frame.
  * @param msg Message published.
  */
void placeCam_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!placeCamStarted)
    placeCamStarted = true;

  placeCamMutex.Lock();
  unsigned char* ptrImage;
  unsigned char* ptrData;
  ptrImage = placeCamCapturedFrame;
  ptrData = (unsigned char*)&msg->data[0];
  for (int i=0; i<PLACE_CAM_SY; i++)
  {
    for(int j=0; j<PLACE_CAM_SX; j++)
    {
      *ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = 255;
    }
  }
  placeCamMutex.Unlock();
}



/**
  * Callback for subscription to ROS topic encapsulating the kinect rgb.
  * Captures and stores the lastest frame.
  * @param msg Message published.
  */
void kinectRGB_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!kinectRGBStarted)
    kinectRGBStarted = true;
  
  kinectRGBMutex.Lock();
  unsigned char* ptrImage;
  unsigned char* ptrData;
  ptrImage = kinectRGBCapturedFrame;
  ptrData = (unsigned char*)&msg->data[0];
  for (int i=0; i<KINECT_RGB_SY; i++)
  {
    for(int j=0; j<KINECT_RGB_SX; j++)
    {
			*ptrImage++ = ptrData[2];
			*ptrImage++ = ptrData[1];
			*ptrImage++ = ptrData[0];
			*ptrImage++ = 255;
			ptrData += 3;
      /*
			*ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = *ptrData++;
      *ptrImage++ = 255;
			*/
    }
  }
  kinectRGBMutex.Unlock();
}


/**
  * Callback for subscription to ROS topic encapsulating the kinect depth.
  * Captures and stores the lastest frame.
  * @param msg Message published.
  */
//void kinectDepth_Callback(const stereo_msgs::DisparityImageConstPtr& msg)
void kinectDepth_Callback(const sensor_msgs::ImageConstPtr& msg)
{
  if(!kinectDepthStarted)
    kinectDepthStarted = true;
  
  kinectDepthMutex.Lock();
  unsigned char* ptrImage;
  unsigned char* ptrData;
  ptrImage = kinectDepthCapturedFrame;
  ptrData = (unsigned char*)&msg->data[0];
  for (int i=0; i<KINECT_DEPTH_SY; i++)
  {
    for(int j=0; j<KINECT_DEPTH_SX; j++)
    {
      *ptrImage++ = ptrData[0];
      *ptrImage++ = ptrData[0];
      *ptrImage++ = ptrData[0];
      *ptrImage++ = 255;
			ptrData+=2;
    }
  }
  kinectDepthMutex.Unlock();
}



/**
  * Method to initialize the openGL environment.
  */
void openGLInit()
{ 
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glClearDepth(1);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);	
  glShadeModel(GL_SMOOTH);
  glEnable(GL_COLOR_MATERIAL);
  glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glEnable(GL_POINT_SMOOTH);
  glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); 
 
  glMatrixMode(GL_PROJECTION);
  gluPerspective(FIELD_OF_VIEW, SKEW, ZNEAR, ZFAR);
  glMatrixMode(GL_MODELVIEW); 
  gluLookAt(1800.0, -1500.0, 800.0, 500.0, 300.0, 100.0, 0.0f, 0.0f, 1.0f);
  GLdouble modelMatrix[16];
  glGetDoublev(GL_MODELVIEW_MATRIX, modelMatrix);
  w.cameraMat = HomogTransf(&modelMatrix[0]).transp();

  //Lightning
  GLfloat lightAmbienColor[] = {0.1, 0.1, 0.1, 0.75};
  GLfloat lightDiffuseColor[] = {0.6, 0.6, 0.6, 0.75};
  GLfloat lightSpecularColor[] = {0.25, 0.25, 0.25, 0.75};
  GLfloat lightPosition[] = {1800.0, -2000.0, 800.0, 1.0};

  glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbienColor);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuseColor);
  glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecularColor);
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 10);
}


//CHANGE
void createDisplayLists()
{
  vector<Body>::iterator i;
  vector<Link>::iterator j;
  int k;
  Vec vertex0, vertex1, vertex2;
  Vec triangle, normal;

  for (i=w.bodies.begin(); i<w.bodies.end(); i++)
  {
    i->glList = ::glGenLists(1); // ask for a free id number 
    glNewList(i->glList,GL_COMPILE_AND_EXECUTE); 
    glBegin(GL_TRIANGLES); 
    for(k=0; k<(i->Tri).nn; k++)
    {
      triangle = (i->Tri).getRow(k);
      vertex0 = (i->Vert).getRow((int)triangle[0]);
      vertex1 = (i->Vert).getRow((int)triangle[1]);
      vertex2 = (i->Vert).getRow((int)triangle[2]);
      normal = (vertex1-vertex0)^(vertex2-vertex0);
      normal.normalize();
      glNormal3d(normal[0],normal[1],normal[2]);
      glVertex3d(vertex0[0],vertex0[1],vertex0[2]);
      glVertex3d(vertex1[0],vertex1[1],vertex1[2]);
      glVertex3d(vertex2[0],vertex2[1],vertex2[2]);
    }
    glEnd(); 
    glEndList();

  }
}

/**
  * Helper function to draw a body in the current openGL coordinate frame.
  * @param bodyToDraw Body to draw.
  */
void drawBodyHelper(Body &bodyToDraw)
{
 /* Coloring and Effects */ 
  GLfloat colorAmbientAndDiffuse[4] = {0.752941, 0.752941, 0.752941, 1.0};
  GLfloat colorEmission[4] = { 0.060235, 0.060235, 0.060235, 1.0};
  GLfloat colorSpecular[4] = { 0.715294, 0.715294, 0.715294, 1.0};
  GLfloat colorShininess = 0.5500;
  glEnable(GL_LIGHTING);
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, colorAmbientAndDiffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, colorEmission);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, colorSpecular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, &colorShininess);

  switch(bodyToDraw.mode)
  {
    case HIGHLIGHT:
    {
      //Translucid
      glColor4f((bodyToDraw.color)[0], (bodyToDraw.color)[1], (bodyToDraw.color)[2],0.1);	
      glCallList(bodyToDraw.glList);
      break;
    }
    case NORMAL:
    {
      //Opaque
      glColor4f((bodyToDraw.color)[0], (bodyToDraw.color)[1], (bodyToDraw.color)[2],1.0);	
      glCallList(bodyToDraw.glList);   
      break;
    }
    case INVISIBLE:
    {
      //No display
      break;
    }
  };
  if(bodyToDraw.frame)
    drawAxis(5,100.0);		//draw an axis for each body that has frame = true
}

/**
  * Helper function to draw a complete link and all forawrdly attached links.
  * @param Hom Acumulated homogeneous transformation where to draw the link.
  */
void drawLinkHelper(HomogTransf Hom, int currLinkId)
{
  //TODO
  if(currLinkId >= (int)w.links.size()) 
	{
	/*
		double const r[3] = {0.0, 0.0, 0.0};
		double const t[3] = {0.0, 0.0, 105.0};
		Vec offset = Vec("0.0 0.0 105.0",3);
		HomogTransf Trans = HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(), offset);

		Hom = Hom*Trans;
		glPushMatrix();
		glMultMatrixd(&Hom.transp()[0][0]);
		glEnable(GL_LIGHTING);
		drawAxis(15,100.0);
		glDisable(GL_LIGHTING);
		glPopMatrix();*/
		return;
	}
   int j;
 
  glPushMatrix();
  RotMat originRot;
  Vec zAxis = Vec("0.0 0.0 1.0",3);
  Vec v = zAxis + w.links[currLinkId].rotAxis;
  v.normalize();
  HomogTransf GoOrigin = HomogTransf(Quaternion(v*w.links[currLinkId].rotAxis, v^w.links[currLinkId].rotAxis).getRotMat(), w.links[currLinkId].trans); 
  RotMat rot;
  worldMutex.Lock();
  rot.rotZ(w.links[currLinkId].angle*DEG2RAD);
  worldMutex.Unlock();
  HomogTransf DoRot = HomogTransf(rot, Vec("0.0 0.0 0.0", 3));
  Hom = Hom*GoOrigin*DoRot*GoOrigin.inv();
  glMultMatrixd(&Hom.transp()[0][0]);
  
  //draw all bodies attached to this link
  vector<int>::iterator i;
  for(i=w.links[currLinkId].idBodies.begin(); i<w.links[currLinkId].idBodies.end(); i++)
  {
    worldMutex.Lock();
    w.bodies[*i].currPose = Hom;
    worldMutex.Unlock();
    drawBodyHelper(w.bodies[*i]);
  }
  glPopMatrix();
  
  //Draw all forwardly attached links
  for(j=0; j<(int)w.links.size(); j++)
  {
   if(w.links[j].prevLink == currLinkId)
      drawLinkHelper(Hom,j);
  }
}

/**
  * Draw the OpenGL world.
  */
void drawWorld()
{
   Container c = containerOpenGL;

  //3D Viewport
  //Viewport of OpenGL container
  // glViewport needs the coords of the lower left corner and size of the viewport 
  double ratioX = (double)currentWidth/(double)originalWidth;
  double ratioY = (double)currentHeight/(double)originalHeight;
  glViewport((int)(c.x*ratioX),
             (int)(currentHeight - (c.y + c.height)*ratioY),
             (int)(c.width*ratioX),
             (int)(c.height*ratioY));
  
  glClearColor(0.255,0.412,0.667, 1.0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
 
  //Draw environment 
  glMatrixMode(GL_MODELVIEW); 
  worldMutex.Lock();
  glLoadMatrixd(&(w.cameraMat.transp())[0][0]);
  worldMutex.Unlock();
  glEnable(GL_LIGHTING);
  drawAxis(5,350.0);         //central axis
  glDisable(GL_LIGHTING);
  drawFloor();

  //Small set of axis
  glDisable(GL_LIGHTING);
  worldMutex.Lock();
  HomogTransf OriginToCamera = w.cameraMat.inv();
  worldMutex.Unlock();
  Vec xCam = OriginToCamera.getRotation().getCol(0);
  Vec yCam = OriginToCamera.getRotation().getCol(1);
  Vec zCam = OriginToCamera.getRotation().getCol(2);
  Vec pos = OriginToCamera.getTranslation();
  double sizeScreenY = 2*ZNEAR*tan(FIELD_OF_VIEW*PI/(2*180.0));
  double sizeScreenX = 2*ZNEAR*SKEW*tan(FIELD_OF_VIEW*PI/(2*180.0));
  Vec trans = pos - zCam*ZNEAR*2 - yCam*sizeScreenY*0.85 - xCam*sizeScreenX*0.85;
  HomogTransf Transp = HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),trans).transp();
  glPushMatrix();	
  glMultMatrixd(&Transp[0][0]);
  drawAxis(0.05,1.0);		//bottom left axis - follows camera
  glPopMatrix();

  //Draw Static bodies
  vector<int>::iterator i;
  Vec transVec = Vec("0.0 0.0 0.0", 3);
  RotMat rot = Quaternion("1.0 0.0 0.0 0.0").getRotMat();
  HomogTransf Hom = HomogTransf(rot, transVec);
  for (i=w.idStaticBodies.begin(); i<w.idStaticBodies.end(); i++)
  {
    worldMutex.Lock();
    w.bodies[*i].currPose = Hom;
    worldMutex.Unlock();
    drawBodyHelper(w.bodies[*i]);
  }
  
  //Draw Links bodies
  if(w.links.size() > 0)
    drawLinkHelper(Hom, w.links.front().linkId);
  
  //Draw the last TRAJ_TIME seconds of the trajectory
  drawTrajectory();
}

/**
  * Method to update and draw the last TRAJ_TIME seconds of the trajectory of the TCP.
  * Obtains the TCP from robot_CartesianLog_values.
  * Also draws a reference frame at the beginnign and end of the trajectory.
  */
void drawTrajectory()
{
 //Read the current cartesian position of the robot.
 loggerMutex.Lock();
 Quaternion robotQ(&robot_CartesianLog_values[3]);
 Vec robotT(&robot_CartesianLog_values[0],3);
 loggerMutex.Unlock();
 
 //Update the trajectory of the robot with the current location and time.
 RobotLocation newLocation;
 newLocation.robotT = robotT;
 newLocation.robotQ = robotQ;
 struct timeval now;
 gettimeofday(&now, NULL);
 double tnow = now.tv_sec + (now.tv_usec/1000000.0);
 newLocation.time = tnow;
 robotTrajectory.push_back(newLocation);

 //Trim trajectory to the last TRAJ_TIME seconds
 vector<RobotLocation>::iterator j;
 for (j = robotTrajectory.begin(); j<robotTrajectory.end(); j++)
  {
    if(tnow - (j->time) > TRAJ_TIME)
      robotTrajectory.erase(j);
 }

  //Draw a sphere on the current location of the TCP
  GLUquadricObj *sphere;
  sphere = gluNewQuadric();
  gluQuadricOrientation(sphere,GLU_OUTSIDE);
  glColor3f(1.0/3.0, 0.0, 2.0/3.0);
  glPushMatrix();	
  glTranslated((robotTrajectory.end()-1)->robotT[0],
               (robotTrajectory.end()-1)->robotT[1],
               (robotTrajectory.end()-1)->robotT[2]);
  gluSphere(sphere,4,5,5);
  glPopMatrix();

  //Draw trajectory
  GLUquadricObj *cylinder;
  cylinder = gluNewQuadric();
  gluQuadricOrientation(cylinder,GLU_OUTSIDE);
  for (j = robotTrajectory.begin(); j<(robotTrajectory.end()-1); j++)
  {
    //Draw spheres at locations
    glPushMatrix();	
    glTranslated(j->robotT[0],j->robotT[1],j->robotT[2]);
    gluSphere(sphere,4,5,5);
    glPopMatrix();

    //Draw cylinders connecting them
    Vec z = Vec("0.0 0.0 1.0",3);
    Vec newZ = (((j+1)->robotT) - (j->robotT));
    newZ.normalize(); 
    Vec v = z + newZ;
    v.normalize();
    HomogTransf transformation = HomogTransf(Quaternion(v*newZ, v^newZ).getRotMat(), j->robotT).transp(); 
    glPushMatrix();	
    glMultMatrixd(&transformation[0][0]);
    gluCylinder(cylinder,4,4,(((j+1)->robotT) - (j->robotT)).norm(),5,1);
    glPopMatrix();
 }

  //Draw axis at the end of the trajectory
  HomogTransf Transp;
  Transp = HomogTransf((robotTrajectory.end()-1)->robotQ.getRotMat(),(robotTrajectory.end()-1)->robotT).transp();
  glPushMatrix();	
  glMultMatrixd(&Transp[0][0]);
  glEnable(GL_LIGHTING);
  drawAxis(2,75.0);
  glDisable(GL_LIGHTING);		
  glPopMatrix();

  //Draw axis at the beginnign of the trajectory
  //Transp = HomogTransf(robotTrajectory.begin()->robotQ.getRotMat(),robotTrajectory.begin()->robotT).transp();
  //glPushMatrix();	
  //glMultMatrixd(&Transp[0][0]);
  //glEnable(GL_LIGHTING);
  //drawAxis(2,75.0, 0.5);
  //glDisable(GL_LIGHTING);		
  //glPopMatrix();
}

/**
  * Method to draw a grid on top of the table (z=0).
  * The table size if 48in x 48in.
  * The grid size if 2in x 2in.
  */
void drawFloor()
{
  float x, y;
  float xmin = 0.0;
  float xmax = 48.0 * 25.4;
  float ymin = 0.0;
  float ymax = 48.0 * 25.4;
  float step = 25.4*2;

  glLineWidth(2.0);
  glBegin(GL_LINES);
  glColor3f(0.0, 0.0, 0.0);
  for (x = xmin; x <= xmax; x += step)
    {
      glVertex3f(x,ymin,0.5);
      glVertex3f(x,ymax,0.5);
    }
  for (y = ymin; y <= ymax; y += step)
    {
      glVertex3f(xmin,y,0.5);
      glVertex3f(xmax,y,0.5);
    }
  glEnd();
}

/**
  * Method to draw a set of axis in the current openGL set of corrdinates.
  * Color scheme: x-red, y-green, z-white.
  * @param radius Radius of the cylinder drawn for axis.
  * @param lenght Lenght of axis.
  * @param alpha Transparency of the set of axis (optional, default value = 1.0). 
  */
void drawAxis(double radius, double length, double alpha)
{
  GLUquadricObj *cylinder;
  GLUquadricObj *cone;
  cylinder = gluNewQuadric();
  cone = gluNewQuadric();
  gluQuadricOrientation(cylinder,GLU_OUTSIDE);
  gluQuadricOrientation(cone,GLU_OUTSIDE);
  
  glPushMatrix();	
  
  // Draw axis Z
  glColor4f(1.0, 1.0, 1.0, alpha);
  gluCylinder(cylinder,radius,radius,length,10,1);
  glTranslated(0,0,length);
  gluCylinder(cone, 2*radius, 0, 5*radius, 10, 1);
  glTranslated(0,0,-length);
  
  // Draw axis Y
  glRotated(90,-1,0,0);
  glColor4f(0.0, 1.0, 0.0, alpha);
  gluCylinder(cylinder,radius,radius,length,10,1);
  glTranslated(0,0,length);
  gluCylinder(cone, 2*radius, 0, 5*radius, 10, 1);
  glTranslated(0,0,-length);
  glRotated(90,1,0,0);
  
  //Axis X	
  glRotated(90,0,1,0);
  glColor4f(1.0, 0.0, 0.0, alpha);
  gluCylinder(cylinder,radius,radius,length,10,1);
  glTranslated(0,0,length);
  gluCylinder(cone, 2*radius, 0, 5*radius, 10, 1);
  glTranslated(0,0,-length);
  glRotated(90,0,-1,0);

  glPopMatrix();
}


/**
  * Method to draw the text of the text container.
  */
void drawText()
{
  Container c = containerText;
  int i;
  int j =0;
  //sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(0.255*255,0.412*255,0.667*255,255));
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  openGLwindow.Draw(rect);

  for (i=0; i<(int)textContainerMsg.size() && i<TEXT_CONTAINER_MAX_LINES; i++)
  {
    string s = textContainerMsg[i];
    int index;
    size_t len = s.length();
    int k=0;
    while(len * 12 > TEXT_CONTAINER_WIDTH)
    {
      index = TEXT_CONTAINER_WIDTH/12;
      s.insert(index*(k+1)-1, "\n    ");
      len -= index;
      
      k++;
    }
    j++;
 
    sf::String msgText(s);
    msgText.SetFont(font);
    msgText.SetStyle(1);
    msgText.SetPosition(c.x + 10.0f,c.y + 10.0f + (20 * j));
    j += k;
    msgText.SetSize(20);
    msgText.SetColor(sf::Color(0, 0, 0));
    openGLwindow.Draw(msgText);
  }
  sf::String title("Manipulation Lab -- Carnegie Mellon University");
  title.SetFont(font);
  title.SetStyle(3);
  title.SetPosition(c.x + 45.0f,containerText.y + containerText.height - 24.0f);
  title.SetSize(16);
  title.SetColor(sf::Color(0, 0, 0));
  openGLwindow.Draw(title);

  //Draw RI logo
  double width = 60;
  sf::Sprite logoRISprite;
  logoRISprite.SetImage(logoRI);
  logoRISprite.Resize(width, width*logoRISprite.GetSize().y/logoRISprite.GetSize().x);
  logoRISprite.SetPosition(c.x + c.width - logoRISprite.GetSize().x - 5.0f,
                     c.y + c.height - logoRISprite.GetSize().y - 5.0f);
  openGLwindow.Draw(logoRISprite);
}

/**
  * Method to draw the last image captured by the grasping camera.
  * The expected resolution is HAND_CAM_SX x HAND_CAM_SY (640x480)
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawHandCam(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS)
    return false;
  if(handCamStarted)
  {
    sf::Image handImg;
    sf::Sprite handSprite;
    handCamMutex.Lock();
    handImg.LoadFromPixels(HAND_CAM_SX,HAND_CAM_SY,handCamCapturedFrame);
    handSprite.SetImage(handImg);
    handCamMutex.Unlock();
    handSprite.Resize(containerAux[containerId].width,containerAux[containerId].height);
    handSprite.SetPosition(containerAux[containerId].x,containerAux[containerId].y);
    openGLwindow.Draw(handSprite); 
  }
  else
    emptyContainer(containerId);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "HAND CAMERA");
  sf::String titleText(title);
  titleText.SetPosition(containerAux[containerId].x + 8.0f, containerAux[containerId].y + containerAux[containerId].height - 24.0f);
  titleText.SetSize(16);
  if(handCamStarted)
    titleText.SetColor(sf::Color(255, 255, 255));
  else
    titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);

  return true;
}

/**
  * Method to draw the last image captured by the placing camera.
  * The expected resolution is PLACE_CAM_SX x PLACE_CAM_SY (640x480)
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawPlaceCam(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  if(placeCamStarted)
  {
    sf::Image placeImg;
    sf::Sprite placeSprite;
    placeCamMutex.Lock();
    placeImg.LoadFromPixels(PLACE_CAM_SX,PLACE_CAM_SY,placeCamCapturedFrame);
    placeSprite.SetImage(placeImg);
    placeCamMutex.Unlock();
    placeSprite.Resize(containerAux[containerId].width,containerAux[containerId].height);
    placeSprite.SetPosition(containerAux[containerId].x,containerAux[containerId].y);
    openGLwindow.Draw(placeSprite);
  }
  else
   emptyContainer(containerId);
  
  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "PLACE CAMERA");
  sf::String titleText(title);
  titleText.SetPosition(containerAux[containerId].x + 8.0f, containerAux[containerId].y + containerAux[containerId].height - 24.0f);
  titleText.SetSize(16);
  if(placeCamStarted)
    titleText.SetColor(sf::Color(255, 255, 255));
  else
    titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);

  return true;
}

/**
  * Method to draw ros_out topic.
  * @param containerId Id of the container where to draw.
  */
void drawROSout()
{
  Container c = containerROSout;
  int i;
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(0,0,0,255));
  openGLwindow.Draw(rect);

  //First we should count how many lines we need to display
  //This will tell us whether to start displaying from the top or the bottom.
  int totalLines=0;
  int charactersPerLine = (int)(TEXT_CONTAINER_WIDTH/12)-1;
  for (i=0; i<(int)rosoutContainerMsg.size(); i++)
  {
    string s = rosoutContainerMsg[i];
    size_t len = s.length();
    int partialLines=1;
    while(len * 12 > TEXT_CONTAINER_WIDTH)
    {
      s.insert(charactersPerLine*partialLines - 1, "\n     ");
      len = len - charactersPerLine + 5;
      partialLines++;
    }
    totalLines += partialLines;
  }

  //Now we draw the text
  if(totalLines <= ROSOUT_CONTAINER_MAX_LINES)
  {
    //We begin to write text from the top
    totalLines=0;
    for (i=0; i<(int)rosoutContainerMsg.size(); i++)
    {
      string s = rosoutContainerMsg[i];
      string level = s.substr(0,5);
      size_t len = s.length();
      int partialLines=1;
      while(len * 12 > TEXT_CONTAINER_WIDTH)
      {
        s.insert(charactersPerLine*partialLines - 1, "\n     ");
        len = len - charactersPerLine + 5;
        partialLines++;
      }
      sf::String rosText(s);
      rosText.SetFont(font);
      rosText.SetStyle(1);
      rosText.SetPosition(c.x + 10.0f,c.y + 15.0f + (20 * totalLines));
      rosText.SetSize(20);
      totalLines += partialLines;
      if(level.find("FATAL") != string::npos) rosText.SetColor(sf::Color(255, 255, 255));
      else if(level.find("ERROR") != string::npos) rosText.SetColor(sf::Color(255, 0, 0));
      else if(level.find("WARN") != string::npos) rosText.SetColor(sf::Color(255, 255, 0));
      else if(level.find("DEBUG") != string::npos) rosText.SetColor(sf::Color(100, 100, 100));
      else if(level.find("INFO") != string::npos) rosText.SetColor(sf::Color(255, 255, 255));
      else rosText.SetColor(sf::Color(255, 255, 255));
      openGLwindow.Draw(rosText);
    }
  }
  else
  {
    //We begin to write text from the bottom
    totalLines=0;
    for (i=rosoutContainerMsg.size()-1; totalLines<ROSOUT_CONTAINER_MAX_LINES; i--)
    {
      string s = rosoutContainerMsg[i];
      string level = s.substr(0,5);
      size_t len = s.length();
      int partialLines = 1;
      while(len * 12 > TEXT_CONTAINER_WIDTH)
      {
        s.insert(charactersPerLine*partialLines - 1, "\n     ");
        len = len - charactersPerLine + 5;
        partialLines++;
      }
      if(totalLines + partialLines >ROSOUT_CONTAINER_MAX_LINES)
      {
        //we need to trim the string s
        int iniPos = 0;
        for( int j=0; j < totalLines + partialLines - ROSOUT_CONTAINER_MAX_LINES ; j++)
          iniPos = s.find("\n", iniPos) + 1;
        s = s.substr(iniPos,s.length());
        totalLines = ROSOUT_CONTAINER_MAX_LINES;
      }
      else
        totalLines += partialLines;
      sf::String rosText(s);
      rosText.SetFont(font);
      rosText.SetStyle(1);
      rosText.SetPosition(c.x + 10.0f,c.y + 15.0f + (20 * (ROSOUT_CONTAINER_MAX_LINES - totalLines)));
      rosText.SetSize(20);
      if(level.find("FATAL") != string::npos) rosText.SetColor(sf::Color(255, 255, 255));
      else if(level.find("ERROR") != string::npos) rosText.SetColor(sf::Color(255, 0, 0));
      else if(level.find("WARN") != string::npos) rosText.SetColor(sf::Color(255, 255, 0));
      else if(level.find("DEBUG") != string::npos) rosText.SetColor(sf::Color(100, 100, 100));
      else if(level.find("INFO") != string::npos) rosText.SetColor(sf::Color(255, 255, 255));
      else rosText.SetColor(sf::Color(255, 255, 255));
      openGLwindow.Draw(rosText);
    }

  }
}


/**
  * Method to draw a visualization of the hand state.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawHandState(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;

  Container c = containerAux[containerId];
  
  //Draw white background	
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  openGLwindow.Draw(rect);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "HAND STATE");
  sf::String titleText(title);
  titleText.SetPosition(c.x + 8.0f, c.y + c.height - 24.0f);
  titleText.SetSize(16);
  titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);

  //Read hand state
  double fingerAngle[3];
  double motorAngle;
  loggerMutex.Lock();
  motorAngle = hand_AnglesLog_values[0];
  fingerAngle[0] = hand_AnglesLog_values[1];
  fingerAngle[1] = hand_AnglesLog_values[2];
  fingerAngle[2] = hand_AnglesLog_values[3];
  loggerMutex.Unlock();
  
  //Draw bars
  int num_bars = 3;
  double maxAngle = 130.0;
  double minAngle = -30.0; 
  double borderWidth = 40; 
  double barWidth = (c.width - 4*borderWidth)/num_bars;
  for (int i=0; i<num_bars; i++)
  {
    //Dimensions of bar
    double left = c.x + borderWidth*(i+1) + barWidth*i;
    double right = left + barWidth;
    double top = c.y + borderWidth;
    double bottom = c.y + c.height - borderWidth;
    
    //Draw contour of bar
    sf::Shape contour = sf::Shape::Rectangle(left - 3,
                                            top - 3, 
                                            right + 3, 
                                            bottom + 3, 
                                            sf::Color(0, 0, 0, 0),
                                            2,
                                            sf::Color(0, 0, 0, 255));
    openGLwindow.Draw(contour);
    
    //Draw motor bar
    double heightMotor = bottom - (motorAngle - minAngle)*((bottom-top)/(maxAngle-minAngle));
    sf::Shape barMotor = sf::Shape::Rectangle(left, heightMotor, right, bottom, sf::Color(100,100,100));
    sf::Shape lineMotor = sf::Shape::Line(left, heightMotor, right, heightMotor,2,sf::Color(0,0,0,255));
    openGLwindow.Draw(barMotor);
 
    //Draw finger bar
    sf::Shape fingerBar;
    double heightFinger = bottom - (fingerAngle[i] - minAngle)*((bottom-top)/(maxAngle-minAngle));
    if (motorAngle < fingerAngle[i])
      fingerBar = sf::Shape::Rectangle(left, heightFinger, right, heightMotor, sf::Color(0,0,255));
    else
      fingerBar = sf::Shape::Rectangle(left, heightFinger, right, heightMotor, sf::Color(255,0,0));
    openGLwindow.Draw(fingerBar);
    openGLwindow.Draw(lineMotor);

    //Draw ticks
    double heightTick1 = bottom - (0.0 - minAngle)*((bottom-top)/(maxAngle-minAngle));
    double heightTick2 = bottom - (100.0 - minAngle)*((bottom-top)/(maxAngle-minAngle));
    sf::Shape tick1 = sf::Shape::Line(left-8, heightTick1, left-3, heightTick1,2,sf::Color(0,0,0,255));
    sf::Shape tick2 = sf::Shape::Line(left-8, heightTick2, left-3, heightTick2,2,sf::Color(0,0,0,255));
    openGLwindow.Draw(tick1);
    openGLwindow.Draw(tick2);

    //Write text
    char tick1msg[MAX_BUFFER];
    char tick2msg[MAX_BUFFER];
    sprintf(tick1msg, "0");
    sprintf(tick2msg, "100");
    sf::String tick1msgText(tick1msg);
    sf::String tick2msgText(tick2msg);
    tick1msgText.SetPosition(left - 8 - 10, heightTick1 - 8);
    tick2msgText.SetPosition(left - 8 - 24, heightTick2 - 8);
    tick1msgText.SetSize(12);
    tick2msgText.SetSize(12);
    tick1msgText.SetColor(sf::Color(0, 0, 0));
    tick2msgText.SetColor(sf::Color(0, 0, 0));
    tick1msgText.SetFont(font);   
    tick2msgText.SetFont(font);   
    tick1msgText.SetStyle(1);   
    tick2msgText.SetStyle(1);   
    openGLwindow.Draw(tick1msgText);
    openGLwindow.Draw(tick2msgText);
  }
  return true;
}

/**
  * Method to draw the last image broadcasted to the matlab stream of images.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawMatLab(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container c = containerAux[containerId];
 
  //Draw white background	
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  openGLwindow.Draw(rect);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "MATLAB STREAM");
  sf::String titleText(title);
  titleText.SetPosition(c.x + 8.0f, c.y + c.height - 24.0f);
  titleText.SetSize(16);
  titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);
  return true;
}

/**
  * Method to draw the last image broadcasted to the vision stream of images.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawVision(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container c = containerAux[containerId];
 
  //Draw white background	
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  openGLwindow.Draw(rect);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "VISION NODE STREAM");
  sf::String titleText(title);
  titleText.SetPosition(c.x + 8.0f, c.y + c.height - 24.0f);
  titleText.SetSize(16);
  titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);
  return true;
}

/**
  * Method to draw the last RGB image captured by the kinect.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawKinectRGB(int containerId)
{
  //To implement
 if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  if(kinectRGBStarted)
  {
    sf::Image rgbImg;
    sf::Sprite rgbSprite;
    kinectRGBMutex.Lock();
    rgbImg.LoadFromPixels(KINECT_RGB_SX,KINECT_RGB_SY,kinectRGBCapturedFrame);
    rgbSprite.SetImage(rgbImg);
    kinectRGBMutex.Unlock();
    rgbSprite.Resize(containerAux[containerId].width,containerAux[containerId].height);
    rgbSprite.SetPosition(containerAux[containerId].x,containerAux[containerId].y);
    openGLwindow.Draw(rgbSprite);
  }
  else
   emptyContainer(containerId);

  Container c = containerAux[containerId];
  
  //Draw white background	
  //sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  //openGLwindow.Draw(rect);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "KINECT RGB");
  sf::String titleText(title);
  titleText.SetPosition(c.x + 8.0f, c.y + c.height - 24.0f);
  titleText.SetSize(16);
	if(kinectRGBStarted)
    titleText.SetColor(sf::Color(255, 255, 255));
  else
    titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);
  return true;
}

/**
  * Method to draw the last depth image captured by the kinect.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawKinectDepth(int containerId)
{
  //To implement
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
	if(kinectDepthStarted)
  {
    sf::Image depthImg;
    sf::Sprite depthSprite;
    kinectDepthMutex.Lock();
    depthImg.LoadFromPixels(KINECT_DEPTH_SX,KINECT_DEPTH_SY,kinectDepthCapturedFrame);
    depthSprite.SetImage(depthImg);
    kinectDepthMutex.Unlock();
    depthSprite.Resize(containerAux[containerId].width,containerAux[containerId].height);
    depthSprite.SetPosition(containerAux[containerId].x,containerAux[containerId].y);
    openGLwindow.Draw(depthSprite);
  }
  else
   emptyContainer(containerId);

  Container c = containerAux[containerId];
  
  //Draw white background	
  //sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  //openGLwindow.Draw(rect);

  //Write text
  char title[MAX_BUFFER];
  sprintf(title, "KINECT DEPTH");
  sf::String titleText(title);
  titleText.SetPosition(c.x + 8.0f, c.y + c.height - 24.0f);
  titleText.SetSize(16);
	if(kinectDepthStarted)
    titleText.SetColor(sf::Color(255, 255, 255));
  else
    titleText.SetColor(sf::Color(0, 0, 0));
  titleText.SetFont(font);   
  titleText.SetStyle(3);   
  openGLwindow.Draw(titleText);
  return true;
}

/**
  * Method to draw an info message.
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawInfo(int containerId)
{
  //To implement
 if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container c = containerAux[containerId];
  
  //Draw black background	
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255));
  openGLwindow.Draw(rect);

  //Write text
  char infoMsg[MAX_BUFFER];
  sprintf(infoMsg, "Topic List:\n 0-Info message\n 1-Hand camera\n 2-Place camera\n 3-Kinect RGB\n 4-Kinect depth\n 5-Matlab stream\n 6-Vision_node stream\n 7-Hand state");
  sf::String infoMsgText(infoMsg);
  infoMsgText.SetPosition(c.x + 12.0f, c.y + 20.0f);
  infoMsgText.SetSize(18);
  infoMsgText.SetColor(sf::Color(0, 0, 0));
  infoMsgText.SetFont(font);   
  infoMsgText.SetStyle(1);   
  openGLwindow.Draw(infoMsgText);
  return true;
}

/**
  * Method to draw the mouse pointer in the OpenGL container. 
  * It draws the selected triangle and a dot on the mouse pointer. 
  */ 
void drawMouse()
{
  if(w.selectedBody>=0)
    {
      glEnable(GL_LIGHTING);
      glMatrixMode(GL_MODELVIEW); 
      
      worldMutex.Lock();
      glLoadMatrixd(&(w.cameraMat.transp())[0][0]);
      HomogTransf Hom = w.bodies[w.selectedBody].currPose;
      worldMutex.Unlock();
      
      glPushMatrix();	
      glMultMatrixd(&Hom.transp()[0][0]);
    
      Vec triangle = w.bodies[w.selectedBody].Tri.getRow(w.selectedTriangle);
      Vec vertex0 = w.bodies[w.selectedBody].Vert.getRow((int)triangle[0]);
      Vec vertex1 = w.bodies[w.selectedBody].Vert.getRow((int)triangle[1]);
      Vec vertex2 = w.bodies[w.selectedBody].Vert.getRow((int)triangle[2]);
     
      glEnable(GL_POLYGON_OFFSET_FILL); 
      glEnable(GL_POLYGON_OFFSET_LINE);
      glPolygonOffset(-1.0,0.0);
      glBegin(GL_TRIANGLES); 
      glColor4f(1.0, 0.0, 0.0, 0.4); 
      glVertex3d(vertex0[0],vertex0[1],vertex0[2]);
      glVertex3d(vertex1[0],vertex1[1],vertex1[2]);
      glVertex3d(vertex2[0],vertex2[1],vertex2[2]);
      glEnd(); 
      
      glDisable(GL_LIGHTING);
      glLineWidth(2.0);
      glBegin(GL_LINE_STRIP);
      glColor4f(1.0, 0.0, 0.0, 1.0);
      glVertex3d(vertex0[0],vertex0[1],vertex0[2]);
      glVertex3d(vertex1[0],vertex1[1],vertex1[2]);
      glVertex3d(vertex2[0],vertex2[1],vertex2[2]);
      glVertex3d(vertex0[0],vertex0[1],vertex0[2]);
      glEnd();
      glDisable(GL_POLYGON_OFFSET_FILL); 
      glDisable(GL_POLYGON_OFFSET_LINE);
      glPolygonOffset(1.0,0.0);
      
      glPopMatrix();

      glPointSize(5.0);
      glColor3f(1.0, 0.0, 0.0);
      glBegin(GL_POINTS); // render with points
      worldMutex.Lock();
      Vec direction = w.mouseCamera - w.cameraMat.inv().getTranslation();
      direction.normalize();
      Vec position = w.mouseCamera + direction*1.0;
      worldMutex.Unlock();
      glVertex3d(position[0], position[1], position[2]); //display a point
      glEnd(); 
   }
}

/**
  * Method to set a white empty background to a container
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool emptyContainer(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container c = containerAux[containerId];
  sf::Shape rect = sf::Shape::Rectangle(c.x, c.y, c.x + c.width, c.y + c.height, sf::Color(255,255,255,255));
  openGLwindow.Draw(rect);
  return true;
}

/**
  * Method to draw an auxiliary container with the current topicId 
  * @param containerId Id of the container where to draw.
  * @return True under success.
  */
bool drawContainer(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container c = containerAux[containerId];
  if(!c.isActive)
    return emptyContainer(containerId);
  else
  { 
    switch(c.topic)
    {
      case 0:
        return drawInfo(containerId);
      case 1:
        return drawHandCam(containerId);
      case 2:
        return drawPlaceCam(containerId);
      case 3:
        return drawKinectRGB(containerId);
      case 4:
        return drawKinectDepth(containerId);
      case 5:
        return drawMatLab(containerId);
      case 6:
        return drawVision(containerId);
      case 7:
        return drawHandState(containerId);
      default:
        return emptyContainer(containerId);
    }
  }
}

/**
  * Method to draw a thin border around all containers in the visualizer
  */
void drawBorders()
{
  //exterior
  sf::Shape line;
  line = sf::Shape::Line(0, 0, 0, originalHeight, 7, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(0, originalHeight,originalWidth,originalHeight, 7, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(originalWidth,originalHeight,originalWidth,0, 7, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(originalWidth,0,0,0, 7, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  
  //ROSout and text containers
  line = sf::Shape::Line(containerROSout.x,containerROSout.y,containerROSout.x,originalHeight, 4, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(containerText.x,containerText.y,originalWidth,containerText.y, 4, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);

  //auxContainers
  line = sf::Shape::Line(containerAux[0].x,containerAux[0].y,containerAux[2].x + containerAux[2].width,containerAux[0].y, 4, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(containerAux[1].x,containerAux[1].y,containerAux[1].x,originalHeight, 4, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
  line = sf::Shape::Line(containerAux[2].x,containerAux[2].y,containerAux[2].x,originalHeight, 4, sf::Color(0,0,0)); 
  openGLwindow.Draw(line);
}


/**
  * Method to toggle the content of an auxiliary container.
  * @param containerId Id of the container to toggle.
  * @return True under success.
  */
bool toggleContainer(int containerId)
{
  if(containerId < 0 || containerId >= NUM_AUX_CONTAINERS) 
    return false;
  Container *c = &containerAux[containerId];
  if(c->topic<NUM_TOPICS-1)
    (c->topic)++;
  else
    c->topic = 0;  
  return true;
}

/**
  * Method to initialize the configuration of all containers in display_node.
  * @param containerId Id of the container to toggle.
  */
void initContainers(int windowWidth, int windowHeight)
{
  //First find the height of the auxiliary containers
  int leftWidth = windowWidth - TEXT_CONTAINER_WIDTH;
  double auxContainerWidth = (double)leftWidth/3.0; //Divided by the number of auxiliary containers
  double auxContainerHeight = auxContainerWidth/AUX_CONTAINER_X2Y_RATIO;
  
  //OpenGL container
  containerOpenGL.x = 0;
  containerOpenGL.y = 0;
  containerOpenGL.width = windowWidth - TEXT_CONTAINER_WIDTH;
  containerOpenGL.height = windowHeight - (int)auxContainerHeight;
  containerOpenGL.isActive = true;
 
  //ROSout container
  containerROSout.x = containerOpenGL.width;
  containerROSout.y = 0;
  containerROSout.width = TEXT_CONTAINER_WIDTH;
  containerROSout.height = windowHeight - (int)auxContainerHeight;
  containerROSout.isActive = true;
 
  //Text container
  containerText.x = containerOpenGL.width;
  containerText.y = containerOpenGL.height;
  containerText.width = TEXT_CONTAINER_WIDTH;
  containerText.height = (int)auxContainerHeight;
  containerText.isActive = true;
  
  //Aux1 container
  containerAux[0].x = 0;
  containerAux[0].y = containerOpenGL.height;
  containerAux[0].width = (int)auxContainerWidth;
  containerAux[0].height = (int)auxContainerHeight;
  containerAux[0].isActive = true;
  containerAux[0].topic = 3;

  //Aux2 container
  containerAux[1].x = (int)auxContainerWidth;
  containerAux[1].y = containerOpenGL.height;
  containerAux[1].width = (int)auxContainerWidth;
  containerAux[1].height = (int)auxContainerHeight;
  containerAux[1].isActive = true;
  containerAux[1].topic = 4;

  //Aux3 container
  containerAux[2].x = 2*auxContainerWidth;
  containerAux[2].y = containerOpenGL.height;
  containerAux[2].width = (int)auxContainerWidth;
  containerAux[2].height = (int)auxContainerHeight;
  containerAux[2].isActive = true;
  containerAux[2].topic = 7;
}

/**
  * Method to find and update the coordinates of the mouse pointer (w.mouseCamera) in the camera plane.
  */ 
void updateMouseOnCamera()
{
  //We return as feedback the position of the camera and the state of the mouse
  const sf::Input& Input =  openGLwindow.GetInput();
 //Mouse in World Coordinates in projected Camera
  worldMutex.Lock();
  HomogTransf OriginToCamera = w.cameraMat.inv();
  worldMutex.Unlock();
  Vec xCam = OriginToCamera.getRotation().getCol(0);
  Vec yCam = OriginToCamera.getRotation().getCol(1);
  Vec zCam = OriginToCamera.getRotation().getCol(2);
  Vec pos = OriginToCamera.getTranslation();
  Quaternion quat = OriginToCamera.getRotation().getQuaternion();
  double sizeScreenY = 2*ZNEAR*tan(FIELD_OF_VIEW*PI/(2*180.0));
  double sizeScreenX = 2*ZNEAR*SKEW*tan(FIELD_OF_VIEW*PI/(2*180.0));
  double mouseX = (Input.GetMouseX() - containerOpenGL.width/2.0)*sizeScreenX/containerOpenGL.width; 
  double mouseY = (Input.GetMouseY() - containerOpenGL.height/2.0)*sizeScreenY/containerOpenGL.height; 
  worldMutex.Lock();
  w.mouseCamera = pos - zCam*ZNEAR + xCam*mouseX - yCam*mouseY;
  worldMutex.Unlock();
}

/**
  * Method to project the mouse pointer from the camera plane into the workspace.
  * It updates w.selectedBody, w.selectedTriangle and w.mouseWorld.
  */ 
void updateMouseOnWorld()
{
  //Mouse projected to Objects
  worldMutex.Lock();
  HomogTransf OriginToCamera = w.cameraMat.inv();
  worldMutex.Unlock();
  RotMat cameraRot = OriginToCamera.getRotation();
  RotMat incRot;
  Vec oldZ = -cameraRot.getCol(2);
  worldMutex.Lock();
  Vec newZ = w.mouseCamera - OriginToCamera.getTranslation();
  worldMutex.Unlock();
  
  newZ.normalize();
  double rotAngle = acos(oldZ * newZ);
  Vec rotAxis = oldZ^newZ;
  incRot.setAxisAngle(rotAxis, rotAngle);
  RotMat pointerRot = incRot*cameraRot;
  
  PQP_REAL rotPointer[3][3];
  PQP_REAL transPointer[3];
  transPointer[0] = OriginToCamera.getTranslation()[0];
  transPointer[1] = OriginToCamera.getTranslation()[1];
  transPointer[2] = OriginToCamera.getTranslation()[2];
  rotPointer[0][0] = pointerRot[0][0];
  rotPointer[0][1] = pointerRot[0][1];
  rotPointer[0][2] = pointerRot[0][2];
  rotPointer[1][0] = pointerRot[1][0];
  rotPointer[1][1] = pointerRot[1][1];
  rotPointer[1][2] = pointerRot[1][2];
  rotPointer[2][0] = pointerRot[2][0];
  rotPointer[2][1] = pointerRot[2][1];
  rotPointer[2][2] = pointerRot[2][2];
  
  PQP_REAL rotBody[3][3];
  PQP_REAL transBody[3];
  PQP_CollideResult result;
  Vec mouseObject(0.0,3);
  double minDist = 1E20;
  int objectId = -1;
  int selectedTriangle = -1;
  int i,j;
  for(j=0; j<w.nbodies; j++)
    {
      if(w.bodies[j].interact)
	{
	  HomogTransf Hom = w.bodies[j].currPose;
	  Vec pos = Hom.getTranslation();
	  RotMat rotation = Hom.getRotation();
	  transBody[0] = pos[0];
	  transBody[1] = pos[1];
	  transBody[2] = pos[2];
	  rotBody[0][0] = rotation[0][0];
	  rotBody[0][1] = rotation[0][1];
	  rotBody[0][2] = rotation[0][2];
	  rotBody[1][0] = rotation[1][0];
	  rotBody[1][1] = rotation[1][1];
	  rotBody[1][2] = rotation[1][2];
	  rotBody[2][0] = rotation[2][0];
	  rotBody[2][1] = rotation[2][1];
	  rotBody[2][2] = rotation[2][2];
	  
	  PQP_Collide(&result, rotPointer, transPointer, &pointer, rotBody, transBody, &w.bodies[j].pqp); //Quick
	  if (result.Colliding())
	    {
	      // From the set of intersecting triangles, find the one closer to the camera origin.
	      int numPairs = result.NumPairs();
	      Vec distances(numPairs);
	      for (i=0; i<numPairs; i++)
		{
		  Vec triangle = w.bodies[j].Tri.getRow(result.Id2(i));
		  Vec p0 = Hom*w.bodies[j].Vert.getRow(triangle[0]);
		  Vec p1 = Hom*w.bodies[j].Vert.getRow(triangle[1]);
		  Vec p2 = Hom*w.bodies[j].Vert.getRow(triangle[2]);
		  worldMutex.Lock();
		  Vec intPoint = linePlaneIntersection(OriginToCamera.getTranslation(),w.mouseCamera, p0, (p1-p0)^(p2-p0));
		  worldMutex.Unlock();
		  distances[i] = (intPoint - OriginToCamera.getTranslation()).norm();
		}
	      int triId  = distances.minInd(); 
	      Vec triangle = w.bodies[j].Tri.getRow(result.Id2(triId));
	      //Projection of the camera center to the plane defined by the triangle.
	      Vec p0 = Hom*w.bodies[j].Vert.getRow(triangle[0]);
	      Vec p1 = Hom*w.bodies[j].Vert.getRow(triangle[1]);
	      Vec p2 = Hom*w.bodies[j].Vert.getRow(triangle[2]);
	      worldMutex.Lock();
	      Vec selectionPoint = linePlaneIntersection(OriginToCamera.getTranslation(),w.mouseCamera, p0, (p1-p0)^(p2-p0));
	      worldMutex.Unlock();
	      
	      double dist = (selectionPoint - OriginToCamera.getTranslation()).norm();
	      if(dist<minDist)
		{
		  selectedTriangle = result.Id2(triId);
		  mouseObject = selectionPoint;
		  minDist = dist;
		  objectId =j;
		}
	      
	    }
	}
    }
  worldMutex.Lock();
  w.selectedBody = objectId;
  w.selectedTriangle = selectedTriangle;
  w.mouseWorld = mouseObject;
  worldMutex.Unlock();
}

/**
  * Handler of SFML window events including keyboard and mouse.
  * @return True under success.
  */
bool handleWindowEvent(sf::Event Event, const sf::Input& Input)
{
  switch (Event.Type)
  {
    case sf::Event::Closed:
      {
        openGLwindow.Close();
        return 1;
      }
    case sf::Event::Resized:
      {
       // Maintain the X-Y ratio of the window
       double diffWidth = fabs(Event.Size.Width - currentWidth);
       double diffHeight = fabs(Event.Size.Height - currentHeight);
       if(diffWidth>diffHeight)
         {
           currentWidth = Event.Size.Width;
           currentHeight = (int)((double)Event.Size.Width/windowWidth2HeightRatio);
         }
       else
         {
           currentHeight = Event.Size.Height;
           currentWidth = (int)((double)Event.Size.Height*windowWidth2HeightRatio);
         }
       openGLwindow.SetSize(currentWidth,currentHeight);
      }
    case sf::Event::MouseButtonPressed:
      {
        previousMouseX = Event.MouseButton.X;
        previousMouseY = Event.MouseButton.Y;
        worldMutex.Lock();
        initialOriginToCamera = w.cameraMat.inv();
        worldMutex.Unlock();
        if (w.selectedBody>=0) //If we are touching an object
        {
          worldMutex.Lock();
          motionCenter = w.mouseWorld; //both for motion of the camera and objects
          worldMutex.Unlock();
        }
        else
          motionCenter = Vec(0.0,3);//If we are not touching any oblect, the motion center is the origin.
        pressed=true;
        break;
      }
    case sf::Event::MouseButtonReleased:
      {
        pressed=false;
        break;
      }
    case sf::Event::MouseMoved:
      {
        if(pressed) //We first require to go thorugh the MouseButtonPressed event
        {
          if(Input.IsMouseButtonDown(sf::Mouse::Right)) //Camera Rotation
          {
            Vec xCam = initialOriginToCamera.getRotation().getCol(0);
            Vec yCam = initialOriginToCamera.getRotation().getCol(1);
            RotMat Rx,Ry;
            Rx.setAxisAngle(xCam, 0.002*(double)(previousMouseY - Event.MouseMove.Y));
            Ry.setAxisAngle(yCam, 0.002*(double)(previousMouseX - Event.MouseMove.X));
            HomogTransf R = HomogTransf(Rx*Ry,Vec(0.0,3));

            HomogTransf T = HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),-motionCenter);
            //HomogTransf T = HomogTransf(Quaternion("1.0 0.0 0.0 0.0").getRotMat(),Vec(3));
            HomogTransf OriginToCamera = T.inv()*R*T*initialOriginToCamera;

            //Correction for keeping head vertical
            //The vector y of camera has to be always in the plane defined by zCam and zWorld
            yCam = OriginToCamera.getRotation().getCol(1);
            Vec zCam = OriginToCamera.getRotation().getCol(2);
            Vec vNormal = Vec("0.0 0.0 1.0",3)^zCam;
            vNormal.normalize();
            Vec yCamProjected = yCam - vNormal*(yCam*vNormal);
            yCamProjected.normalize();

            Vec v = yCam + yCamProjected;
            v.normalize();
            HomogTransf Projection = HomogTransf(Quaternion(v*yCamProjected, v^yCamProjected).getRotMat(), Vec(0.0,3)); 
            OriginToCamera = T.inv()*Projection*T*OriginToCamera;
            worldMutex.Lock();
            w.cameraMat=OriginToCamera.inv();
            worldMutex.Unlock();
          }
          else if(Input.IsMouseButtonDown(sf::Mouse::Left)) //XY Camera Translation
          {
            Vec disp = Vec(0.0,3);
            disp[0] = -1.5*(double)(previousMouseX - Event.MouseMove.X);
            disp[1] = 1.5*(double)(previousMouseY - Event.MouseMove.Y);
            worldMutex.Lock();
            w.cameraMat.setTranslation(initialOriginToCamera.inv().getTranslation() + disp);
            worldMutex.Unlock();
          }
        }
        break;
      }
    case sf::Event::MouseWheelMoved: //Z Camera Translation
      {
        //Mouse in World Coordinates
        Vec xCam = Vec("1.0 0.0 0.0",3);
        Vec yCam = Vec("0.0 1.0 0.0",3);
        Vec zCam = Vec("0.0 0.0 1.0",3);
        double sizeScreenY = 2*ZNEAR*tan(FIELD_OF_VIEW*PI/(2*180.0));
        double sizeScreenX = 2*ZNEAR*SKEW*tan(FIELD_OF_VIEW*PI/(2*180.0));
        double mouseX = (Input.GetMouseX() - openGLwindow.GetWidth()/2.0)*sizeScreenX/openGLwindow.GetWidth(); 
        double mouseY = (Input.GetMouseY() - openGLwindow.GetHeight()/2.0)*sizeScreenY/openGLwindow.GetHeight(); 
        Vec disp = - zCam*ZNEAR + xCam*mouseX - yCam*mouseY;
        disp.normalize();
        disp *= 150.0*Event.MouseWheel.Delta;
        worldMutex.Lock();
        w.cameraMat.setTranslation(w.cameraMat.getTranslation() + disp);
        worldMutex.Unlock();
        break;
      }
    case sf::Event::KeyPressed:
      {
        if(Event.Key.Code == sf::Key::Escape)
        {
          openGLwindow.Close();
        }
        if(Event.Key.Code == sf::Key::Num0){
          w.links[0].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Num1){
          w.links[1].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Num2){
          w.links[2].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Num3){
          w.links[3].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Num4){
          w.links[4].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Num5){
          w.links[5].angle += 10;
        }
        if(Event.Key.Code == sf::Key::Q){
          w.links[6].angle += 10;
        }
        if(Event.Key.Code == sf::Key::W){
          w.links[7].angle += 10;
        }
        if(Event.Key.Code == sf::Key::E){
          w.links[8].angle += 10;
        }
	if(Event.Key.Code == sf::Key::F2)
        {
          toggleContainer(0);
        }
        if(Event.Key.Code == sf::Key::F3)
        {
          toggleContainer(1);
        }
        if(Event.Key.Code == sf::Key::F4)
        {
          toggleContainer(2);
        }
        if(Event.Key.Code == sf::Key::F1)
        {
          //Screenshot
          char command[MAX_BUFFER];
          sprintf(command,"mkdir -p screenshots");
          if(system(command)==0)
          {  
            time_t rawTime = time(NULL);
            struct tm * timeInfo = localtime ( &rawTime );
            char fileName[MAX_BUFFER];
            sprintf (fileName, "screenshots/screenshot%02d-%02d-%02d-%02d:%02d:%02d.jpg", 1900 + timeInfo->tm_year, timeInfo->tm_mon, timeInfo->tm_mday, timeInfo->tm_hour, timeInfo->tm_min, timeInfo->tm_sec);
            sf::Image screen = openGLwindow.Capture();
            screen.SaveToFile(fileName); 
          }
       }
        if(Event.Key.Code == sf::Key::C)
        {
          if(Event.Key.Control)
          {
            openGLwindow.Close();
            return 1;
          }
        } 
        break;
      }
    default:
      break;
  };
  return 0;
}


/**
  * Method to parse a .3d configuration file.
  * It dynamically creates and popultes the list of links and static bodies of the world global variable.
  * @param configFile Name of the .3d configuration file.
  * @param configFolder Path to the location of the .3d configuration file. 
  * @return True under success.
  */
bool readConfigFile(const char* configFile, const char* configFolder)
{
  int i;
  bool bodyEnd, linkEnd;
  char c;
  FILE *cfile;
  char buffer[MAX_BUFFER];
  
  //Open configuration file
  sprintf(buffer,"%s/%s",configFolder,configFile);
  if ((cfile=fopen(buffer,"r")) == NULL)
    return 0;

  ////////////
  //Read links 
  long pos=fileSearch(cfile, "@NL");
  if(pos==-1)
    return 0;     //Invalid file
  fseek(cfile,pos,SEEK_SET);

  //Read the number of links
  if(fscanf(cfile,"@NL %d\n", &(w.nLinks))!=1)
    return 0;
  w.links.resize(w.nLinks);
  
  //Update world
  for(i=0; i<w.nLinks; i++)
  {
    //Initialize body with default parameters.
    w.links[i].trans = Vec("0.0 0.0 0.0",3);
    w.links[i].rotAxis = Vec("0.0 0.0 1.0",3);
    
    //Look in the file for configuration of link i
    sprintf(buffer,"@L%d",i);
    pos=fileSearch(cfile,buffer);
    fseek(cfile,pos,SEEK_SET);
    w.links[i].linkId = i;
  
    //scan line by line the configuration of the link
    linkEnd=false;
    while(fgets(buffer,MAX_BUFFER,cfile) && (!linkEnd))
    {
      sscanf(buffer,"@%c",&c);
      switch(c)
      {
        case 'e':
          linkEnd=true;
          break;
        case 't':
          sscanf(buffer,"@t %lf %lf %lf",&w.links[i].trans[0],&w.links[i].trans[1],&w.links[i].trans[2]);
          break;
        case 'r':
          sscanf(buffer,"@r %lf %lf %lf",&w.links[i].rotAxis[0],&w.links[i].rotAxis[1],&w.links[i].rotAxis[2]);
          break;
        case 'l':
          sscanf(buffer,"@l %d",&w.links[i].prevLink);
          break;
        default:
          break;
      };
    }
  }

  ////////////
  //Read bodies 
 
  pos=fileSearch(cfile, "@NB");
  if(pos==-1)
    return 0;     //Invalid file
  fseek(cfile,pos,SEEK_SET);

  //Read the number of bodies
  if(fscanf(cfile,"@NB %d\n", &(w.nbodies))!=1)
    return 0;

  //update world
  for(i=0; i<w.nbodies; i++)
  {
    Body newBody;
    //Initialize body with default parameters.
    newBody.pos = Vec("0.0 0.0 0.0",3);
    newBody.quat = Quaternion("1.0 0.0 0.0 0.0");
    newBody.scale = 1.0;
    newBody.color = Vec("0.5 0.5 0.5",3);
    newBody.mode = NORMAL;
    newBody.frame = false;
    newBody.interact = false;
    newBody.link = 0;

    //Look in the file for configuration of body i
    sprintf(buffer,"@B%d",i);
    pos=fileSearch(cfile,buffer);
    fseek(cfile,pos,SEEK_SET);
    //scan line by line the configuration of the body
    bodyEnd=false;

    while(fgets(buffer,MAX_BUFFER,cfile) && (!bodyEnd))
    {
      sscanf(buffer,"@%c",&c);
      switch(c)
      {
        case 'e':
          bodyEnd=true;
          break;
        case 'a':
          sscanf(buffer, "@a %s", newBody.fileName);
	  sprintf(buffer,"%s/%s",configFolder, newBody.fileName);
	  readWRL(buffer, newBody.Vert, newBody.Tri);
	  break;
	case 'p':
	  sscanf(buffer,"@p %lf %lf %lf",&newBody.pos[0],&newBody.pos[1],&newBody.pos[2]);
	  break;
	case 'o':
          sscanf(buffer,"@o %lf %lf %lf %lf",&newBody.quat[0],&newBody.quat[1],&newBody.quat[2],&newBody.quat[3]);
          break;
        case 's':
          sscanf(buffer,"@s %lf",&newBody.scale);
          break;
        case 'c':
          sscanf(buffer,"@c %lf %lf %lf",&newBody.color[0],&newBody.color[1],&newBody.color[2]);
          break;
        case 'f':
          int frame;
          sscanf(buffer,"@f %d",&frame);
          newBody.frame = (bool)frame;
          break;
        case 'i':
          int interact;
          sscanf(buffer,"@i %d",&interact);
          newBody.interact = (bool)interact;
        case 'l':
          sscanf(buffer,"@l %d",&newBody.link);
          break;
        case 'm':
          sscanf(buffer,"@m %d",&newBody.mode);
          break;
        default:
          break;
      };
    }

    //Add body to world
    newBody.Vert*=newBody.scale;
    w.bodies.push_back(newBody);
    if (newBody.link == -1)
      w.idStaticBodies.push_back(i);  //add static body
    else
      w.links[newBody.link].idBodies.push_back(i); //add Links body
  }
  return 1;
}

/**
  * Method to create the collision PQP models of all objects in the environment for GUI.
  */
void createPQPModels() 
{
  PQP_REAL p1[3],p2[3],p3[3];  // 3 points will make triangle p
  for (int i=0; i<w.nbodies; i++)
    {
      if(w.bodies[i].interact)  // We only create the PQP model for collision checking if the body is meant to be interactible
	{
	  w.bodies[i].pqp.BeginModel();   // begin the model
	  for (int j=0; j<w.bodies[i].Tri.nn; j++)
	    {
	      Vec triangle=w.bodies[i].Tri.getRow(j);
	      Vec vertex=w.bodies[i].Vert.getRow((int)triangle[0]);
	      p1[0]=vertex[0];
	      p1[1]=vertex[1];
	      p1[2]=vertex[2];
	      
	      vertex=w.bodies[i].Vert.getRow((int)triangle[1]);
	      p2[0]=vertex[0];
	      p2[1]=vertex[1];
	      p2[2]=vertex[2];
	      
	      vertex=w.bodies[i].Vert.getRow((int)triangle[2]);
	      p3[0]=vertex[0];
	      p3[1]=vertex[1];
	      p3[2]=vertex[2];
	      
	      w.bodies[i].pqp.AddTri(p1,p2,p3,j); // add triangle p
	    }
	  w.bodies[i].pqp.EndModel();	   // end (build) the model
	}
    }

  //Model with a single really long and thin triangle pointing in the Z direction.
  //This will be used to find the intersectino of the mouse pointer and all objects in the environment. 
  pointer.BeginModel();
  p1[0]=1e-9;
  p1[1]=0.0;
  p1[2]=0.0;
  p2[0]=-1e-9;
  p2[1]=0.0;
  p2[2]=0.0;
  p3[0]=0.0;
  p3[1]=0.0;
  p3[2]=-1e+4;
  pointer.AddTri(p1,p2,p3,0);
  pointer.EndModel();       
}

/**
  * Helper function to find the minimum distance from a point to a line defined by two points.
  * @param p0 Point.
  * @param p1 Point defining line.
  * @param p2 Point defining line. 
  */
double pointLineDistance(Vec p0, Vec p1, Vec p2)
{
  return (((p0-p1)^(p0-p2)).norm()/(p2-p1).norm());
}

/**
  * Helper function to find the projection of a point to a plane defined by a point and its normal.
  * @param p0 Point.
  * @param p1 Point defining plane.
  * @param n  Normal to plane.
  */
Vec pointPlaneProjection(Vec p0, Vec p1, Vec n)
{
  n.normalize();
  return (p0 + n*((p1-p0)*n));
}

/**
  * Helper function to find the point in line 1 closest to line 2.
  * @param p1 Point defining line 1.
  * @param p2 Point defining line 1.
  * @param p3 Point defining line 2.
  * @param p4 Point defining line 2.
  */ 
Vec lineLineProjection(Vec p1, Vec p2, Vec p3, Vec p4)
{
   Vec p13 = p1 - p3;
   Vec p43 = p4 - p3;
   if (p43.norm() < TOLERANCE)
     return(Vec());
   Vec p21 = p2 - p1;
   if (p43.norm() < TOLERANCE)
     return(Vec());

   double denom = (p21*p21) * (p43*p43) - (p43*p21) * (p43*p21);
   if (fabs(denom) < TOLERANCE)
     return(Vec());
   double numer = (p13*p43) * (p43*p21) - (p13*p21) * (p43*p43);
   double mu = numer / denom;
   return(p1 + p21*mu);
}
 

/**
  * Helper function to find intersection between a line and a plane.
  * @param p1 Point defining line 1.
  * @param p2 Point defining line 1.
  * @param p3 Point defining plane.
  * @param n  Normal to plane.
  */ 
Vec linePlaneIntersection(Vec p1, Vec p2, Vec p3, Vec n)
{
  n.normalize();
  double u = n*(p3 - p1)/(n*(p2-p1));
  return (p1 + (p2-p1)*u);
}


int main(int argc, char *argv[])
{ 
  char buffer[MAX_BUFFER];

  //Default values
  handCamStarted=false;
  placeCamStarted=false;
  kinectRGBStarted=false;
  kinectDepthStarted=false;
  robot_CartesianLog_values[0] = 0.0;
  robot_CartesianLog_values[1] = 0.0;
  robot_CartesianLog_values[2] = 0.0;
  robot_CartesianLog_values[3] = 1.0;
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
  hand_EncodersLog_values[0] = 0;
  hand_EncodersLog_values[1] = 0;
  hand_EncodersLog_values[2] = 0;
  hand_EncodersLog_values[3] = 0;
  hand_AnglesLog_values[0] = 0.0;
  hand_AnglesLog_values[1] = 0.0;
  hand_AnglesLog_values[2] = 0.0;
  hand_AnglesLog_values[3] = 0.0;
  
  //ROS stuff
  ros::init(argc, argv, "display");
  ros::NodeHandle node;
  nodePtr = &node;
  ROS_INFO("DISPLAY: Subscribing to topics...");
  subscribeTopics();
  
  //Read 3D display configuration 
  std::string configFolder;
  std::string configFile;
  nodePtr->getParam("/display/configFolder", configFolder);
  nodePtr->getParam("/display/configFile", configFile);
  if(!readConfigFile(configFile.c_str(), configFolder.c_str()))
    {
      ROS_ERROR("DISPLAY: Could not find configuration file %s/%s",configFolder.c_str(),configFile.c_str());
      exit(-1);
    }

  //Create PQP models for GUI interface
  createPQPModels();
  
  //Load RI logo from file
  sprintf(buffer, "%s/logoRI.png", configFolder.c_str()); 
  if (!logoRI.LoadFromFile(buffer))
  {
     ROS_ERROR("DISPLAY: Could not find the RI logo at %s.",buffer);
     exit(-1);
  }

 //Load font
  sprintf(buffer, "%s/cour.ttf", configFolder.c_str());
  if(!font.LoadFromFile(buffer))
  {
    ROS_ERROR("DISPLAY: Could not load font");
    exit(-1);
  }

  
 
  //Initialization of the Window Display
  //capture Desktop configuration
  sf::VideoMode Mode = sf::VideoMode::GetDesktopMode();
  int nMonitors, activeMonitor;
  nodePtr->getParam("/display/nMonitors", nMonitors);
  nodePtr->getParam("/display/activeMonitor", activeMonitor);
  int windowHeight, windowWidth, windowPosX, windowPosY;
  //Optimized for Ubuntu window
  int sizeBar = 29;
  if(nMonitors == 2)
    {
      windowHeight = Mode.Height - sizeBar;
      windowWidth = (int)Mode.Width/2;
      if(activeMonitor == 1)
        {
          windowPosX = 0;
          windowPosY = sizeBar;
        }
      else
	{
          windowPosX = (int)Mode.Width/2;
          windowPosY = sizeBar;
        }
    }
  else
    {
      windowHeight = Mode.Height - sizeBar;
      windowWidth = Mode.Width;
      windowPosX = 0;
      windowPosY = sizeBar;
    }
  windowWidth2HeightRatio = (double)windowWidth/(double)windowHeight;
  initContainers(windowWidth,windowHeight);
  openGLwindow.Create(sf::VideoMode(windowWidth, windowHeight, Mode.BitsPerPixel), "RobotDisplay");
  openGLwindow.SetPosition(windowPosX,windowPosY);
  openGLwindow.PreserveOpenGLStates(true); 
  openGLwindow.SetActive();
  //Save window size
  currentWidth = windowWidth;
  currentHeight = windowHeight;
  originalWidth = windowWidth;
  originalHeight = windowHeight;
  
  openGLInit();
  createDisplayLists();

  const sf::Input& Input =  openGLwindow.GetInput();
  sf::Event Event;
  sf::Clock time;
  pressed=false;
  
  //ROS initialization
  ROS_INFO("DISPLAY: Running node /display...");
  ros::AsyncSpinner spinner(2); 
  spinner.start();

  //ROS advertise services
  advertiseServices();
  
  //main OpenGL loop
  while(openGLwindow.IsOpened() && ros::ok())
  {
    time.Reset(); 
    while (openGLwindow.GetEvent(Event))
      handleWindowEvent(Event, Input);
    
    updateMouseOnCamera();
    updateMouseOnWorld();
    
    drawWorld();
    drawText();
    drawROSout();
    drawContainer(0);
    drawContainer(1);
    drawContainer(2);
    drawBorders();
    drawMouse();
    openGLwindow.Display();

    //Refresh of the screen
    double sleepTime = 1.0/REFRESH_RATE - time.GetElapsedTime();
    if(sleepTime>0)
      sf::Sleep(sleepTime); 
  }
  ROS_INFO("DISPLAY: Shutting down node /display...");
  return 1;
}
