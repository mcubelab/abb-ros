#include "ABBInterpreter.h"

/**
  * Formats message to ping the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::pingRobot(int idCode)
{
  char buff[10];
  string msg("00 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setCartesian(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[20];
  string msg("01 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the cartesian coordinates of the ABB robot, but
  * using a joint move to get there.
  * The coordinates are always with respect to the currently defined work object and tool.
  * @param x X-coordinate of the robot.
  * @param y Y-coordinate of the robot.
  * @param z Z-coordinate of the robot.
  * @param q0 First component of the orientation quaternion.
  * @param qx Second component of the orientation quaternion.
  * @param qy Third component of the orientation quaternion.
  * @param qz Fourth component of the orientation quaternion.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setCartesianJ(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[20];
  string msg("01 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "j #";

  return (msg);
}


/**
  * Formats message to set the joint coordinates of the ABB robot.
  * @param joint1 Value of joint 1.
  * @param joint2 Value of joint 2.
  * @param joint3 Value of joint 3.
  * @param joint4 Value of joint 4.
  * @param joint5 Value of joint 5.
  * @param joint6 Value of joint 6.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setJoints(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode)
{
  char buff[20];
  string msg("02 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.2lf ",joint1);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint2);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint3);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint4);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint5);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint6);
  msg += buff ;
  msg += "#";

  return (msg);
}


string ABBInterpreter::getIK(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[20];
  string msg("12 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

string ABBInterpreter::getFK(double joint1, double joint2, double joint3, double joint4, double joint5, double joint6, int idCode)
{
  char buff[20];
  string msg("13 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.2lf ",joint1);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint2);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint3);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint4);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint5);
  msg += buff ;
  sprintf(buff,"%+08.2lf ",joint6);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to query the ABB robot for its cartesian coordinates.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::getCartesian(int idCode)
{
  char buff[20];
  string msg("03 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);
}

/**
  * Formats message to query the ABB robot for its joint axis coordinates.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::getJoints(int idCode)
{
  char buff[20];
  string msg("04 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);
}

/**
  * Formats message to define the tool coordinates.
  * @param x X-coordinate of the tool.
  * @param y Y-coordinate of the tool.
  * @param z Z-coordinate of the tool.
  * @param q0 First component of the orientation quaternion of the tool.
  * @param qx Second component of the orientation quaternion of the tool.
  * @param qy Third component of the orientation quaternion of the tool.
  * @param qz Fourth component of the orientation quaternion of the tool.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setTool(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[20];
  string msg("06 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to define the tool coordinates.
  * @param m Mass of the tool in Kg.
  * @param cgx X-coordinate of the CG of the tool in mm.
  * @param cgy Y-coordinate of the CG of the tool in mm.
  * @param cgz Z-coordinate of the CG of the tool in mm.
  * @param ix Moment of inertia with respect to axis X.
  * @param iy Moment of inertia with respect to axis Y.
  * @param iz Moment of inertia with respect to axis Z.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setInertia(double m, double cgx, double cgy, double cgz, double ix, double iy, double iz, int idCode)
{
  char buff[80];
  string msg("14 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.5lf ",m);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",cgx);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",cgy);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",cgz);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",ix);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",iy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",iz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to define the coordinates of the work object reference frame.
  * @param x X-coordinate of the work object reference frame.
  * @param y Y-coordinate of the work object reference frame.
  * @param z Z-coordinate of the work object reference frame.
  * @param q0 First component of the orientation quaternion of the work object reference frame.
  * @param qx Second component of the orientation quaternion of the work object reference frame.
  * @param qy Third component of the orientation quaternion of the work object reference frame.
  * @param qz Fourth component of the orientation quaternion of the work object reference frame.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setWorkObject(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  char buff[20];
  string msg("07 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%+08.1lf ",x);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",y);
  msg += buff ;
  sprintf(buff,"%+08.1lf ",z);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",q0);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qx);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qy);
  msg += buff ;
  sprintf(buff,"%+08.5lf ",qz);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the speed of the ABB robot.
  * The values specified for tcp and ori are modified by the percentage of override specified by the operator in the teach pendant. 
  * @param tcp Linear speed of the TCP in mm/s (max recommended value ~ 500).
  * @param ori Reorientation speed of the TCP in deg/s (max recommended value ~ 150).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setSpeed(double tcp, double ori, int idCode)
{
  char buff[20];
  string msg("08 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%08.1lf ",tcp);
  msg += buff ;
  sprintf(buff,"%08.2lf ",ori);
  msg += buff ;
  msg += "#";

  return (msg);
}


/**
  * Formats message to set the TCP acceleration of the ABB robot.
  * @param acc Linear acceleration of the TCP in mm/s^2 (0.1-28 m/s^2).
  * @param deacc Linear deacceleration of the TCP in mm/s^2 (0.1-28 m/s^2).
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setAcc(double acc, double deacc, int idCode)
{
  char buff[20];
  string msg("15 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%08.2lf ",acc);
  msg += buff ;
  sprintf(buff,"%08.2lf ",deacc);
  msg += buff ;
  msg += "#";

  return (msg);
}


/*
  * Deprecated: Formats message to set the zone mode of the ABB robot (distance from where to interpolate to the next destination).
    Possible modes:
    ---------------
    Mode - Name in RAPID - Linear  - Orientation
    0          fine         0 mm        0°
    1          z0           0.3 mm      0.03°  <- Default and recommended value.
    2          z1           1 mm        0.1°
    3          z5           5 mm        0.8°
    4          z10         10 mm        1.5°
  * @param mode Mode to chose between 0-4.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  
string ABBInterpreter::setZone(int mode,int idCode)
{
  switch (mode)
    {
    case 0:
      return(ABBInterpreter::setZoneManual(0, 0.0, 0.0, idCode));
    case 1:
      return(ABBInterpreter::setZoneManual(1, 0.3, 0.03, idCode));
    case 2:
      return(ABBInterpreter::setZoneManual(1, 1.0, 0.10, idCode));
    case 3:
      return(ABBInterpreter::setZoneManual(1, 5.0, 0.80, idCode));
    case 4:
      return(ABBInterpreter::setZoneManual(1, 10.0, 1.50, idCode));
    default:
      return(ABBInterpreter::setZoneManual(0, 0.0, 0.0, idCode));
    }
    }*/

/**
  * Formats message to set the zone mode of the ABB robot (distance from where to interpolate to the next destination).
  * @param fine Motion mode: 1 - Stop point. 0 - Fly by point.
  * @param tcp_mm linear distance from target point to begin to interpolate the position of the TCP (recommended = 5.0mm)
  * @param ori_mm linear distance from the target point to begin to interpolate the orientation of the TCP (recommended = 5.0mm)
  * @param ori_deg angular distance from the target point to begin to interpolate the orientation of the TCP (recommended = 1.0deg)
                   Hi.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setZone(bool fine, double tcp_mm, double ori_mm, double ori_deg, int idCode)
{
 char buff[20];
  string msg("09 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.1d ",fine);
  msg += buff ;
  sprintf(buff,"%.2lf ", tcp_mm);
  msg += buff ;
  sprintf(buff,"%.2lf ", ori_mm);
  msg += buff ;
  sprintf(buff,"%.2lf ", ori_deg);
  msg += buff ;
  msg += "#";

  return (msg);
}

/* Here lies the setBuffer code for position of end effector - Dont need it for joint control
string ABBInterpreter::setBuffer(PosList poslist) //Need help on this one, dont really understand it
{
	// pos ---  vector<double>
	// #include <vector>
	// typedef std::vector<double> Pos;
	// typedef std::vector<Pos> PosList;
	//
	// poslist[i][j]
	
	for (int i=1, i<sizeof(poslist), i++)
	{
		msg += addJointPosBuffer(poslist[i][1], poslist[i][2], poslist[i][3],
						         poslist[i][4], poslist[i][5], poslist[i][6]);
	}
	    
	
}
*/

string ABBInterpreter::addBuffer(double x, double y, double z, double q0, double qx, double qy, double qz, int idCode)
{
  //appends single target to the buffer
  // move will execute at current speed (which you can change between addBuffer calls)
  
  char buff[20];
  string msg("30 ");//instruction code;
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  
  sprintf(buff,"%+08.1lf ",x);  msg += buff;
  sprintf(buff,"%+08.1lf ",y);  msg += buff;
  sprintf(buff,"%+08.1lf ",z);  msg += buff;
  sprintf(buff,"%+08.5lf ",q0); msg += buff;
  sprintf(buff,"%+08.5lf ",qx); msg += buff;
  sprintf(buff,"%+08.5lf ",qy); msg += buff;
  sprintf(buff,"%+08.5lf ",qz); msg += buff;
  
  msg += "#";
  
  return (msg);
}


string ABBInterpreter::clearBuffer(int idCode)
{
  char buff[20];
  string msg("31 ");
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);

}



string ABBInterpreter::lenBuffer(int idCode)
{
  char buff[20];
  string msg("32 ");
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);		
}



string ABBInterpreter::executeBuffer(int idCode)
{
  char buff[20];
  string msg("33 ");
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";
  return (msg);
}


string ABBInterpreter::addJointPosBuffer(double q1, double q2, double q3, double q4, double q5, double q6, int idCode)
{
	char buff[20];
	string msg("37 ");
	sprintf(buff,"%.3d ",idCode); //identification code
	msg += buff;
	sprintf(buff,"%+08.1lf ",q1);  msg += buff;
	sprintf(buff,"%+08.1lf ",q2);  msg += buff;
    sprintf(buff,"%+08.1lf ",q3);  msg += buff;
    sprintf(buff,"%+08.1lf ",q4);  msg += buff;
	sprintf(buff,"%+08.1lf ",q5);  msg += buff;
    sprintf(buff,"%+08.1lf ",q6);  msg += buff;
    msg += "#";
	return (msg);
	
	/*    def addJointPosBuffer(self,joint_pos):
	#appends single joint position to the buffer
	if len(joint_pos) == 6:
            msg = "37 "
            msg = msg + format(joint_pos[0],"+08.2f")+" "+ format(joint_pos[1],"+08.2f")+" "+ format(joint_pos[2],"+08.2f")+" "+ format(joint_pos[3],"+08.2f")+" "+ format(joint_pos[4],"+08.2f")+" "+ format(joint_pos[5],"+08.2f")+" #"
            if self.verbose: print 'addJointBuffer:',msg
            self.robsock.send(msg)
            data = self.robsock.recv(self.BUFLEN)
            time.sleep(self.idel)
            return data
        else:
            return False
     */
 }
        
string ABBInterpreter::clearJointPosBuffer(int idCode)
{
	char buff[20];
	string msg("38 ");
	sprintf(buff,"%.3d ",idCode); //identification code
    msg += buff;
	msg += "#";
	return (msg);
	
	/*def clearJointPosBuffer(self):
         msg = "38 #"
         self.robsock.send(msg)
         data = self.robsock.recv(self.BUFLEN)
         return data
    */
}

/*
string ABBInterpreter::lenJointPosBuffer(int idCode)
{
	string msg("39 ");
	sprintf(buff,"%.3d ",idCode); //identification code
    msg += buff;
	msg += "#";
	return (msg);
	
}
*/

string ABBInterpreter::executeJointPosBuffer(int idCode)
{
	char buff[20];
	string msg("40 ");
	sprintf(buff,"%.3d ",idCode); //identification code
    msg += buff;
	msg += "#";
	return (msg);
	
	/*
	def executeJointPosBuffer(self):
        msg = "40 #"
        self.robsock.send(msg)
        data = self.robsock.recv(self.BUFLEN)
        return data
    */
}  

/**
  * Formats message to call special command.
  * @param command Number identifying the special command.
  * @param param1 General purpose parameter 1.
  * @param param2 General purpose parameter 2.
  * @param param3 General purpose parameter 3.
  * @param param4 General purpose parameter 4.
  * @param param5 General purpose parameter 5.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::specialCommand(int command, double param1, double param2, double param3, double param4, double param5, int idCode)
{
  char buff[20];
  string msg("10 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.1d ",command);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param1);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param2);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param3);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param4);
  msg += buff ;
  sprintf(buff,"%+09.2lf ", param5);
  msg += buff ;
  msg += "#";

  return (msg);
}

/**
  * Formats message to set the vacuum on/off.
  * @param vacuum 1-on 0-off.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::setVacuum(int vacuum, int idCode)
{
  char buff[20];
  string msg("11 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  sprintf(buff,"%.2d ",vacuum);
  msg += buff ;
  msg += "#";

  return (msg);
}


/**
  * Formats message to close the connection with the server in the ABB robot.
  * @param idCode User code identifying the message. Will be sent back with the acknowledgement.
  * @return String to be sent to ABB server.
  */
string ABBInterpreter::closeConnection(int idCode)
{
  char buff[20];
  string msg("99 ");//instruction code;
  
  sprintf(buff,"%.3d ",idCode); //identification code
  msg += buff;
  msg += "#";

  return (msg);
}

/**
  * Parser for the answer from the controller to the command getCartesian().
  * @param msg String message to parse.
  * @param x Placer for the X-coordinate of the ABB robot.
  * @param y Placer for the Y-coordinate of the ABB robot.
  * @param z Placer for the Z-coordinate of the ABB robot.
  * @param q0 Placer for the first component of the orientation quaternion of the ABB robot.
  * @param qx Placer for the second component of the orientation quaternion of the ABB robot.
  * @param qy Placer for the third component of the orientation quaternion of the ABB robot.
  * @param qz Placer for the fourth component of the orientation quaternion of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int ABBInterpreter::parseCartesian(std::string msg, double *x, double *y, 
    double *z,double *q0, double *qx, double *qy, double*qz)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf %lf",&idCode,&ok,x,y,z,q0,qx,qy,qz);
  if (ok)
    return idCode;
  else
    return -1;
}

/**
  * Parser for the answer from the controller to the command getJoints().
  * @param msg String message to parse.
  * @param joint1 Placer for the joint 1 of the ABB robot.
  * @param joint2 Placer for the joint 2 of the ABB robot.
  * @param joint3 Placer for the joint 3 of the ABB robot.
  * @param joint4 Placer for the joint 4 of the ABB robot.
  * @param joint5 Placer for the joint 5 of the ABB robot.
  * @param joint6 Placer for the joint 6 of the ABB robot.
  * @return Whether the message was received correctly or not by the ABB robot.
  */
int ABBInterpreter::parseJoints(std::string msg,  double *joint1, 
    double *joint2, double *joint3, double *joint4, 
    double *joint5, double *joint6)
{
  int ok, idCode;
  sscanf(msg.c_str(),"%*d %d %d %*f %lf %lf %lf %lf %lf %lf",&idCode,&ok,joint1,joint2,joint3,joint4,joint5,joint6);
  if (ok)
    return idCode;
  else
    return -1;
}
