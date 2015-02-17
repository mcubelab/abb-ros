bool RegraspController::droopGeneral(regraspComm::Droop msg)
{
  double HAND_DEPTH = 200;
  
  vector<Vec> verts; // = msg.object_vertices;
  for (unsigned int i=0;i<msg.object_vertices.size();i+=3)
    {
      cout << "i " << i << endl;
      Vec v(3);
      v[0] = msg.object_vertices[i];
      v[1] = msg.object_vertices[i+1];
      v[2] = msg.object_vertices[i+2];
      verts.push_back(v);

    }

  cout << "size of verts: " << verts.size() << endl;
  GeometryTools gt(verts);

  //approach(msg.grasp_pose);
  Vec grasp(3);
  grasp[0] = msg.grasp_pose.position.x;
  grasp[1] = msg.grasp_pose.position.y;
  grasp[2] = msg.grasp_pose.position.z;  
  Quaternion q;
  q[0] = msg.grasp_pose.orientation.w;
  q[1] = msg.grasp_pose.orientation.x;
  q[2] = msg.grasp_pose.orientation.y;
  q[3] = msg.grasp_pose.orientation.z;

  // find the center of mass
  Vec com(3);
  gt.centroid(com);

  // find droop direction

  Vec dir = grasp - com;
  cout << "dir: " << dir << endl;

  // Droops (change in orientation) if vec from grasp to centroid 
  // does not point in the boundary
  bool willRotate = gt.pointsIn(grasp, dir);
  cout << "whats up: " << willRotate << endl;
  // Only rotates if obj fits in the hand
  double dist;
  Vec intersectionPt;
  Vec outwards = dir*-1;
  outwards[2] = 0;
  Vec inwards = outwards*-1;
  cout << "inwards: " << inwards << endl;
  gt.findIntersect(grasp, (grasp+outwards),intersectionPt, dist);
  cout << "intersectionPt: " << intersectionPt << " dist: " << dist << endl;
  if (dist >= HAND_DEPTH)
    willRotate = false;

  cout << "willRotate: " << willRotate << endl;

  if (willRotate)
    {
      // grasp the block
      Quaternion grasp_quat;
      double angle;      
      gt.findSideQuat(q,grasp_quat,angle);
      approach(grasp, grasp_quat);

      // lift (droop)
      double height = gt.liftEdge(inwards);
      robot.SetCartesian(msg.grasp_pose.position.x, 
  			 msg.grasp_pose.position.y, 
  			 msg.grasp_pose.position.z+height, 
  			 msg.grasp_pose.orientation.w,
  			 msg.grasp_pose.orientation.x,
  			 msg.grasp_pose.orientation.y,
  			 msg.grasp_pose.orientation.z);
      
      // place the object
      int steps = 5;
      double deltaX = (msg.grasp_pose.position.x - com[0])/steps;
      double deltaY = (msg.grasp_pose.position.y - com[1])/steps; 
      for (int i=0;i<steps;i++)
	{
	  robot.SetCartesian(msg.grasp_pose.position.x-(i*deltaX), 
			     msg.grasp_pose.position.y-(i*deltaY), 
			     msg.grasp_pose.position.z+(height-10*i), 
			     msg.grasp_pose.orientation.w,
			     msg.grasp_pose.orientation.x,
			     msg.grasp_pose.orientation.y,
			     msg.grasp_pose.orientation.z);
	  cout << "place pose:" << endl;
	  cout << msg.grasp_pose.position.x+(i*deltaX) << endl;
	  cout << msg.grasp_pose.position.y+(i*deltaY) << endl; 
	  cout << msg.grasp_pose.position.z+(height-5*i) << endl;
	  
	  hand.SetAngle(10);
	}
      ROS_INFO("General droop is completed!");

    }
  else
    {
      ROS_INFO("ERROR: Regrasp failed!");
      return false;
    }

  return true;

}
    
bool RegraspController::approach(Vec v, Quaternion q)
{
  ROS_INFO("In approach");

  double x = v[0]; //pose.position.x;
  double y = v[1]; //pose.position.y;
  double z = v[2]; //pose.position.z;
  double q0 = q[0]; //pose.orientation.w;
  double q1 = q[1]; //pose.orientation.x;
  double q2 = q[2]; //pose.orientation.y;
  double q3 = q[3]; //pose.orientation.z;

  // PROB: pose close to the table 
  // circular-esque set of waypoints
  // x^2/10^2 + y^2/100^2 = 1
  int n = 6;
  double t[6]= {0,2,4,6,8,10};
  double fingerAngles[6] = {65,55,45,35,25,15};

  cout << "starting" << endl;
  for (int i=n-1;i>=0;i--)
    {
      cout << "z " << ellipse(t[i])+z << endl;
      cout << "fingerAngle  " << fingerAngles[i] << endl; 
      robot.SetCartesian(x,y,z+ellipse(t[i]),q0,q1,q2,q3);
      hand.SetAngle(fingerAngles[i]);
      hand.WaitRest(0.25);
    }

 

  return true;
}

double RegraspController::ellipse(double t)
{
  return -10*sqrt(100-pow(t,2)) +100;
}
