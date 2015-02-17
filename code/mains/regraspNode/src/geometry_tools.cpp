#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
#include <regraspComm/regrasp_comm.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>
#include <matlab_comm/matlab_comm.h>
#include <math.h> 
#include "regraspNode/geometry_tools.h"

using namespace std;

GeometryTools::GeometryTools(vector<Vec> v)
{
  vertices = v;
  num_sides = v.size();

  if (num_sides == 0)
    {
      ROS_INFO("ERROR: bad vertices");
      return;
    }

  for (int i=0;i<(num_sides-1);i++)
    {
      side_lengths.push_back(dist(vertices[i][0], vertices[i][1], 
				  vertices[i+1][0], vertices[i+1][1]));
    }
  side_lengths.push_back(dist(vertices[num_sides-1][0], vertices[num_sides-1][1], 
			      vertices[0][0], vertices[0][1]));

}

void GeometryTools::rotate(Quaternion q)
{
  RotMat r = q.getRotMat();

  for (size_t i=0;i<vertices.size();i++)
    {
      vertices[i] = r*vertices[i];
    }

}

bool GeometryTools::findBoundary()
{
  // Find lowest surface and assume its on the table, so ignore it
  

  return true;
}

bool GeometryTools::findSideQuat(Quaternion q, Quaternion &grasp, double &angle)
{
  double a = (3.14159*(4.5/12.0));
  Quaternion yaw = q.angle2quat(0.0,0.0,a);
  grasp = q^yaw;

  Quaternion def = Quaternion("0.0 0.707 0.707 0.0");
  Quaternion tmp = q/def;
  double p1,p2;
  tmp.quat2angle(angle, p1, p2);

  return true;

}

bool GeometryTools::findPrincipalDir(Vec &dir, double &len)
{
  Vec cm(3);
  centroid(cm);
  vector<Vec> possible; 

  len = 0;
  dir = Vec("0 0 0",3);

  vector<Vec> tmp = vertices;
  tmp.push_back(vertices[0]);
  for (int i=0;i<(num_sides);i++)
    {
      Vec midpt = (tmp[i]+tmp[i+1])/2;
      double d = dist(cm[0], cm[1], midpt[0], midpt[1]);
      if (d > len)
	{
	  len = d;
	  dir = cm - midpt;
	}
      if (d == len)
	{
	  Vec tmp = cm - midpt;
	  cout << "tmp pc: " << tmp << endl;
	  // if multiple pcs, choose max y, min x (easier to grab)
	  if (tmp[0] > dir[0])
	    dir = tmp;
	  if (tmp[1] > dir[1])
	    dir = tmp;
	}
    }

  return true;
}

bool GeometryTools::findIntersect(Vec x1, Vec x2, Vec &i, double &d)
{
  d = INFINITY;
  for (int j=0;j<(num_sides-1);j++)
    {
      Vec intersect(3);
      if (findIntersect(vertices[j], vertices[j+1], x1, x2, intersect))
	{
	  double tmp = dist(x1[0],x1[1],intersect[0],intersect[1]);
	  if (tmp<d)
	    {
	      d = tmp;
	      i = intersect;
	    }
	}
    }

  Vec intersect(3);
  if (findIntersect(vertices[num_sides-1], vertices[0], x1, x2, intersect))
    {
      double tmp = dist(x1[0],x1[1],intersect[0],intersect[1]);
      if (tmp<d)
	{
	  d = tmp;
	  i = intersect;
	}
    }

  return true;
}

bool GeometryTools::findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i)
{
  double den = ((x1[0] - x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]));
  if (den == 0)
    return false;

  i[0] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[0]-x4[0]) - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]))/den;
  i[1] = ((x1[0]*x2[1] - x1[1]*x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]))/den;
  i[2] = 0;
  
  return true;

}

double GeometryTools::liftEdge(Vec dir)
{
  double edge= 0 ;
  double best = -INFINITY;
  for (int i=0;i<(num_sides-1);i++)
    {
      Vec side = vertices[i] - vertices[i+1];
      double dot = abs(dir*side);
      if (dot > best)
        {
          best = dot;
          edge = dist(vertices[i][0],vertices[i][1],
                      vertices[i+1][0],vertices[i+1][1]);
        }
    }
  Vec side = vertices[num_sides-1] - vertices[0];
  double dot = abs(dir*side);
  if (dot > best)
    {
      best = dot;
      edge = dist(vertices[num_sides-1][0],vertices[num_sides-1][1],
                  vertices[0][0],vertices[0][1]);

    }
  return edge;
}

bool GeometryTools::pointsIn(Vec pt, Vec dir)
{
  // find where line intersects plane of z=0
  double scalar = pt[2]/dir[2];
  Vec intersect = pt - dir*scalar;
  // cout << "pt: " << pt << endl;
  // cout << "dir: " << dir << endl;
  // cout << "scalar: " << scalar << endl;
  // cout << "intersect " << intersect << endl;

  // assuming some shit i think
  double x_min, y_min, x_max, y_max;
  boundingBox(x_min, y_min, x_max, y_max);

  if ((intersect[0] < x_max) && (intersect[0] > x_min) &&
      (intersect[1] < y_max) && (intersect[1] > y_min))
    return true;
  else
    return false;

}

bool GeometryTools::centroid(Vec &com)
{
  double x = 0;
  double y = 0;
  
  for (int i=0;i<num_sides;i++)
    {
      x += vertices[i][0];
      y += vertices[i][1];
    }

  x = x/num_sides;
  y = y/num_sides;

  com[0] = x;
  com[1] = y;
  com[2] = 0;

  return true;
}

double GeometryTools::area(void)
{
  if (regular())
    return (pow(side_lengths[0],2)*num_sides)/(4*tan(PI/num_sides));

  if (num_sides==4)
    {
      double width = std::max(side_lengths[0], side_lengths[2]);
      double height = std::max(side_lengths[1], side_lengths[3]);
      return (width*height);
    }
  
  ROS_INFO("ERROR: area for this shape not implemented yet");
  return 0;
  
}

double GeometryTools::max(vector<double> x)
{
  double m = x[0];
  for (unsigned int i=1;i<x.size();i++)
    {
      if (x[i] > m)
	m = x[i];
    }
  return m;
}

int GeometryTools::argmax(vector<double> x)
{
  double m = x[0];
  int ind = 0;
  for (unsigned int i=1;i<x.size();i++)
    {
      if (x[i] > m)
	{
	  m = x[i];
	  ind = i;
	}
    }
  return ind;
}

double GeometryTools::min(vector<double> x)
{
  double m = x[0];
  for (unsigned int i=1;i<x.size();i++)
    {
      if (x[i] < m)
	m = x[i];
    }
  return m;
}

int GeometryTools::argmin(vector<double> x)
{
  double m = x[0];
  int ind = 0;
  for (unsigned int i=1;i<x.size();i++)
    {
      if (x[i] < m)
	{
	  m = x[i];
	  ind = i;
	}
    }
  return ind;
}

bool GeometryTools::regular(void)
{
  bool tmp = side_lengths[0];
  for (int i=1;i<num_sides;i++)
    {
      if (side_lengths[i] != tmp)
	return false;
    }
  return true;
}

double GeometryTools::dist(double dX0, double dY0, double dX1, double dY1)
{
  return sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}

bool GeometryTools::boundingBox(double &x_min, double &y_min, 
				double &x_max, double &y_max)
{
  x_min = INFINITY;
  y_min = INFINITY;
  x_max = -INFINITY;
  y_max = -INFINITY;
  
  for (int i=0;i<num_sides;i++)
    {
      double x = vertices[i][0];
      double y = vertices[i][1];

      if (x < x_min)
	x_min = x;
      if (x > x_max)
	x_max = x;
      if (y < y_min)
	y_min = y;
      if (y > y_max)
	y_max = y;
    }
  
  return true;
}

vector<double> GeometryTools::getSideLengths(void)
{
  return side_lengths;
}

// int main () {
//   Vec a = Vec("1 0 0", 3);
//   Vec b = Vec("6 5 0", 3);
//   Vec c = Vec("5 6 0", 3);
//   Vec d = Vec("0 1 0", 3);
//   // Vec a = Vec("0 0 0", 3);
//   // Vec b = Vec("2 0 0", 3);
//   // Vec c = Vec("2 1 0", 3);
//   // Vec d = Vec("0 1 0", 3);
//   vector<Vec> vs;
//   vs.push_back(a);
//   vs.push_back(b);
//   vs.push_back(c);
//   vs.push_back(d);

//   GeometryTools gt(vs);

//   Vec cen(3);
//   double x_min,x_max, y_min, y_max;
//   gt.centroid(cen);

//   vector<double> sl = gt.getSideLengths();
//   cout << "side lengths " << sl[0] << " "<< sl[1] << " "<< sl[2] << " "<< sl[3] << endl;
//   cout << "area: " << gt.area() << endl;
//   cout << "regular? " << gt.regular() << endl;

//   gt.boundingBox(x_min,y_min,x_max,y_max);
//   cout << "bounding box: " <<  x_min << " " << y_min << " " << x_max << " " << y_max << endl;
  
//   Vec g = Vec("-1 -1 0",3);
//   cout << "liftEdge: " << gt.liftEdge(g) << endl;

//   Vec pt = Vec("5 5 1",3);
//   Vec dir = pt-cen;//Vec("-1 -0.5 -1",3);
//   cout << "pointsIn: " << gt.pointsIn(pt, dir) << endl;

//   Vec grasp = Vec("5 5 0",3);//Vec("1.5 0.5 0",3);
//   Vec tmp = Vec("6 6 0",3);
//   Vec i(3);
//   double distance;

//   //bool ret = gt.findIntersect(grasp,tmp,d,a,i);
//   //cout << "intersect: " << ret << " " << i << endl;
//   gt.findIntersect(grasp, tmp, i, distance);
//   cout << "dist " << distance << " pt: " << i << endl;
//   return 0;
// }

  // Vec findIntersect(Vec x1, Vec x2, Vec x3, Vec x4);


// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "geometry_tools");
//   ros::NodeHandle node;

//   vector<Vec> verts;
//   GeometryTools tools(verts);

//   // regrasper.advertiseServices();
//   // ROS_INFO("Regrasp server ready...");
//   ros::spin();

//   return 0;

// }
