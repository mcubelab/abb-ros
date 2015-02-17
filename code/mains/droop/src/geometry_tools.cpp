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
#include "droop/geometry_tools.h"

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

  Vec norm;
  bool ret = normal(norm);
  if (!ret)
    {
      cout << "ERROR: not enough points" << endl;
      return; 
    }

  if ((norm[0]==0) && (norm[1]==0)) // already in the xy plane
    return;

  

}

void GeometryTools::print()
{
  cout << "gt" << endl;
  for (size_t i=0;i<vertices.size();i++)
    cout << vertices[i] << endl;
}

bool GeometryTools::xyPlaneTransform(Vec n, RotMat &r)
{
  n.normalize();
  Vec v = Vec("0 0 1",3);

  // find axis
  Vec x = (n^v);
  x.normalize();
  // find angle
  double theta = acos((n*v)/(n.norm()*v.norm()));

  // find a rotation matrix 
  // a is a skew-symmetric matrix corresponding to x
  RotMat a = RotMat();
  a[0][0] = 0;
  a[0][1] = -x[2];
  a[0][2] = x[1];
  a[1][0] = x[2];
  a[1][1] = 0;
  a[1][2] = -x[0];
  a[2][0] = -x[1];
  a[2][1] = x[0];
  a[2][2] = 0;

  r = RotMat();
  r[0][0] = 1;
  r[1][1] = 1;
  r[2][2] = 1;

  r = r + a*sin(theta) + (a*a)*(1-cos(theta));

}

bool GeometryTools::normal(Vec &normal)
{
  if (vertices.size()<3)
    return false;
  
  Vec l1 = vertices[0] - vertices[1];
  Vec l2 = vertices[0] - vertices[2];
  normal = l1^l2;

}

bool GeometryTools::in(Vec pt)
{
  // Ray casting alg

  vector<Vec> v = vertices;
  v.push_back(vertices[0]);

  // cast a ray in any direction
  Vec pt_end = pt;
  pt_end[0] = pt_end[0]+500;

  int count = 0;
  for (size_t i=0;i<(v.size()-1);i++)
    {
      Vec intersect;
      cout << "1 " << v[i] << endl;
      cout << "2 " << v[i+1] << endl;
      cout << "3 " << pt << endl;
      cout << "4 " << pt_end << endl;
      cout << "" << endl;

      if (findIntersect(v[i],v[i+1],pt,pt_end,intersect))
	{
	  cout << "intersect " << intersect << endl;
	  count++;
	}
    }

  cout << "count " << count << endl;
  if (count %2 !=0)
    return true;
  else
    return false;

}

bool GeometryTools::convexHull()
{
  vector<Vec> pts = vertices; 

  // Jarvis March/wrapping
  int n = vertices.size();
  if (n<3)
    return false;
  
  // initialize result
  int k=0;
  vector<Vec> h(n);

  // sort points 
  sort(pts.begin(), pts.end());

  // build hull
  for (int i=0;i<n;i++)
    { 
      //cout << "k " << k << endl;
      while (k>=2 && xycross(h[k-2],h[k-1], pts[i])<=0)
	k--;

      //h[k++] = pts[i];
      h = insert(h,k++,pts[i]);
      // for (size_t q=0;q<h.size();q++)
      // 	cout << h[q] << endl;
      //h.insert(h.begin()+k+1,pts[i],1);
    }
  
  // // build upper hull
  // for (int i=n-2,t=k+1;i>=0;i++)
  //   {
  //     //i = (i+1)%n;
  //     while (k>=t && xycross(h[k-2],h[k-1], pts[i])<=0)
  // 	k--;
  //     h[k++] = pts[i];
  //   }

  // h.resize(k);
  // print result
  // cout << "hull " << endl;
  // for (size_t i=0;i<h.size();i++)
  //   {
  //     cout << h[i] << endl;
  //   }
  vertices = h;
}

vector<Vec> GeometryTools::insert(vector<Vec> v, int pos, Vec val)
{
  //cout << "pos " << pos << " val " << val << endl;
  vector<Vec> tmp = v;
  // move everything back 1
  for (size_t i=pos;i<v.size()-1;i++)
    {
      //cout << "i " << i << endl;
      v[i+1] = tmp[i];
    }
  v[pos] = val;

  return v;
}

double GeometryTools::xycross(Vec o, Vec p1, Vec p2)
{
  //cout << "butts " << o << " & " << p1 << " & " << p2 << endl;
  return (p1[0]-o[0])*(p2[1]-o[1]) - (p1[1]-o[1])*(p2[0]-o[0]);
}

//helper for convex hull
int GeometryTools::orientation(Vec p, Vec q, Vec r)
{
  int val = (q[1]-p[1])*(r[0]-q[0]) - (q[0]-p[0])*(r[1]-q[1]);
  if (val==0)
    return 0;
  return (val>0)? 1:2; //clock or counter clockwise
}

void GeometryTools::rotate(Quaternion q)
{
  RotMat r = q.getRotMat();
  rotate(r);

}

void GeometryTools::rotate(RotMat r)
{

  for (size_t i=0;i<vertices.size();i++)
    {
      vertices[i] = r*vertices[i];
    }

}

bool GeometryTools::findBoundary()
{


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
  // line1 endpoints: x1 & x2
  // line2 endpoints: x3 & x4

  double den = ((x1[0] - x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]-x4[0]));
  if (den == 0)
    return false;

  i = Vec(3);
  i[0] = trunc(10000.*((x1[0]*x2[1] - x1[1]*x2[0])*(x3[0]-x4[0]) - (x1[0]-x2[0])*(x3[0]*x4[1] - x3[1]*x4[0]))/den);
  i[1] = trunc(10000.*((x1[0]*x2[1] - x1[1]*x2[0])*(x3[1]-x4[1]) - (x1[1]-x2[1])*(x3[0]*x4[1] - x3[1]*x4[0]))/den);
  i[2] = 0;
  
  // check that i is within the line segments
  double x1_min = trunc(10000.*std::min(x1[0], x2[0]));
  double x2_min = trunc(10000.*std::min(x3[0], x4[0]));
  double y1_min = trunc(10000.*std::min(x1[1], x2[1]));
  double y2_min = trunc(10000.*std::min(x3[1], x4[1]));
  double x1_max = trunc(10000.*std::max(x1[0], x2[0]));
  double x2_max = trunc(10000.*std::max(x3[0], x4[0]));
  double y1_max = trunc(10000.*std::max(x1[1], x2[1]));
  double y2_max = trunc(10000.*std::max(x3[1], x4[1]));

  if ((i[0]>=x1_min) && (i[0]>=x2_min) &&
      (i[0]<=x1_max) && (i[0]<=x2_max) &&
      (i[1]>=y1_min) && (i[1]>=y2_min) &&
      (i[1]<=y1_max) && (i[1]<=y2_max))
    return true;
  else
    return false;

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
  com = Vec(3);

  double x = 0;
  double y = 0;
  double z = 0;
  
  for (int i=0;i<num_sides;i++)
    {
      x += vertices[i][0];
      y += vertices[i][1];
      z += vertices[i][2];
    }

  x = x/num_sides;
  y = y/num_sides;
  z = z/num_sides;

  com[0] = x;
  com[1] = y;
  com[2] = z;

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
