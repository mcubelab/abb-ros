
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
#include "droop/Polygon.h"

using namespace std;

Polygon::Polygon(vector<Vec> v)
{
  n = v.size();
  
  // check that there are at least 3 points and that they make a coplanar "face"
  if (n<3)
    return;
  vertices = v;
  if (!coplanar())
    return;
  // find the normal
  normal = findNormal();

  // order the points 
  convexHull(v, vertices);

  // find centroid
  centroid = findCentroid();

}

void Polygon::print()
{
  cout << "Vertices: " << endl;
  for (size_t i=0;i<vertices.size();i++)
    cout << vertices[i] << endl;
}

bool Polygon::coplanar()
{
  Vec x1 = vertices[0];
  Vec x2 = vertices[1];
  Vec x3 = vertices[2];
  
  for (int i=3;i<n;i++)
    {
      Vec x4 = vertices[i];
      double ret = (x3-x1)*((x2-x1)^(x4-x3));
      if (ret>0.000001)
	return false;
    }
  return true;
}

bool Polygon::coplanar(Vec pt)
{
  Vec x1 = vertices[0];
  Vec x2 = vertices[1];
  Vec x3 = vertices[2];

  double ret = (x3-x1)*((x2-x1)^(pt-x3));
  if (fabs(ret)>0.000001)
    return false;
  else
    return true;

  // Vec nn = findNormal();
  // double mag = nn.norm();
  // double dist = (nn*pt + nn*vertices[0])/mag;
  // cout << "dist " << dist << endl;
  // if (fabs(dist) < 0.001)
  //   return true;
  // return false;

}

Vec Polygon::findNormal()
{  
  Vec l1 = vertices[0] - vertices[1];
  Vec l2 = vertices[0] - vertices[2];
  normal = l1^l2;
  normal.normalize();
  return normal;
}

bool Polygon::switchNormal(bool reverse)
{
  if (reverse)
    normal = -normal;
  
  return true;
}

Vec Polygon::findCentroid()
{
  Vec com = Vec(3);

  double x = 0;
  double y = 0;
  double z = 0;
  
  for (int i=0;i<n;i++)
    {
      x += vertices[i][0];
      y += vertices[i][1];
      z += vertices[i][2];
    }

  x = x/n;
  y = y/n;
  z = z/n;

  com[0] = x;
  com[1] = y;
  com[2] = z;

  return com;
}

Vec Polygon::operator[](const int i) const
{
  if ((i>=0) && (i<n))
    return vertices[i];
  else 
    {
      cout << "invalid vertex index" << endl;
      return Vec("0 0 0",3); // empty vector 
    }
}


bool Polygon::findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i)
{
  if ((x2-x1).parallel((x4-x3)))
    {
      i = Vec("-1000 -1000 -1000",3);
      return false;
    }

  Vec a = x2-x1;
  Vec b = x4-x3;
  Vec c = x3-x1;

  double dist  = (c*(a^b))/(a^b).norm();

  if (dist < 0.00001)
    {
      Mat tmp = Mat(3,2);
      tmp[0][0] = a[0];
      tmp[0][1] = b[0];
      tmp[1][0] = a[1];
      tmp[1][1] = b[1];
      tmp[2][0] = a[2];
      tmp[2][1] = b[2];
      Mat cm = Mat(c);
      cm = cm.transp();
      Mat itmp = tmp.pinv();
      Mat titmp = itmp.transp();
      Mat v = cm*titmp; // [t s] where x1+at or x3+bs
      
      i = x1 + a*v[0][0];
      
      double x1_min = min(x1[0],x2[0]);
      double x1_max = max(x1[0],x2[0]);
      double y1_min = min(x1[1],x2[1]);
      double y1_max = max(x1[1],x2[1]);
      double z1_min = min(x1[2],x2[2]);
      double z1_max = max(x1[2],x2[2]);
      double x2_min = min(x3[0],x4[0]);
      double x2_max = max(x3[0],x4[0]);
      double y2_min = min(x3[1],x4[1]);
      double y2_max = max(x3[1],x4[1]);
      double z2_min = min(x3[2],x4[2]);
      double z2_max = max(x3[2],x4[2]);
      double epsilon = 0.00001;

      if (((i[0]>=x1_min) || (abs(i[0]-x1_min)<epsilon)) &&
	  ((i[0]<=x1_max) || (abs(i[0]-x1_max)<epsilon)) &&
	  ((i[1]>=y1_min) || (abs(i[1]-y1_min)<epsilon)) &&
	  ((i[1]<=y1_max) || (abs(i[1]-y1_max)<epsilon)) &&
	  ((i[2]>=z1_min) || (abs(i[2]-z1_min)<epsilon)) &&
	  ((i[2]<=z1_max) || (abs(i[2]-z1_max)<epsilon)) &&
	  ((i[0]>=x2_min) || (abs(i[0]-x2_min)<epsilon)) &&
	  ((i[0]<=x2_max) || (abs(i[0]-x2_max)<epsilon)) &&
	  ((i[1]>=y2_min) || (abs(i[1]-y2_min)<epsilon)) &&
	  ((i[1]<=y2_max) || (abs(i[1]-y2_max)<epsilon)) &&
	  ((i[2]>=z2_min) || (abs(i[2]-z2_min)<epsilon)) &&
	  ((i[2]<=z2_max) || (abs(i[2]-z2_max)<epsilon)))
	return true;
      else
	{
	  i = Vec("-1000 -1000 -1000",3);
	  return false;
	}
    }
  else
    {
      i = Vec("-1000 -1000 -1000",3);
      return false;
    }
}

// bool Polygon::findIntersect(Vec x1, Vec x2, Vec x3, Vec x4, Vec &i)
// {
//   // line1 endpoints: x1 & x2
//   // line2 endpoints: x3 & x4
//   // cout << "edge " << x1 << x2 << endl;
//   // cout << "line " << x3 << x4 << endl;

//   Vec num = x1-x3;
//   Vec den  = (x4-x3) - (x2-x1);
//   Vec l1 = x2-x1;
//   Vec l2 = x4-x3;
//   Vec t = l1^l2;
//   //if (((l1*l2)==0) && (t[0]+t[1]+t[2])==0) // they are the same line
//   if ((t[0]+t[1]+t[2])==0) // they are the same line
//     {
//       //cout << "in here " << endl;
//       i = x3;
//     }
//   else
//     {
//       if ((den[0]!=0) && (den[1]!=0))
// 	{
// 	  cout << "in here " << endl;
// 	  double c = (-num[1]+((num[0]*l2[1])/l2[0]))/(l1[1]-((l2[1]*l1[0])/l2[0]));
// 	  //double c = (num[1]-(num[0]/l2[0]))/(-((l2[1]*l1[0])/l2[0])-l1[1]);
// 	  //cout << "c " << c << " l1 " << l1 << endl;
// 	  i = x1 + l1*c;
// 	  cout << "c " << c <<  endl;
// 	  cout << "l1 " << l1 << " l2 " << l2 << endl;
// 	  cout << "waht " << (-num[1]+((num[0]*l2[1])/l2[0])) << endl;
// 	  cout << "wut " << (l1[1]-((l2[1]*l1[0])/l2[0])) << endl;
// 	  cout << " i " << i << endl;
// 	}
//       else if ((den[0]!=0) && (den[2]!=0))
// 	{
// 	  //cout << "in here" << endl;
// 	  double c = (-num[2]+((num[0]*l2[2])/l2[0]))/(l1[2]-((l2[2]*l1[0])/l2[0]));
// 	  //double c = (num[2]-(num[0]/l2[0]))/(-((l2[2]*l1[0])/l2[0])-l1[2]);
// 	  //cout << "c " << c << endl;
// 	  i = x1 + l1*c;
// 	  //cout << "i " << i << endl;
// 	}
//       else if ((den[1]!=0) && (den[2]!=0))
// 	{
// 	  double c = (-num[2]+((num[1]*l2[2])/l2[1]))/(l1[2]-((l2[2]*l1[1])/l2[1]));
// 	  //double c = (num[2]-(num[1]/l2[1]))/(-((l2[2]*l1[1])/l2[1])-l1[2]);
// 	  i = x1 + l1*c;
// 	}
//       else
// 	{
// 	  //cout << "butts" << endl;
// 	  // cout << "line " << x3 << x4 << endl;
// 	  //cout << "den " << den << endl;
// 	  return false;
// 	}
//     }
//   // cout << "num " << num << endl;
//   // cout << "den " << den << endl;
//   // i = Vec(3);

//   // for (int j=0;j<3;j++)
//   //   {
//   //   if (den[j] == 0)
//   //     {
//   // 	cout << "j " << j << endl;
//   // 	if (abs(x1[j]-x3[j])<0.00001)
//   // 	  i[j] = x1[j];
//   // 	else
//   // 	  return false;
//   //     }
//   //   else
//   //     {
//   // 	double c = num[j]/den[j];
//   // 	i[j] = x1[j]+l1[j]*c;
//   //     }
//   //   }
//   // cout << "I " << i << endl;
//   // //return true;
  
//   // check that i is within the line segments
//   // double x1_min = trunc(10000.*std::min(x1[0], x2[0]));
//   // double x2_min = trunc(10000.*std::min(x3[0], x4[0]));
//   // double y1_min = trunc(10000.*std::min(x1[1], x2[1]));
//   // double y2_min = trunc(10000.*std::min(x3[1], x4[1]));
//   // double z1_min = trunc(10000.*std::min(x1[2], x2[2]));
//   // double z2_min = trunc(10000.*std::min(x3[2], x4[2]));
//   // double x1_max = trunc(10000.*std::max(x1[0], x2[0]));
//   // double x2_max = trunc(10000.*std::max(x3[0], x4[0]));
//   // double y1_max = trunc(10000.*std::max(x1[1], x2[1]));
//   // double y2_max = trunc(10000.*std::max(x3[1], x4[1]));
//   // double z1_max = trunc(10000.*std::max(x1[2], x2[2]));
//   // double z2_max = trunc(10000.*std::max(x3[2], x4[2]));

//   double x1_min = min(x1[0],x2[0]);
//   double x1_max = max(x1[0],x2[0]);
//   double y1_min = min(x1[1],x2[1]);
//   double y1_max = max(x1[1],x2[1]);
//   double z1_min = min(x1[2],x2[2]);
//   double z1_max = max(x1[2],x2[2]);

//   double x2_min = min(x3[0],x4[0]);
//   double x2_max = max(x3[0],x4[0]);
//   double y2_min = min(x3[1],x4[1]);
//   double y2_max = max(x3[1],x4[1]);
//   double z2_min = min(x3[2],x4[2]);
//   double z2_max = max(x3[2],x4[2]);


//   // if ((i[0]>=x1_min) && (i[0]>=x2_min) &&
//   //     (i[0]<=x1_max) && (i[0]<=x2_max) &&
//   //     (i[1]>=y1_min) && (i[1]>=y2_min) &&
//   //     (i[1]<=y1_max) && (i[1]<=y2_max) &&
//   //     (i[2]>=z1_min) && (i[2]>=z2_min) &&
//   //     (i[2]<=z1_max) && (i[2]<=z2_max))

//   //cout << "x1_min " << x1_min << " x1_max " << x1_max << endl;
//   //cout << "my " << min(x1[0],x2[0]) << " head " << max(x1[0],x2[0]) << endl;
  
//   double epsilon = 0.00001;
//   // cout << "a " << ((i[0]>=x1_min) || (abs(i[0]-x1_min)<epsilon)) << endl;
//   // cout << "b " << ((i[0]<=x1_max) || (abs(i[0]-x1_max)<epsilon)) << endl;
//   // cout << "c " << ((i[1]>=y1_min) || (abs(i[1]-y1_min)<epsilon)) << endl;
//   //cout << "edge " << x1 << x2 << endl;
//   //cout << "i " << i << " z1_min " << z1_min << " z1_max " << z1_max << endl;
//   // cout << "a " << (i[1]>=y1_min) << " " << (abs(i[1]-y1_min)<epsilon) << endl;
//   // cout << "b " << (i[1]<=y1_max) << " " << (abs(i[1]-y1_max)<epsilon) << endl;
//   // cout << "c " << (i[1]>=y2_min) << " " << (abs(i[1]-y2_min)<epsilon) << endl;
//   // cout << "d " << (i[1]<=y2_max) << " " << (abs(i[1]-y2_max)<epsilon) << endl;
  

//   if (((i[0]>=x1_min) || (abs(i[0]-x1_min)<epsilon)) &&
//       ((i[0]<=x1_max) || (abs(i[0]-x1_max)<epsilon)) &&
//       ((i[1]>=y1_min) || (abs(i[1]-y1_min)<epsilon)) &&
//       ((i[1]<=y1_max) || (abs(i[1]-y1_max)<epsilon)) &&
//       ((i[2]>=z1_min) || (abs(i[2]-z1_min)<epsilon)) &&
//       ((i[2]<=z1_max) || (abs(i[2]-z1_max)<epsilon)) &&
//       ((i[0]>=x2_min) || (abs(i[0]-x2_min)<epsilon)) &&
//       ((i[0]<=x2_max) || (abs(i[0]-x2_max)<epsilon)) &&
//       ((i[1]>=y2_min) || (abs(i[1]-y2_min)<epsilon)) &&
//       ((i[1]<=y2_max) || (abs(i[1]-y2_max)<epsilon)) &&
//       ((i[2]>=z2_min) || (abs(i[2]-z2_min)<epsilon)) &&
//       ((i[2]<=z2_max) || (abs(i[2]-z2_max)<epsilon)))
//     return true;
//   //if ((i[0]>=x1_min) && (i[0]<=x1_max) )&& 
//     //      (i[1]>=y1_min) && (i[1]<=y1_max) &&
//     // (i[2]>=z1_min) && (i[2]<=z1_max))
//   //    return true;
//   else
//     return false;

// }

bool Polygon::findEdgeIntersect(Vec o, Vec slope, Vec &i)
{
  if (!coplanar(o))
    {
      i = Vec("-1000 -1000 -1000",3);
      return false;
    }

  // find an end pt in the plane
  Vec pt = o+(slope)*100;
  if (!coplanar(pt))
    {
      i = Vec("-1000 -1000 -1000",3);
      return false;
    }

  vector<Vec> tmp = vertices;
  tmp.push_back(vertices[0]);
  Vec maybe;
  double closest = 100000;
  bool yes = false;
  for (int j=0;j<n;j++)
    if (findIntersect(tmp[j],tmp[j+1],o,pt,maybe))
      {
	Vec dist = o-maybe;
	if (dist.norm()<=closest)
	  {
	    i = maybe;
	    yes = true;
	  }
      }

  if (yes)
    return true;
  else
    {
      i = Vec("-1000 -1000 -1000",3);
      return false;
    }
}

Vec Polygon::projection(Vec v)
{
  // double d = normal*vertices[0];
  // double t = ((v*normal)+d)/(normal*normal);
  // Vec proj = v-normal*t;

  Vec d = v-vertices[0];
  double t = d*normal;
  Vec proj = v - normal*t;

  return proj;
}

bool Polygon::findPlanarAxes(Vec &x, Vec &y)
{
  // find an arbitrary axis orthogonal to the normal
  // here i use (y,-x,0)
  x = Vec(3);
  x[0] = normal[1];
  x[1] = -normal[0];
  x[2] = 0;
  
  // find third axis with cross product
  y = normal^x;

  return true;
}

bool Polygon::convexHull(vector<Vec> v, vector<Vec> &ordered_v)
{
  //prin();
  // Jarvis March/wrapping
  vector<Vec> pts = v; 

  // initialize result
  int k=0;
  vector<Vec> h(n);

  // sort points 
  sort(pts.begin(), pts.end());

  // build hull
  for (int i=0;i<n;i++)
    { 
      //cout << "k " << k << endl;
      while ((k>=2) && cross(h[k-2],h[k-1], pts[i]))
	k--;

      h = insert(h,k++,pts[i]);
    }
 
  ordered_v = h;
  return true;
}

vector<Vec> Polygon::insert(vector<Vec> v, int pos, Vec val)
{
  vector<Vec> tmp = v;
  // move everything back 1
  for (size_t i=pos;i<v.size()-1;i++)
    v[i+1] = tmp[i];
  v[pos] = val;

  return v;
}

double Polygon::cross(Vec o, Vec p1, Vec p2)
{
  Vec l1 = p1-o;
  Vec l2 = p2-o;  
  Vec c = l1^l2;
  double test = c*normal;
  if (test>=0)
    return true;
  return false;
}

bool Polygon::in(Vec pt)
{
  if (!coplanar(pt))
    return false;

  double d;
  if (onEdge(pt, d))
    return true;

  // Ray casting alg
  vector<Vec> v = vertices;
  v.push_back(vertices[0]);

  // cast a ray in any direction along the plane
  Vec x = Vec(3);
  if (normal[2]==0)
    {
      x[0] = normal[1];
      x[1] = -normal[0];
      x[2] = 0;
    }
  else if (normal[1]==0)
    {
      x[0] = normal[2];
      x[1] = 0;
      x[2] = -normal[0];
    }
  else
    {
      x[0] = 0;
      x[1] = normal[2];
      x[2] = -normal[1];
    }

  Vec y = x^normal;
  
  double tmp = (x+y).norm();
  Vec pt_end = pt + (x+y)*(200/tmp);
  if (!coplanar(pt_end))
    cout << "nope" << endl;

  int count = 0;
  int corner = 0;
  for (size_t i=0;i<(v.size()-1);i++)
    {
      // check if on the an edge by checking the distance to nearest pt on line
      //double d = ((pt-v[i])^(pt-v[i+1])).norm()/(v[i+1]-v[i]).norm();
      //      if (d<.0001)
      // if (onEdge(pt))
      // 	return true;

      Vec intersect;
      if (findIntersect(v[i],v[i+1],pt,pt_end,intersect))
	{
	  //cout << "intersect " << intersect << endl;
	  if ((intersect==v[i]) || (intersect==v[i+1]))
	    corner++;
	  count++;
	}
    }
  
  //cout << "count " << count << " corner " << corner << endl;
  if ((count %2 !=0) && (count!=0))
    return true;
  else if ((corner %4!=0) && (corner!=0))
    return true;
  else
    return false;
}

void Polygon::rotate(Quaternion q)
{
  Quaternion test = Quaternion("1 0 0 0");
  if (q==test)
    return;
  
  RotMat r = q.getRotMat();
  rotate(r);
}

void Polygon::rotate(RotMat r)
{
  for (int i=0;i<n;i++)
    vertices[i] = r*vertices[i];
  centroid = findCentroid();
  normal = findNormal();
}

void Polygon::translate(Vec v)
{
  for (int i=0;i<n;i++)
    vertices[i] = vertices[i]+v;
  centroid = findCentroid();
  normal = findNormal();
 
}

bool Polygon::ptIntersection(Vec o, Vec slope, Vec &intersection)
{
  if (coplanar(o))
    {
      if (in(o))
	{
	  intersection = o;
	  return true;
	}
      else 
	{
	  intersection = Vec("-1000 -1000 -1000",3);
	  return false;
	}
    }

  // find a second pt on the line
  Vec linept = o+slope;
  Vec l = linept-o;
  double d = normal*vertices[0];

  // Solve the equation:
  // nx*(ox + (lx-ox)*t)+ny*(oy + (ly-oy)*t)+nz*(oz + (lz-oz)*t)=0
  // -(nx*ox+ny*oy+nz*oz) = nx(lx-ox)*t+ny(ly-oy)*t+nz(lz-oz)*t
  // t = -(nx*ox+ny*oy+nz*oz) / nx(lx-ox)+ny(ly-oy)+nz(lz-oz)
  if ((normal*l)==0)
    {
      intersection = Vec("-1000 -1000 -1000",3);
      return false;
    }
  double t = (-(normal*o)+d) / (normal*l);
  if (t<0)
    {
      intersection = Vec("-1000 -1000 -1000",3);
      return false;
    }

  intersection = Vec(3);
  intersection[0] = o[0] + (linept[0] - o[0])*t;
  intersection[1] = o[1] + (linept[1] - o[1])*t;
  intersection[2] = o[2] + (linept[2] - o[2])*t;

  for (int i=0;i<3;i++)
    if (isnan(intersection[i]) || isinf(intersection[i]))
      {
	intersection = Vec("-1000 -1000 -1000",3);
        return false;
      }

  if (in(intersection))
    return true;
  else
    {
      intersection = Vec("-1000 -1000 -1000",3);
      return false;
    }
}

bool Polygon::ptPlaneDist(Vec v, double &dist)
{
  //  cout << "normal " << normal << endl;
  double d = normal*vertices[0];
  double num = (v*normal) + d;
  double den = normal.norm(); // should be one...
  // cout << "dot " << (v*normal) << " d " << d << endl;
  // cout << "den " << den << endl;
  dist = num/den;
  return true;

}

bool Polygon::onEdge(Vec v, double &d)
{
  vector<Vec> tmp = vertices;
  tmp.push_back(vertices[0]);
  for(int i=0;i<n;i++)
    {
      Vec num = (v-tmp[i])^(v-tmp[i+1]);
      Vec den = tmp[i+1]-tmp[i];
      d = num.norm()/den.norm();
      
      if (d<0.00001)
	{
	  double x1_min = min(tmp[i][0],tmp[i+1][0]);
	  double x1_max = max(tmp[i][0],tmp[i+1][0]);
	  double y1_min = min(tmp[i][1],tmp[i+1][1]);
	  double y1_max = max(tmp[i][1],tmp[i+1][1]);
	  double z1_min = min(tmp[i][2],tmp[i+1][2]);
	  double z1_max = max(tmp[i][2],tmp[i+1][2]);
	  double epsilon = 0.00001;
	  
	  if (((v[0]>=x1_min) || (abs(v[0]-x1_min)<epsilon)) &&
	      ((v[0]<=x1_max) || (abs(v[0]-x1_max)<epsilon)) &&
	      ((v[1]>=y1_min) || (abs(v[1]-y1_min)<epsilon)) &&
	      ((v[1]<=y1_max) || (abs(v[1]-y1_max)<epsilon)) &&
	      ((v[2]>=z1_min) || (abs(v[2]-z1_min)<epsilon)) &&
	      ((v[2]<=z1_max) || (abs(v[2]-z1_max)<epsilon)))
	    return true;
	}
    }
  return false;
}

bool Polygon::operator == (const Polygon &p) const
{
  vector<Vec> tmp = vertices;
  tmp.push_back(vertices[0]);
  vector<Vec> ptmp = p.getVertices();
  ptmp.push_back(ptmp[0]);
  vector<double> edges;
  vector<double> pedges;
  if (n==p.size()) // same number of vertices
    {
      // check if sides the same length
      for (int i=0;i<n;i++)
	{
	  edges.push_back((tmp[i]-tmp[i+1]).norm()); 
	  pedges.push_back((ptmp[i]-ptmp[i+1]).norm());
	}
      sort(edges.begin(),edges.end());
      sort(pedges.begin(),pedges.end());
      for (int i=0;i<edges.size();i++)
	{
	  if (fabs(edges[i]-pedges[i])>0.00001)
	    return false;
	}
      return true;
    }
  return false;
}
