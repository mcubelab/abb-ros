#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <hand_comm/hand_comm.h>
#include <geometry_msgs/Pose.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>
#include <matlab_comm/matlab_comm.h>
#include <math.h> 
#include "droop/Polygon.h"
#include "droop/Polyhedron.h"
#include <iostream>
#include <fstream>
#include <string>

using namespace std;

Polyhedron::Polyhedron(vector<Polygon> f)
{

  faces = f;
  n = faces.size();
  centroid = findCentroid();
  rotation = Quaternion("1 0 0 0");
  alignNormals();
  obj = 0;
}

Polyhedron::Polyhedron(const Polyhedron &orig)
{
  vector<Polygon> f = orig.getFaces();
  faces = f;
  n = faces.size();
  centroid = findCentroid();
  rotation = Quaternion("1 0 0 0");
  alignNormals();
  obj = 0;
}

Polyhedron::Polyhedron(int obj_num)
{
  vector<Vec> verts; 
  vector<Vec> surfaces;
  readModel(obj_num, verts, surfaces);
  findFaces(verts, surfaces);

  //cout << "OBJ " << obj_num << endl;

  n = faces.size();
  //Vec tmp = findCentroid();
  centroid = findCentroid();//new Vec(tmp); //&tmp;
  obj = obj_num;
  rotation = Quaternion("1 0 0 0");
  alignNormals();
}

void Polyhedron::print()
{
  cout << "faces: " << endl;
  for (int i=0;i<n;i++)
    faces[i].print();

}

vector<Vec> Polyhedron::getVertices()
{
  vector<Vec> v, v_final;
  for (int i=0;i<n;i++)
    for (int j=0;j<faces[i].size();j++)
      v.push_back(faces[i].getVertices()[j]);
 
  // remove duplicates
  for (size_t i=0;i<v.size();i++)
    {
      bool dup = false;
      for (size_t j=0;j<v_final.size();j++)
	if ((v[i]==v_final[j]) && (i!=j))
	  dup = true;
      if (!dup)
	v_final.push_back(v[i]);
    }

  return v_final;
}

Vec Polyhedron::findCentroid()
{
  vector<Vec> v = getVertices();
  Vec com = Vec(3);

  double x = 0;
  double y = 0;
  double z = 0;
  
  for (size_t i=0;i<v.size();i++)
    {
      x += v[i][0];
      y += v[i][1];
      z += v[i][2];
    }

  x = x/v.size();
  y = y/v.size();
  z = z/v.size();

  com[0] = x;
  com[1] = y;
  com[2] = z;

  return com;

}

double Polyhedron::findMoment(Vec grasp)
{
  double moment = 0;
  mass = 1;

  // use know moments and offset to grasp using parallel axis theorem
  // I = I_axis + md^2 where d is dist from real axis to 'axis'

  cout << "obj " << obj << " tri " << RecObj::BIG_TRIANGLE << endl;

  if (obj==RecObj::BIG_TRIANGLE)
    {
      // I=(1/6)md^2 (where d is length of hypo (d~144)
      double i_x = (1/6.0)*mass*(pow(0.144,2));
      cout << "ix " << i_x << endl;

      // use parallel axis theorem
      vector<Vec> v = faces[2].getVertices();
      Vec axis_pt =v[0];
      for (int i=1;i<v.size();i++)
	axis_pt = axis_pt+v[i];
      axis_pt = axis_pt/v.size();
      Vec p = axis_pt-grasp;
      double dist = sqrt(p*p);
      dist = dist/1000;
      moment = i_x + mass*pow(dist,2);
    }
  // else if (obj==RecObj::CYLINDER) // I =(1/4)mr^2+(1/3)ml^2
  //   {

  //   }
  // else if (obj==RecObj::RECTANGLE) 
  //   {
  //     // I=(1/12)m(x^2+y^2)
  //     double i_x = (1/12)*mass*(
  //   }

  return moment;
}

void Polyhedron::rotate(Quaternion q)
{
  Quaternion test = Quaternion ("1 0 0 0");
  if (q==test)
    return;

  rotation = q*rotation;
  RotMat r = q.getRotMat();
  rotate(r);  
}

void Polyhedron::rotate(RotMat r)
{
  Quaternion q = r.getQuaternion();
  rotation = q*rotation;
  for (int i=0;i<n;i++)
    faces[i].rotate(r);

  centroid = findCentroid();
  alignNormals();
}


void Polyhedron::translate(Vec v)
{
  for (int i=0;i<n;i++)
    faces[i].translate(v);
  
  centroid = findCentroid();
}

bool Polyhedron::findSupportFace(Vec grasp, int &face_index)
{
  // cout << "grasp " << grasp << endl;
  // cout << "kk " << (grasp==centroid) << endl;
  // cout << "LL " << (in(centroid)) << endl;
  if (!in(grasp))
    {
      cout << "ERROR: grasp outside object" << endl;
      face_index = -1000;
      return false;
    }

  // find the direction of the force
  if (centroid==grasp)
    grasp[2] = grasp[2]+1;
  Vec line = centroid - grasp;
  //cout << "grasp " << grasp << " line " << line << endl;

  for (int i=0;i<n;i++)
    {
      // find the point on the surface where the line from the 
      // grasp to the center of mass intersects that surface plane
      Vec intersection;
      bool ret = faces[i].ptIntersection(grasp, line, intersection);
      //faces[i].print();
      //cout << "ret " << ret << " intersection " << intersection << endl;
      // find if that point is within the face 
      if (ret)
        {
          bool inret = faces[i].in(intersection);
	  //cout << "inret " << inret << endl;
	  if (inret)
            {
              face_index = i;
              return true;
            }
        }
    }
  cout << "ERROR: did not find support face" << endl;
  face_index =-1000;
  return false;
}

int Polyhedron::readModel(int obj, vector<Vec> &verts, 
                          vector<Vec> &surfaces)
{
  string intro = "/home/simplehands/Documents/hands/code/nodes/vision/ROS/objRec_node/objectFolder/vert_files";
  //string intro = "/home/annie/ros/sandbox/hands/code/nodes/vision/ROS/objRec_node/objectFolder/vert_files";
  string tri = intro + "/big_triangle.txt";
  string rect = intro + "/rectangle.txt";
  string cyl = intro + "/cylinder.txt";
  string lrect = intro + "/long_block.txt";

  string delimiter = ",";
  string new_sur = "!";

  Vec sur = Vec("0 0 0",3);
  int c = 0;

  string line;
  ifstream * myfile;// (tri.c_str());

  // if (obj==RecObj::BIG_TRIANGLE)
  //   ifstream myfile (tri.c_str());
  // else if (obj==RecObj::CYLINDER)
  //   ifstream myfile (cyl.c_str());
  if (obj==RecObj::BIG_TRIANGLE)
    myfile = new ifstream (tri.c_str());
  else if (obj==RecObj::CYLINDER)
    myfile = new ifstream (cyl.c_str());
  else if (obj==RecObj::RECTANGLE)
    myfile = new ifstream (rect.c_str());
  else if (obj==RecObj::LONG_BLOCK)
    myfile = new ifstream (lrect.c_str()); 
  else
    {
      cout << "ERROR: bad object" << endl;
      return 0;
    }
  
  if (myfile->is_open())
    {
      while ( getline (*myfile,line) )
	{
	  if (line.compare(new_sur)==0) // new surface started
	    {
	      surfaces.push_back(sur);
	      c = 0;
	    }
	  else
	    {
	      // read in line for a vertex
	      line = line.substr(0, line.size()-1);
	      size_t pos = 0;
	      string token;
	      int i = 0;
	      Vec v = Vec(3);
	      while ((pos = line.find(delimiter)) != string::npos) 
		{
		  token = line.substr(0, pos);
		  //cout << token << endl;
		  double tmp = ::atof(token.c_str());
		  v[i] = tmp;
		  i++;
		  line.erase(0, pos + delimiter.length());
		}
	      //cout << line << endl;
	      double tmp = ::atof(line.c_str());
	      v[i] = tmp;
	      
	      int ind;
	      bool ret = find(verts, v, ind);
	      if (ret)
		{
		  sur[c] = ind;
		  c++;
		}
	      else
		{
		  verts.push_back(v);
		  sur[c] = verts.size()-1;
		  c++;
		}
	    }
	}
      myfile->close();
      //delete[] myfile;
    }
  else cout << "Unable to open file"; 
 
  surfaces.erase(surfaces.begin());
  
  return 0;
}

bool Polyhedron::findFaces(vector<Vec> verts, vector<Vec> tri_ind)
{
  vector< vector < int > > surfaces;
  vector< vector < int > > sur;
  for (size_t i=0;i<tri_ind.size();i++)
    {
      vector<int> cur;
      cur.push_back(tri_ind[i][0]);
      cur.push_back(tri_ind[i][1]);
      cur.push_back(tri_ind[i][2]);

      for (size_t j=0;j<tri_ind.size();j++)
	{
	  int pt;
	  bool ret = edge(tri_ind[i],tri_ind[j], pt);
	  if (ret)
	    {

	      bool yes = coplanar(verts[tri_ind[i][0]],verts[tri_ind[i][1]],
				  verts[tri_ind[i][2]],verts[pt]);
	      if (yes)
		{
		  cur.push_back(pt);
		  //tri_ind.erase(tri_ind.begin()+j);
		}
	    }	  
	}
      sur.push_back(cur);
    }


  for (size_t i=0;i<sur.size();i++)
    {
      bool dup = false;
      for (size_t j=0;j<surfaces.size();j++)
	{
	  if (permutation(sur[i],surfaces[j]))
	    dup = true;
	}
      if (!dup)
	surfaces.push_back(sur[i]);
    }

  n = surfaces.size();
  for (size_t i=0;i<surfaces.size();i++)
    {
      vector<Vec> sur;
      for (size_t j=0;j<surfaces[i].size();j++)
	sur.push_back(verts[surfaces[i][j]]);
      Polygon face(sur);
      faces.push_back(face);
    }

  return true;
}

bool Polyhedron::permutation(vector<int> t1, vector<int> t2)
{
  if (t1.size() != t2.size())
    return false;
  
  sort(t1.begin(), t1.end());
  sort(t2.begin(), t2.end());
  
  for (size_t i=0;i<t1.size();i++)
    {
      if (t1[i]!=t2[i])
	return false;
    }
  return true;

}

bool Polyhedron::coplanar(Vec x1, Vec x2, Vec x3, Vec x4)
{
  double ret = (x3-x1)*((x2-x1)^(x4-x3));
  if (ret==0)
    return true;
  else
    return false;
}

bool Polyhedron::edge(Vec v1, Vec v2, int &pt)
{

  int matches = 0;
  if (v1.in(v2[0]))
    matches++;
  else
    pt = v2[0];
  if (v1.in(v2[1]))
    matches++;
  else
    pt = v2[1];
  if (v1.in(v2[2]))
    matches++;
  else
    pt = v2[2];

  if (matches==2)
    return true;
  else
    return false;
}

bool Polyhedron::find(vector<Vec> arr, Vec elem, int &ind)
{

  for (size_t i=0;i<arr.size();i++)
    {
      if ((arr[i][0]==elem[0]) && (arr[i][1]==elem[1]) && (arr[i][2]==elem[2]))
	{
	  ind = i;
	  return true;
	}
    }
  return false;
}

// TODO
bool Polyhedron::findGraspRotation(Vec grasp, Quaternion &q) 
{
  int current;
  int next;
  Vec tmp = centroid;
  tmp[2] = tmp[2]+1;
  cout << findSupportFace(tmp, current) << endl;
  cout << "current " << current << endl;
  cout << "grasp " << grasp << endl;
  cout << findSupportFace(grasp, next) << endl;
  
  // since rotation due to grasp, will only be about a single axis (torque axis)
  Vec r = centroid - grasp;
  Vec f = Vec("0 0 -1",3);
  Vec torque = r^f;
  torque.normalize();

  cout << "torque " << torque << endl;

  // find rotation of normal of new support face to (0,0,1)
  cout << "current " << current << endl;
  faces[current].print();
  cout << "cur normal " << faces[current].getNormal();
  cout << "next " << next << endl;
  faces[next].print();
  cout << "next normal " << faces[next].getNormal();
  Vec old = faces[current].getNormal(); // should be (0,0,1)
  Vec rot = faces[next].getNormal();
  double dot = old*rot;
  double angle;
  if (dot>1)
    angle = acos(1);
  else
    angle = acos(old*rot);
  
  cout << "angle " << angle << endl;

  RotMat transf;
  transf.setAxisAngle(torque, angle);
  q = transf.getQuaternion();

  cout << "axis " << torque << endl;
  cout << "angle " << angle << endl;

  return true;
}

bool Polyhedron::getFaceGrasp(int face, Vec &grasp)
{
  // ideal grasp should be from the face centroid through
  // obj centroid to the other side of the obj
  Vec faceCM = faces[face].getCentroid();
  Vec dir = centroid - faceCM;
  
  // now find where that line hits teh boundary of the obj
  for (int i=0;i<n;i++)
    {
      if (i!=face)
	{
	  Vec intersect(3);
	  bool ret = faces[i].ptIntersection(faceCM, dir, intersect);
	  if ((ret) && (faces[i].in(intersect)))
	    {
	      grasp = intersect;
	      return true;
	    }
	}
    }
  return false;
}

bool Polyhedron::testgraspable(Vec grasp, int &grasp_face, Vec &new_grasp, bool &give_up)
{
  double FINGER_WIDTH = 70;
  double HAND_DEPTH = 50;
  double Z_THRESH = bottom() + 15;
  double EDGE_MARGIN = 15;

  int face;
  findSupportFace(centroid,face);
  //faces[face].print();

  // find dist block will rotate in hand
  Vec ln = faces[face].projection((grasp-centroid));
  ln.normalize();
  Vec flat_centroid = faces[face].projection(centroid);
  Vec worst(3);
  faces[face].findEdgeIntersect(flat_centroid, ln, worst);
  Vec rot = grasp-worst;
  double rotDist = rot.norm();
  //cout << "rotDist " << rotDist << endl;

  // find dist to edge/surface
  int close = 0;
  double edgeDist = 10000;
  double surDist = 10000;

  for (int i=0;i<n;i++)
    {
      double maybe;
      faces[i].onEdge(grasp, maybe);
      if ((maybe<edgeDist) && (i!=face))
	{
	  edgeDist = maybe;
	  close = i;
	  surDist = (faces[i].projection(grasp)-grasp).norm();
	}
      // double maybes = (faces[i].projection(grasp)-grasp).norm();
      // //cout << "surdist " << maybes << endl;
      // if ((maybe<surDist) && (i!=face))
      // 	{
      // 	  surDist = maybes;
	  
      // 	}
    }

  //cout << "close " << close << " dist " << edgeDist << " surDist " << surDist << endl;
  // find dist to nearest surface


  // find a pair of parallel faces
  for (int i=0;i<n;i++)
    for(int j=(n-1);j>i;j--)
      {
	Vec v1 = faces[i].getNormal();
	Vec v2 = faces[j].getNormal();
	Vec ground = Vec("0 0 1",3);

	// cout << "i " << i << " j " << j << endl;
	// cout << "norm " << faces[i].getNormal() << endl;
	// cout << "b " << (v1.parallel(v2)) << " a " << (faces[i].getNormal().parallel(ground)) << endl;

	if ((i!=j) && (v1.parallel(v2)) && (!faces[i].getNormal().parallel(ground)))
	  {            
	    // Decide if it fits in the gripper
	    Vec tmp = faces[i].getCentroid()-faces[j].getCentroid();
	    double dist = tmp.norm();
	    grasp_face = i;
	    if (dist<FINGER_WIDTH)
	      {
		bool good = true;
		new_grasp = grasp;

		// check if too close to the ground
		if (new_grasp[2]<Z_THRESH)
		  {
		    new_grasp[2] += 5;
		    //cout << "too low" << endl;
		    good = false;
		  }
		
		// check if rotation fits in the hand
		if (rotDist>=HAND_DEPTH)
		  {
		    // move in the direction of the worst pt
		    Vec dir = (worst-new_grasp);
		    dir.normalize();
		    new_grasp += dir*5;
		    //cout << "too in" << endl;
		    good = false;
		  }

		// check if too close to an edge
		if ((edgeDist<EDGE_MARGIN) || (surDist<EDGE_MARGIN))
		  {
		    new_grasp += (-faces[close].getNormal())*5;;
		    //cout << "too close to an edge" << endl;
		    good = false;
		  }

		if (!in(new_grasp))
		  {
		    give_up = true;
		    cout << "ERROR:  GRASP IMPOSSIBLE" << endl;
		  }
		
		give_up = false;
		//cout << "new grasp " << new_grasp << endl;
		return good;
	      }
	  }
      }

  cout << "block bad shape/orientation" << endl;
  return false;
}

bool Polyhedron::findApproach(Vec grasp, int grasp_face, Quaternion &q)
{
  // point at the centroid from the grasp
  // From the frame of the robotiq hand, we want:
  // z axis pointing in the normal to the closest surface
  // y axis in the normal to the grasp surfaces

  Vec x(3);
  Vec y = faces[grasp_face].getNormal();
  Vec ground = Vec("0 0 1",3);
  
  int support;
  findSupportFace(centroid,support);
  int close = 0;
  double edgeDist = 10000;
  for (int i=0;i<n;i++)
    {
      double maybe;
      faces[i].ptPlaneDist(grasp, maybe);
      // if ((maybe<edgeDist) && (!ground.parallel(faces[i].getNormal())) && 
      // 	  (!y.parallel(faces[i].getNormal())))
      if ((maybe<edgeDist) && (i!=support) && 
	  (!y.parallel(faces[i].getNormal())))
	{
	  edgeDist = maybe;
	  close = i;
	}
    }

  Vec z = -faces[close].getNormal() - ground;
  z.normalize();

  //  cout << "grasp face " << grasp_face << " normal " << y << endl;
  //cout << "close " << close << " normal " << faces[close].getNormal() << endl;
  //cout<< "y " << y << " z " << z << endl;
  x = y^z;

  x.normalize();
  y.normalize();
  z.normalize();

  RotMat r(x,y,z);
  q = r.getQuaternion();

  return true;
}

bool Polyhedron::pick(int goal_face, Vec &orig_grasp, Vec &usable_grasp, Quaternion &q)
{
  vector<int> try_faces = getSymmetricFaces(goal_face);
  int support;
  findSupportFace(centroid, support);
  Vec grasp;
  
  for (int j=0;j<1;j++)//try_faces.size();j++)
    {
      int support;
      findSupportFace(centroid,support);
      if (try_faces[j]!=support)
	{
	  goal_face = try_faces[j];
	  int grasp_face;

	  getFaceGrasp(goal_face,grasp);
	  // cout << "real orig " << grasp << endl;
	  // Vec wt = faces[support].projection((centroid-grasp));
	  // grasp += ((centroid-grasp)*0.15 + wt*0.65);
	  orig_grasp = grasp;
	  
	  if (grasp.nn!=3)
	    return false;
	  
	  // Vec new_grasp;
	  // bool can = graspable(grasp, grasp_face,new_grasp);
	  // cout << "Graspable? " << can << " new " << new_grasp << endl;
	  
	  bool can = false;  
	  int count = 0;
	  bool give_up;
	  while ((!can) && (count<100))
	    {
	      //cout << "grasp " << grasp << endl;
	      Vec new_grasp;
	      can = testgraspable(grasp, grasp_face,new_grasp, give_up);
	      if (give_up)
		break;

	      grasp = new_grasp;
	      usable_grasp = grasp;
	      count++;
	    }
	  if (count>100)
	    return false;
	  
	  findApproach(grasp, grasp_face, q);
	}
    }

  return true;
}

bool Polyhedron::findRotAxis(Vec grasp, Vec &axis)
{
  int current;
  int next;
  findSupportFace(centroid, current);
  findSupportFace(grasp, next);
  
  // since rotation due to grasp, will only be about a single axis (torque axis)
  Vec r = centroid - grasp;
  Vec f = Vec("0 0 -1",3);
  axis = r^f;
  axis.normalize();

  return true;
}

bool Polyhedron::findFaceAngle(int face, double &angle)
{
  int current;
  findSupportFace(centroid, current);
  Vec old = faces[current].getNormal();
  Vec rot = faces[face].getNormal();

  double dot = old*rot;
  if (dot>1)
    angle = acos(1);
  else
    angle = acos(old*rot);

  return true;

}

bool Polyhedron::findRotationAngle(Vec grasp, int goal, double &angle)
{
  // first find the support face using that grasp
  int grasp_face;
  findSupportFace(grasp, grasp_face);
  int current;
  findSupportFace(centroid, current);
  angle = 0;
  cout << "grasp face " << grasp_face << endl;

  if (grasp_face!=goal) // find how much more it must rotate to reach goal
    {
      // check all the faces where the normals are coplanar with 
      // current & grasp_face and find their range of angles
      Vec x1 = faces[current].getCentroid();
      Vec x2 = x1 + faces[current].getNormal();
      Vec x3 = faces[grasp_face].getCentroid();
      Vec x4 = faces[goal].getCentroid();
      
      double ret = (x3-x1)*((x2-x1)^(x4-x3));
      cout << "ret " << ret << endl;
      if (ret<0.000001)
	{
	  // find the angle between the line from grasp-centroid 
	  // and the line grasp-edge (edge of goal face)
	  Vec line = centroid - grasp;
	  line.normalize();

	  // find the correct edge (between goal and grasp_face)
	  vector<Vec> goal_verts = faces[goal].getVertices();
	  vector<Vec> grasp_verts = faces[grasp_face].getVertices();
	  vector<Vec> edge;
	  for (int i=0;i<goal_verts.size();i++)
	    {
	      for (int j=0;j<grasp_verts.size();j++)
		{
		  if (goal_verts[i] == grasp_verts[j])
		    edge.push_back(goal_verts[i]);
		}
	    }
	  Vec pt = Vec("0 0 0",3);
	  for (int i=0;i<edge.size();i++)
	    pt = pt+edge[i];
	  pt = pt/edge.size();
	  cout << "pt " << pt << endl;
	  Vec line2 = pt - grasp;
	  line2.normalize();

	  cout << "ln " << line << " ln2 " << line2 << endl;

	  // find angle
	  Vec cross = line^line2;
	  angle = asin(cross.norm());
	}

    }

  return true;
}
 
bool Polyhedron::lift(Vec orig_grasp, Vec usable_grasp, Quaternion q, 
		      int goal_face, vector<geometry_msgs::Pose> &waypts)
{
  // find the current support face
  int face;
  Vec tmp = centroid;
  tmp[2] = tmp[2] + 1;
  findSupportFace(tmp,face);

  // find the dist to the farthest pt on the support face from the grasp: lift_dist
  Vec s = centroid - usable_grasp;
  s.normalize();
  // find the projection of this line and the centroid on the surface
  Vec ln = faces[face].projection(s);
  ln.normalize();
  Vec flat_centroid = faces[face].projection(centroid);
  Vec pivot(3);
  faces[face].findEdgeIntersect(flat_centroid, ln, pivot);
  Vec pivotV = usable_grasp - pivot;
  double pivot_dist = pivotV.norm()+15;

  //faces[face].print();
  // Polygon p = faces[face];
  // cout << "parallel? " << (p[0]-p[1]).parallel(ln) << endl;
  // cout << "edge " << (p[0]-p[1]) << endl;
  // cout << "ln " << ln << endl;
  // cout << "centroid " << centroid << " flat centroid " << flat_centroid << endl;
  // cout << "pivot " << pivot << " dist " << pivot_dist << endl;

  // find dist from grasp to goal support face: set_dist
  Vec intersection(3);
  faces[goal_face].ptIntersection(usable_grasp, (centroid-usable_grasp), intersection);
  Vec set_distV = usable_grasp - intersection;
  double set_dist = set_distV.norm();
  
  // determine if usable grasp is too different from ideal
  Vec diffV = usable_grasp - orig_grasp;
  double diff = diffV.norm();
  //cout << "diff " << diff << endl;
  double DIFF_THRESH = 10;
  if (diff>DIFF_THRESH)
    {
      // must calc a motion to compensate
      cout << "NOW WE NEED TO DANCE BITCH" << endl;

      int test;
      findSupportFace(usable_grasp, test);
      if (test!=goal_face)
	cout << "good threshold" << endl;

      // now trace out a circle of radius lift_dist:
      // find the 2 perp axes, axis1 & axis2, to the axis of rotation of the circle, axisR
      Vec axis1 = -ln;
      axis1.normalize();
      Vec axis2 = Vec("0 0 1",3);
      Vec axisR = axis1^axis2;
      double theta = asin(usable_grasp[2]/pivot_dist);

      cout << "axis1 " << axis1 << " axis2 " << axis2 << endl;
      cout << "y " << usable_grasp[2] << " x " << pivot_dist << endl;
      cout << "theta " << theta << ", " << (180/3.14)*theta << endl;

      // first go to an approach position
      Vec approach = usable_grasp + (-ln)*100;
      geometry_msgs::Pose a = cart2pose(approach,q);
      waypts.push_back(a);

      // reach for the grasp
      geometry_msgs::Pose p0 = cart2pose(usable_grasp,q);
      waypts.push_back(p0);

      // lift
      geometry_msgs::Pose p1 = p0;
      p1.position.z += pivot_dist;
      waypts.push_back(p1);

      //double PI = 3.14159;
      int num = 10;
      double angle = PI/2; // default to 90 degrees....
      if (angleBetweenSides(face,goal_face,angle))
	angle = PI - angle;

      theta = PI/2;
      double step_size = (PI/4)/num;  //(angle/2)/num; // 45 
      cout << "step size" << step_size*(180/PI) << " theta " << theta << endl;
      for (int i=0;i<1;i++)
	{
	  cout << "theta " << theta << " " << (180/PI)*theta << endl;	 
	  //cout << "v = " << v << ";" << endl;
	  Vec v = pivot + axis1*pivot_dist*cos(theta) + axis2*pivot_dist*sin(theta);
	  theta += step_size;

	  geometry_msgs::Pose p = cart2pose(v,q);
	  waypts.push_back(p);
	}

    }
  else
    {
      // first go to an approach position
      Vec approach = usable_grasp + (-ln)*100;
      geometry_msgs::Pose a = cart2pose(approach,q);
      waypts.push_back(a);

      // reach for the grasp
      geometry_msgs::Pose p0 = cart2pose(usable_grasp,q);
      waypts.push_back(p0);

      // lift
      geometry_msgs::Pose p1 = p0;
      p1.position.z += pivot_dist;
      waypts.push_back(p1);

      // and set straight down
      geometry_msgs::Pose p2 = p1;
      p2.position.z -= set_dist;
      waypts.push_back(p2);
    }
  
  return true;
}



geometry_msgs::Pose Polyhedron::cart2pose(Vec v, Quaternion q)
{
  geometry_msgs::Pose p;
  p.position.x = v[0];
  p.position.y = v[1];
  p.position.z = v[2];
  p.orientation.w = q[0];
  p.orientation.x = q[1];
  p.orientation.y = q[2];
  p.orientation.z = q[3];

  return p;
}

void Polyhedron::pose2cart(geometry_msgs::Pose p, Vec &v, Quaternion &q)
{
  v[0] = p.position.x;
  v[1] = p.position.y;
  v[2] = p.position.z;
  q[0] = p.orientation.w;
  q[1] = p.orientation.x;
  q[2] = p.orientation.y;
  q[3] = p.orientation.z;

}

bool Polyhedron::findGraspForce(int &close)
{
  // the fingers range from 0-255 counts from 70mm to close

  // find width of the object
  double FINGER_WIDTH = 70;
  
  // find a pair of parallel faces
  for (int i=0;i<n;i++)
    for(int j=(n-1);j>i;j--)
      {
	Vec v1 = faces[i].getNormal();
	Vec v2 = faces[j].getNormal();
	Vec ground = Vec("0 0 1",3);

	if ((i!=j) && (v1.parallel(v2)) && (!faces[i].getNormal().parallel(ground)))
	  {            
	    // Decide if it fits in the gripper
	    Vec tmp = faces[i].getCentroid()-faces[j].getCentroid();
	    double dist = tmp.norm();
	    if (dist<FINGER_WIDTH)
	      {
		close = (dist*256)/63;
		return true;
	      }
	  }
      }
  return false;

}

bool Polyhedron::findHandOffset(Quaternion q, Vec &offset)
{  
  RotMat r = q.getRotMat();

  // 130mm from tcp to middle of fingertips (where the bump is)
  //Vec v = Vec("0 0 -120",3);
  Vec v = Vec("0 0 -140",3);
  offset = r*v;

  return true;
}

bool Polyhedron::angleBetweenSides(int s1, int s2, double &angle)
{
  Polygon p1 = faces[s1];
  Polygon p2 = faces[s2];

  // check that faces are adjacent & use that edge as the axis of rotation
  int count = 0;
  for (int i=0;i<p1.size();i++)
    for (int j=0;j<p2.size();j++)
      {
	if (p1[i]==p2[j])
	  count++;

	if (count>=2)
	  break;
      }
  if (count <2)
    return false;

  // Find angle using mag of the cross product
  Vec cross = p1.getNormal()^p2.getNormal();
  angle = asin(cross.norm());

  return true;
}  

Polygon Polyhedron::operator[](const int i) const
{
  if ((i>=0) && (i<n))
    return faces[i];
  else 
    {
      cout << "invalid face index" << endl;
      vector<Vec> v;
      return Polygon(v); // empty  
    }
}


bool Polyhedron::in(Vec pt)
{
  // ray casting in 3D
  
  // project the point along a line in any direction 
  Vec pt_end = pt*100;
  Vec ln = pt_end - pt;

  // now count the number of faces it crosses
  int count = 0;
  int edge = 0;
  for (int i=0;i<n;i++)
    {
      Vec intersection;
      bool ret = faces[i].ptIntersection(pt, ln, intersection);
      if (ret)
	{
	  //cout << "i " << i << endl;
	  // cout << "pt " << pt << pt_end << endl;
	  //cout << "intersection " << intersection << endl;
	  //faces[i].print();
	  double d;
	  if (faces[i].onEdge(intersection, d))
	      edge++;
	  count++;
	}
    }

  //cout << "count " << count << " edge " << edge << endl;
  if ((count %2 !=0) && (count!=0))
    return true;
  else if ((edge %4!=0) && (edge!=0))
    return true;
  else
    return false;
}

int Polyhedron::quat2face(Quaternion q)
{
  int i;
  Polyhedron p_tmp (*this);
  p_tmp.rotate(q);
  p_tmp.findSupportFace(p_tmp.getCentroid(),i);
  return i;
}

vector<int> Polyhedron::getSymmetricFaces(int face)
{
  vector<int> sameFaces;
  sameFaces.push_back(face);
  for (int i=0;i<n;i++)
    if ((faces[i]==faces[face]) && (face!=i))
      sameFaces.push_back(i);
  
  return sameFaces;
}

double Polyhedron::bottom()
{
  vector<Vec> verts = getVertices();

  double lowest = verts[0][2];
  for (size_t i=0;i<verts.size();i++)
    if (verts[i][2]<lowest)
      lowest = verts[i][2];

  return lowest;
}

double Polyhedron::top()
{
  vector<Vec> verts = getVertices();

  double highest = verts[0][2];
  for (size_t i=0;i<verts.size();i++)
    if (verts[i][2]<highest)
      highest = verts[i][2];

  return highest;
}

bool Polyhedron::findPivotEdge(int goal_face, Vec &pivot)
{
  int support;
  findSupportFace(centroid, support);
  Polygon p1 = faces[support];
  Polygon p2 = faces[goal_face];

  vector<Vec> verts;
  // check that faces are adjacent & use that edge as the axis of rotation
  int count = 0;
  for (int i=0;i<p1.size();i++)
    for (int j=0;j<p2.size();j++)
      {
	if (p1[i]==p2[j])
	  {
	    verts.push_back(p1[i]);
	    count++;
	  }

	if (count>=2)
	  break;
      }
  if (count <2)
    return false;

  pivot = Vec("0 0 0",3);
  for (size_t i=0;i<verts.size();i++)
    pivot = pivot + verts[i];
  pivot = pivot/verts.size();

  return true;
}

bool Polyhedron::findPushGrasp(Vec pivot, Vec &grasp)
{
  // go up in z until reach the top of the block
  grasp = pivot;

  while (in(grasp)) 
    grasp[2] += 1;

  double thresh = (top() - bottom())*0.75;
  if ((grasp[2] - pivot[2]) < thresh)
    return false;
  grasp[2] -= 20;
  cout << "sweetener " << grasp << endl;
  
  return true;
}
bool Polyhedron::pushgraspable(Vec grasp, int &grasp_face, Vec &new_grasp, bool &give_up)
{
  double FINGER_WIDTH = 70;
  double Z_THRESH = bottom() + 15;
  double EDGE_MARGIN = 15;

  int face;
  findSupportFace(centroid,face);
  // find dist to edge/surface
  int close = 0;
  double edgeDist = 10000;
  double surDist = 10000;
  for (int i=0;i<n;i++)
    {
      double maybe;
      faces[i].onEdge(grasp, maybe);
      double msurDist = (faces[i].projection(grasp)-grasp).norm();
      if (((msurDist<surDist)) && (i!=face))
	{
	  edgeDist = maybe;
	  close = i;
	  surDist = msurDist;
	}
    }

  // find a pair of parallel faces
  for (int i=0;i<n;i++)
    for(int j=(n-1);j>i;j--)
      {
	Vec v1 = faces[i].getNormal();
	Vec v2 = faces[j].getNormal();
	Vec ground = Vec("0 0 1",3);

	if ((i!=j) && (v1.parallel(v2)) && (!faces[i].getNormal().parallel(ground)))
	  {            
	    // Decide if it fits in the gripper
	    Vec tmp = faces[i].getCentroid()-faces[j].getCentroid();
	    double dist = tmp.norm();
	    grasp_face = i;
	    if (dist<FINGER_WIDTH)
	      {
		bool good = true;
		new_grasp = grasp;

		// check if too close to the ground
		if (new_grasp[2]<Z_THRESH)
		  {
		    new_grasp[2] += 5;
		    //cout << "too low" << endl;
		    good = false;
		  }
		
		// check if too close to an edge
		if ((edgeDist<EDGE_MARGIN) || (surDist<EDGE_MARGIN))
		  {
		    new_grasp += (-faces[close].getNormal())*5;;
		    //cout << "too close to an edge" << endl;
		    good = false;
		  }

		if (!in(new_grasp))
		  {
		    give_up = true;
		    cout << "ERROR:  GRASP IMPOSSIBLE" << endl;
		  }
		give_up = false;
		return good;
	      }
	  }
      }

  cout << "block bad shape/orientation" << endl;
  return false;
}

bool Polyhedron::pushPick(int goal_face, Vec &grasp, Quaternion &q)
{
  Vec pivot;
  bool adjacent = findPivotEdge(goal_face, pivot);
  if (!adjacent)
    return false;

  cout << "pivot " << pivot << endl;
 
  bool ret = findPushGrasp(pivot, grasp);
  if (!ret)
    return false;

  cout << "grasp " << grasp << endl;

  int grasp_face;
  Vec new_grasp;
  
  bool can = false;
  int count = 0;
  bool give_up;
  while ((!can) && (count<100))
    {
      can = pushgraspable(grasp, grasp_face, new_grasp, give_up);
      cout << "can " << can << endl;
      cout << "grasp " << new_grasp << endl;
      grasp = new_grasp;

      if (give_up)
	break;
    }
  if (count>100)
    return false;

  pushApproach(grasp, grasp_face, q);

  return true;
}

bool Polyhedron::pushApproach(Vec grasp, int grasp_face, Quaternion &q)
{
  // point at the centroid from the grasp
  // From the frame of the robotiq hand, we want:
  // z axis pointing in the normal to the closest surface
  // y axis in the normal to the grasp surfaces

  Vec x(3);
  Vec y = faces[grasp_face].getNormal();
  cout << "grasp face " << grasp_face << " normal " << y << endl;
  //  Vec z(3);

  int support;
  findSupportFace(centroid, support);
  int close = 0;
  double edgeDist = 10000;
  for (int i=0;i<n;i++)
    {
      double maybe;
      faces[i].ptPlaneDist(grasp, maybe);
      if ((maybe<edgeDist) && (i!=support) && 
	  (!y.parallel(faces[i].getNormal())))
	{
	  edgeDist = maybe;
	  close = i;
	}
    }

  cout << "close " << close << " dist " << edgeDist << endl; 
  Vec ground = Vec("0 0 1",3);
  Vec z = -(faces[close].getNormal() - ground);
  z.normalize();

  cout<< "y " << y << " z " << z << endl;
  x = y^z;

  x.normalize();
  y.normalize();
  z.normalize();

  RotMat r(x,y,z);
  q = r.getQuaternion();

  return true;
}

bool Polyhedron::push(Vec grasp, Quaternion q, vector<geometry_msgs::Pose> &waypts)
{
  geometry_msgs::Pose a = cart2pose(grasp,q);
  waypts.push_back(a);

  double dist = grasp[2] - bottom();

  int support;
  findSupportFace(centroid, support);
  Vec dir = faces[support].projection((grasp-centroid));
  dir.normalize();
  Vec down = Vec("0 0 -1",3);

  Vec push = dir+down;
  push.normalize();

  Vec next = push * (dist/2);

  return true;
}

bool Polyhedron::alignNormals()
{
  for (int i=0;i<n;i++)
    {
      Vec in = centroid - faces[i].getCentroid();
      if (in*faces[i].getNormal()>0)
	faces[i].switchNormal(true);
    }
  return true;
}
