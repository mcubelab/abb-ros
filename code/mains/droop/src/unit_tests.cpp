#include <iostream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <robot_comm/robot_comm.h>
#include <robotiq_comm/robotiq_comm.h>
#include <tableVision_comm/tableVision_comm.h>
#include <geometry_msgs/Pose.h>
#include <matlab_comm/matlab_comm.h>
#include <matVec/matVec.h>
#include "droop/Polyhedron.h"
#include <geometry_msgs/Pose.h>

Vec x1 = Vec("0 0 0",3);
Vec x2 = Vec("10 0 0",3);
Vec x3 = Vec("10 10 0",3);
Vec x4 = Vec("0 10 0",3);
Vec x5 = Vec("10 0 10",3);
Vec x6 = Vec("0 0 10",3);
Vec x7 = Vec("5 5 1",3);
Vec x8 = Vec("3 7 0",3);
Vec x9 = Vec("5 -5 0",3);
Vec x10 = Vec("5 5 0",3);
Vec x11 = Vec("5 5 5",3);
Vec x12 = Vec("0 -5 0",3);
Vec x13 = Vec("0 5 0",3);
Vec x14 = Vec("5 -1 0",3);
Vec randpt(3);

vector<Vec> planarXY;
vector<Vec> planarXZ;
vector<Vec> oneBad;
Quaternion q; 
RotMat r;
Polygon test(planarXY);
Polyhedron tri(RecObj::BIG_TRIANGLE);
Polyhedron rtri(RecObj::BIG_TRIANGLE);
//Polyhedron rect(RecObj::RECTANGLE);

void testCoplanar2D()
{
  Polygon p(planarXY);
  assert(p.coplanar());
  Polygon p1(planarXZ);
  assert(p1.coplanar());
}
void testCoplanarSkew()
{
  Polygon p(planarXY);
  p.rotate(q);
  assert(p.coplanar());
  Polygon p1(planarXZ);
  p1.rotate(q);
  assert(p1.coplanar());
}
void testCoplanarBadPt()
{
  Polygon p(oneBad);
  assert(!p.coplanar());
}
void testCoplanarNotInPoly()
{
  Polygon p(planarXY);
  assert(!p.coplanar(x7));
  assert(!p.coplanar(randpt));
}
void testCoplanarInPoly()
{
  Polygon p(planarXY);
  assert(p.coplanar(x8));
}

void testFindIntersectGood()
{
  Vec i;
  Vec t = Vec("5 0 0",3);
  assert(test.findIntersect(x1,x2,x9,x10,i));
  assert(i==t);

  Vec ii;
  t = r*t;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y9 = r*x9;
  Vec y10 = r*x10;
  assert(test.findIntersect(y1,y2,y9,y10,ii));
  assert(ii==t);
}
void testFindIntersectSkew()
{
  Vec i;
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!test.findIntersect(x1,x2,x9,x11,i));
  assert(i==t);

  Vec ii;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y9 = r*x9;
  Vec y11 = r*x11;
  assert(!test.findIntersect(y1,y2,y9,y11,ii));
  assert(ii==t);
}
void testFindIntersectOneCorner()
{
  Vec i;
  assert(test.findIntersect(x1,x2,x12,x13,i));
  assert(i==x1);

  RotMat r = q.getRotMat();
  Vec ii;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y12 = r*x12;
  Vec y13 = r*x13;
  assert(test.findIntersect(y1,y2,y12,y13,ii));
  assert(ii==y1);
}
void testFindIntersectBothCorner()
{
  Vec i;
  assert(test.findIntersect(x1,x2,x2,x3,i));
  assert(i==x2);

  Vec ii;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y3 = r*x3;
  assert(test.findIntersect(y1,y2,y2,y3,ii));
  assert(ii==y2);
}
void testFindInstersectNotInSegment()
{
  Vec i;
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!test.findIntersect(x1,x2,x9,x14,i));
  assert(i==t); 

  Vec ii;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y9 = r*x9;
  Vec y14 = r*x14;
  assert(!test.findIntersect(y1,y2,y9,y14,ii));
  assert(ii==t);
}
void testFindIntersectParallel()
{
  Vec i;
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!test.findIntersect(x1,x2,x3,x4,i));
  assert(i==t);

  Vec ii;
  Vec y1 = r*x1;
  Vec y2 = r*x2;
  Vec y3 = r*x3;
  Vec y4 = r*x4;
  assert(!test.findIntersect(y1,y2,y3,y4,ii));
  assert(ii==t);
}

void testInGood()
{
  Polygon p(planarXY);
  assert(p.in(x10));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y10 = r*x10;
  assert(p1.in(y10));
  
}
void testInOut()
{
  Polygon p(planarXY);
  assert(!p.in(x9));

 Polygon p1(planarXY);
 p1.rotate(q);
 Vec y9 = r*x9;
 assert(!p1.in(y9));
}
void testInEdge()
{
  Vec v = Vec("5 0 0",3);
  Polygon p(planarXY);
  assert(p.in(v));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec yv = r*v;
  assert(p1.in(yv));
}
void testInCorner()
{
  Polygon p(planarXY);
  assert(p.in(x1));
  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y1 = r*x1;
  assert(p1.in(y1));
}
void testInNotCoplanar()
{
  Polygon p(planarXY);
  assert(!p.in(x7));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y7 = r*x7;
  assert(!p1.in(y7));
}
void testProjectionPlanar()
{
  Polygon p(planarXY);
  assert(p.projection(x10)==x10);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y10 = r*x10;
  assert(p1.projection(y10)==y10);
}
void testProjectionOff()
{
  Polygon p(planarXY);
  assert(p.projection(x7)==x10);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y7 = r*x7;
  Vec y10 = r*x10;
  assert(p1.projection(y7)==y10);
}

void testfindEdgeIntersectIn()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("1 0 0",3);
  Vec t = Vec("10 5 0",3);
  assert(p.findEdgeIntersect(x10,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  t = r*t;
  Vec y10 = r*x10;
  assert(p1.findEdgeIntersect(y10,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectNotIn()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 1 0",3);
  Vec t = Vec("5 0 0",3);
  assert(p.findEdgeIntersect(x9,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  t = r*t;
  Vec y9 = r*x9;
  assert(p1.findEdgeIntersect(y9,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectCorner()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("1 1 0",3);
  Vec t = Vec("10 10 0",3);
  assert(p.findEdgeIntersect(x10,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  t = r*t;
  Vec y10 = r*x10;
  assert(p1.findEdgeIntersect(y10,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectNotCoplanar()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 1 0",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.findEdgeIntersect(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y7 = r*x7;
  assert(!p1.findEdgeIntersect(y7,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectLineNotCoplanar()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 0 1",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.findEdgeIntersect(x10,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y10 = r*x10;
  assert(!p1.findEdgeIntersect(y10,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectNoIntersection()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 0 1",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.findEdgeIntersect(x9,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y9 = r*x9;
  assert(!p1.findEdgeIntersect(y9,s,ii));
  assert(ii==t);
}
void testfindEdgeIntersectLinePointsAway()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 -1 0",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.findEdgeIntersect(x9,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y9 = r*x9;
  assert(!p1.findEdgeIntersect(y9,s,ii));
  assert(ii==t);
}

void testPtIntersectionGood()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("0 0 -1",3);
  assert(p.ptIntersection(x7,s,i));
  assert(i==x10);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y10 = r*x10;
  Vec y7 = r*x7;
  assert(p1.ptIntersection(y7,s,ii));
  assert(ii==y10);
}
void testPtIntersectionEdge()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 0 -1",3);
  Vec t = Vec("10 5 0",3);
  assert(p.ptIntersection(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  t = r*t;
  Vec y7 = r*x7;
  assert(p1.ptIntersection(y7,s,ii));
  assert(ii==t);
}
void testPtIntersectionCorner()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 5 -1",3);
  Vec t = Vec("10 10 0",3);
  assert(p.ptIntersection(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  t = r*t;
  Vec y7 = r*x7;
  assert(p1.ptIntersection(y7,s,ii));
  assert(ii==t);
}
void testPtIntersectionCoplanarPt()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 5 -1",3);
  assert(p.ptIntersection(x10,s,i));
  assert(i==x10);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y10 = r*x10;
  assert(p1.ptIntersection(y10,s,ii));
  assert(ii==y10);
}
void testPtIntersectionCoplanar()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 5 0",3);
  assert(p.ptIntersection(x10,s,i));
  assert(i==x10);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y10 = r*x10;
  assert(p1.ptIntersection(y10,s,ii));
  assert(ii==y10);
}
void testPtIntersectionParallel()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 5 0",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.ptIntersection(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y7 = r*x7;
  assert(!p1.ptIntersection(y7,s,ii));
  assert(ii==t);
}
void testPtIntersectionLinePointsAway()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("5 5 1",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.ptIntersection(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y7 = r*x7;
  assert(!p1.ptIntersection(y7,s,ii));
  assert(ii==t);
}
void testPtIntersectionOutsidePolygon()
{
  Polygon p(planarXY);
  Vec i;
  Vec s = Vec("10 10 -1",3);
  Vec t = Vec("-1000 -1000 -1000",3);
  assert(!p.ptIntersection(x7,s,i));
  assert(i==t);

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec ii;
  s = r*s;
  Vec y7 = r*x7;
  assert(!p1.ptIntersection(y7,s,ii));
  assert(ii==t);
}

void testOnEdgeIn()
{
  double tmp;
  Polygon p(planarXY);
  assert(!p.onEdge(x10, tmp));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y10 = r*x10;
  assert(!p1.onEdge(y10, tmp));
}
void testOnEdgeOut()
{
  double tmp;
  Polygon p(planarXY);
  assert(!p.onEdge(x9, tmp));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y9 = r*x9;
  assert(!p1.onEdge(y9, tmp));
}
void testOnEdgeGood()
{
  double tmp;
  Polygon p(planarXY);
  assert(p.onEdge(x13, tmp));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y13 = r*x13;
  assert(p1.onEdge(y13, tmp));
}
void testOnEdgeCorner()
{
  double tmp;
  Polygon p(planarXY);
  assert(p.onEdge(x1, tmp));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec y1 = r*x1;
  assert(p1.onEdge(y1, tmp));
}
void testOnEdgeNotCoplanar()
{
  double tmp;
  Vec v = Vec("0 0 1",3);
  Polygon p(planarXY);
  assert(!p.onEdge(v, tmp));

  Polygon p1(planarXY);
  p1.rotate(q);
  Vec yv = r*v;
  assert(!p1.onEdge(yv, tmp));
}

void testPInGood()
{
  assert(tri.in(tri.getCentroid()));
  assert(rtri.in(rtri.getCentroid()));

  Vec v = Vec("25 10 10",3);
  assert(tri.in(v));
  v = r*v;
  assert(rtri.in(v));
}
void testPInEdge()
{
  assert(tri.in(x2));

  Vec y2 = r*x2;
  assert(rtri.in(y2));
}
void testPInCorner()
{
  assert(tri.in(x1));

  Vec y1 = r*x1;
  assert(rtri.in(y1));
}
void testPInBad()
{
  assert(!tri.in(x9));

  Vec y9 = r*x9;
  assert(!rtri.in(y9));
}

void testFindSupportFaceCentroid()
{
  int i;
  assert(tri.findSupportFace(tri.getCentroid(),i));
  assert(i==4);

  Polyhedron trix = tri;
  Polyhedron triy = tri;
  Quaternion qx = Quaternion("0.707 0.707 0 0");
  Quaternion qy = Quaternion("-0.707 0 0.707 0");
  trix.rotate(qx);
  triy.rotate(qy);
  assert(trix.findSupportFace(trix.getCentroid(),i));
  assert(i==1);
  assert(triy.findSupportFace(triy.getCentroid(),i));
  assert(i==0);
}
void testFindSupportFaceGraspNotInBlock()
{
  int i;
  Vec bad = Vec("100 100 100",3);
  assert(!tri.findSupportFace(bad,i));
  assert(i==-1000);

  int ii;
  Vec bad2 = Vec("-10 -10 -10",3);
  assert(!tri.findSupportFace(bad2,ii));
  assert(ii==-1000);

}
void testFindSupportFaceSameButNotCentroid()
{
  int i;
  Vec v = tri.getCentroid();
  v[0] += 1;
  v[1] += 1;
  v[2] += 1;
  assert(tri.findSupportFace(v,i));
  assert(i==4);

  Polyhedron trix = tri;
  Polyhedron triy = tri;
  Quaternion qx = Quaternion("0.707 0.707 0 0");
  Quaternion qy = Quaternion("-0.707 0 0.707 0");
  trix.rotate(qx);
  triy.rotate(qy);
  Vec xv = trix.getCentroid();
  xv[0] += -1;
  xv[2] += 1;
  Vec yv = triy.getCentroid();
  yv[1] += -1;
  yv[2] += 1;
  assert(trix.findSupportFace(xv,i));
  assert(i==1);
  assert(triy.findSupportFace(yv,i));
  assert(i==0);
}
void testFindSupportFaceDifferent()
{
  int i;
  Vec v = tri.getCentroid();
  v[0] += 1;
  assert(tri.findSupportFace(v,i));
  assert(i==0);
}
void testFindSupportFaceCorner() // supported on a corner
{
  int i;
  Vec v = tri.getCentroid();
  v[0] += 1;
  v[1] += 1;
  assert(tri.findSupportFace(v,i));
  assert(i==0);
}
void testFindSupportFaceEdge() // supported on an edge
{
  int i;
  Vec v = tri.getCentroid();
  v[0] += 34.33333/2;
  v[1] += 34.33333/2;
  v[2] += 17.5/2;

  assert(tri.findSupportFace(v,i));
  assert(i==0);
}

void testGetFaceGraspGood()
{
  Vec grasp;
  Vec t = Vec("103 0 17.5",3);
  tri.getFaceGrasp(0, grasp);
  assert(grasp==t);

  Vec tt = Vec("0 103 17.5",3);
  tri.getFaceGrasp(1, grasp);
  assert(grasp==tt);
}
void testGetFaceGraspEdge() // the grasp is on an edge
{
  Vec grasp;
  Vec t = Vec("0 0 17.5",3);
  tri.getFaceGrasp(2, grasp);
  assert(grasp==t);
}

void testQuat2faceGood()
{
  Quaternion t = Quaternion("0.707 0.707 0 0");
  assert(tri.quat2face(t)==1);

  Quaternion tt = Quaternion("0.707 0 -0.707 0");
  assert(tri.quat2face(tt)==0);
  
  Quaternion ttt = Quaternion("1 0 0 0");
  assert(tri.quat2face(ttt)==4);
}

// void testGraspableOutsideObj()
// {
//   Quaternion t = Quaternion("0.707 0.707 0 0");
//   Polyhedron copy (tri);
//   copy.rotate(t);
//   int i;
//   Vec v;

//   Vec grasp = Vec("100 100 100",3);
//   assert(!copy.graspable(grasp,i,v));

// }
// void testGraspableTooLow()
// {
//   Quaternion t = Quaternion("0.707 0.707 0 0");
//   Polyhedron copy (tri);
//   copy.rotate(t);
//   int i;
//   Vec v;

//   assert(!copy.graspable(x1,i,v));
// }
// void testGraspableCentroid()
// {
//   Quaternion t = Quaternion("0.707 0.707 0 0");
//   Polyhedron copy (tri);
//   copy.rotate(t);
//   int i;
//   Vec v;

//   assert(!copy.graspable(copy.getCentroid(),i,v));
// }
// void testGraspableTooIn()
// {
//   Quaternion t = Quaternion("0.707 0.707 0 0");
//   Polyhedron copy (tri);
//   copy.rotate(t);
//   int i;
//   Vec v;

//   Vec tmp = copy.getCentroid();
//   tmp[0] += 1;
//   assert(!copy.graspable(tmp,i,v));

// }
// void testGraspableGood()
// {
//   Quaternion t = Quaternion("0.707 0.707 0 0");
//   Polyhedron copy (tri);
//   copy.rotate(t);
//   int i;
//   Vec v;
//   assert(copy.graspable(copy.getCentroid(),i,v));
// }
// void testGraspableBadOrientation()
// {
//   int i;
//   Vec v;
//   assert(!tri.graspable(tri.getCentroid(),i,v));
// }

void testEqual()
{
  Polygon p1 = tri[0];
  Polygon p2 = tri[1];
  assert(p1==p2);

  vector<int> tmp;
  tmp.push_back(0);
  tmp.push_back(1);
  vector<int> tmp2;
  tmp2.push_back(3);
  tmp2.push_back(4);

  assert(tri.getSymmetricFaces(0)==tmp);
  assert(tri.getSymmetricFaces(3)==tmp2);
}

void testFindPivotEdge()
{
  Quaternion t = Quaternion("0.707 0.707 0 0");
  Polyhedron copy (tri);
  copy.rotate(t);

  // Vec pivot;
  // copy.findPivotEdge(0,pivot);
  // cout << "pivot " << pivot << endl;
  
  Vec grasp;
  Quaternion q;
  copy.pushPick(0, grasp, q);
  
}

/***************** POLYGON ********************/
// coplanar
void testCoplanar2D();
void testCoplanarSkew();
void testCoplanarBadPt();
void testCoplanarNotInPoly();
void testCoplanarInPoly();
// findIntersect
void testFindIntersectGood();
void testFindIntersectParallel();
void testFindIntersectSkew();
void testFindIntersectOneCorner();
void testFindIntersectBothCorner();
void testFindInstersectNotInSegment();
// in
void testInGood();
void testInOut();
void testInEdge();
void testInCorner();
void testInNotCoplanar();
// projection
void testProjectionPlanar();
void testProjectionOff();
// findEdgeIntersect
void testfindEdgeIntersectIn();
void testfindEdgeIntersectNotIn();
void testfindEdgeIntersectCorner();
void testfindEdgeIntersectNotCoplanar();
void testfindEdgeIntersectLineNotCoplanar();
void testfindEdgeIntersectNoIntersection();
void testfindEdgeIntersectLinePointsAway();
// ptIntersection
void testPtIntersectionGood();
void testPtIntersectionEdge();
void testPtIntersectionCorner();
void testPtIntersectionCoplanarPt();
void testPtIntersectionCoplanar();
void testPtIntersectionParallel();
void testPtIntersectionLinePointsAway();
void testPtIntersectionOutsidePolygon();
// onEdge
void testOnEdgeIn();
void testOnEdgeOut();
void testOnEdgeGood();
void testOnEdgeCorner();
void testOnEdgeNotCoplanar();
// equal
void testEqual();
/**********************************************/

/****************** POLYHEDRON ****************/
// in
void testPInGood();
void testPInEdge();
void testPInCorner();
void testPInBad();
// findSupportFace
void testFindSupportFaceCentroid();
void testFindSupportFaceGraspNotInBlock();
void testFindSupportFaceSameButNotCentroid();
void testFindSupportFaceDifferent();
void testFindSupportFaceCorner(); // supported on a corner
void testFindSupportFaceEdge(); // supported on an edge
// getFaceGrasp
void testGetFaceGraspGood();
void testGetFaceGraspEdge(); // the grasp is on an edge
// quat2face
void testQuat2faceGood();
// graspable
void testGraspableOutsideObj();
void testGraspableTooLow();
void testGraspableCentroid();
void testGraspableTooIn();
void testGraspableGood();
void testGraspableBadOrientation();

void testFindPivotEdge();
/**********************************************/

using namespace std;
int main (int argc, char** argv) 
{  
  ros::init(argc, argv, "DROOP_BITCH");
  ros::NodeHandle node;

  planarXY.push_back(x1);
  planarXY.push_back(x2);
  planarXY.push_back(x3);
  planarXY.push_back(x4);
  planarXZ.push_back(x1);
  planarXZ.push_back(x2);
  planarXZ.push_back(x5);
  planarXZ.push_back(x6);
  oneBad = planarXY;
  oneBad.push_back(x7);
  q[0] = rand();
  q[1] = rand();
  q[2] = rand();
  q[3] = rand();
  q.normalize();
  r = q.getRotMat();
  randpt[0] = rand();
  randpt[1] = rand();
  randpt[2] = rand();

  Polyhedron rect = Polyhedron(RecObj::LONG_BLOCK);
  Vec seenV = Vec("600 200 200",3);;
  Quaternion seenQ = Quaternion("0.707 0.707 0 0");
  int goal = 0;
  rect.rotate(seenQ);
  rect.print();

  //Find grasp
  Vec orig_grasp;
  Vec usable_grasp;
  Quaternion quat;
  rect.pick(goal, orig_grasp, usable_grasp, quat);
  cout << "orig grasp " << orig_grasp << endl;
  cout << "usable grasp " << usable_grasp << endl;
  cout << "quat " << quat << endl;



  // //rtri.rotate(q);
  // Quaternion qq = Quaternion("0.707 0.707 0 0");
  // rtri.rotate(qq);
  // double pi = 3.14159;

  // rect.print();
  // int support;
  // rect.findSupportFace(rect.getCentroid(), support);
  // cout << "support " << support << endl;

  // test findRotationAngle
  // int goal = 2;
  // rtri.print();
  // Vec orig_grasp;
  // Vec usable_grasp;
  // Quaternion quat;
  // rtri.pick(goal, orig_grasp, usable_grasp, quat);
  // // cout << "orig grasp " << orig_grasp << endl;
  // // cout << "usable grasp " << usable_grasp << endl;
  // // cout << "quat " << quat << endl;

  // // Vec gg = Vec("90 -17.4947 10",3);

  // double m = rtri.findMoment(usable_grasp);
  // cout << "m " << m << endl;

  // double angle;
  // rtri.findRotationAngle(usable_grasp, goal, angle);
  // cout << "angle " << angle*(180/pi) << endl;


  // tri.findFaceAngle(int face, double &angle);
  // tri.findRotationAngle(Vec grasp, int goal, double &angle);

  // begin testing
  // testCoplanar2D();
  // testCoplanarSkew();
  // testCoplanarBadPt();
  // testCoplanarNotInPoly();
  // testCoplanarInPoly();

  // testFindIntersectGood();
  // testFindIntersectSkew();
  // testFindIntersectOneCorner();
  // testFindIntersectBothCorner();
  // testFindInstersectNotInSegment();
  // testFindIntersectParallel();
  
  // testInGood();
  // testInOut();
  // testInEdge();
  // testInCorner();
  // testInNotCoplanar();
  
  // testProjectionPlanar();
  // testProjectionOff();
  
  // testfindEdgeIntersectIn();
  // testfindEdgeIntersectNotIn();
  // testfindEdgeIntersectCorner();
  // testfindEdgeIntersectNotCoplanar();
  // testfindEdgeIntersectLineNotCoplanar();
  // testfindEdgeIntersectNoIntersection();
  // testfindEdgeIntersectLinePointsAway();

  // testPtIntersectionGood();
  // testPtIntersectionEdge();
  // testPtIntersectionCorner();
  // testPtIntersectionCoplanarPt();
  // testPtIntersectionCoplanar();
  // testPtIntersectionParallel();
  // testPtIntersectionLinePointsAway();
  // testPtIntersectionOutsidePolygon();

  // testOnEdgeIn();
  // testOnEdgeOut();
  // testOnEdgeGood();
  // testOnEdgeCorner();
  // testOnEdgeNotCoplanar();

  // testPInGood();
  // testPInEdge();
  // testPInCorner();
  // testPInBad();

  // testFindSupportFaceCentroid();
  // testFindSupportFaceGraspNotInBlock();
  // testFindSupportFaceSameButNotCentroid();
  // testFindSupportFaceDifferent();
  // testFindSupportFaceCorner(); // supported on a corner
  // testFindSupportFaceEdge(); // supported on an edge

  // testGetFaceGraspGood();
  // testGetFaceGraspEdge(); // the grasp is on an edge

  // testQuat2faceGood();

  // testGraspableOutsideObj();
  // testGraspableTooLow();
  // testGraspableCentroid();
  // testGraspableTooIn();
  // testGraspableGood();
  // testGraspableBadOrientation();

  // testEqual();

  //testFindPivotEdge();

  return 0;
}
 
