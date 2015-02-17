#include <math.h>
#include <mex.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include "vrmlIO.h"

#define MAX_BUFFER 1024

//MEX Function to load a VRML 3D model into Matlab.

//Output parameters
//nlhs     Number of expected output parameters =  2 in this case.
//plhs[0]  Output parameter 1. Array (3xnTriangles) with all triangles in the model.
//         Each column is a vector with 3 indices that correspond to the 3 vertices in the triangle.
//plhs[1]  Output parameter 2. Array (3xnvertices) with all vertices in the model.
//         Each column is a vector with the (x,y,z) coordinates of the vertex.

//Input parameters
//nrhs     The number of input mxArrays = 1 in this case.
//prhs[0]  String with the name of the WRL file to load.

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  char buffer[MAX_BUFFER];
  std::vector<std::vector<double> > vertices;
  std::vector<std::vector<int> > triangles;

  //Read VRML file
  if(mxGetString(prhs[0], buffer, MAX_BUFFER)!=0)
    {
      mexPrintf("Please provide name of the file to load.\n"); 
      return;
    }
 
  //sprintf(buffer,"3Dmodels/tetrahedron.wrl");
  if(!readWRL(buffer, vertices, triangles))
    {
      mexPrintf("Impossible to read the requested VRML file.\n"); 
      return;
    }

  //Transform 3D model into MATLAB format
  nlhs = 2;

  mwSize dimsT[] = {3,triangles.size()};  
  plhs[0] = mxCreateNumericArray(2, dimsT, mxDOUBLE_CLASS, mxREAL);
  double *mexTriangles=mxGetPr(plhs[0]);
  for (int i=0; i<triangles.size(); i++)
    {
      *mexTriangles++=triangles[i][0] + 1;  // MATLAB arrays are indexed 1..N and C arrays are indexed 0..N-1
      *mexTriangles++=triangles[i][1] + 1;  // MATLAB arrays are indexed 1..N and C arrays are indexed 0..N-1
      *mexTriangles++=triangles[i][2] + 1;  // MATLAB arrays are indexed 1..N and C arrays are indexed 0..N-1
    }

  mwSize dimsV[] = {3,vertices.size()};  
  plhs[1] = mxCreateNumericArray(2, dimsV, mxDOUBLE_CLASS, mxREAL);
  double *mexVertices=mxGetPr(plhs[1]);
  for (int i=0; i<vertices.size(); i++)
    {
      *mexVertices++=vertices[i][0];
      *mexVertices++=vertices[i][1];
      *mexVertices++=vertices[i][2];
    }
  return;
}
