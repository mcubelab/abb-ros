#include <math.h>

#include <mex.h>
#include <PQP.h>

void createPQPModel(const mxArray *T, const mxArray *V, PQP_Model &body);
  
//MEX Function to detect collision between two models for an array of locations of the models.

//Output parameters
//nlhs     Number of expected output parameters =  1 in this case.
//plhs[0]  Output parameter 1. Array (1xnTransformations) of booleans. 1 - collision  0 - no collision.

//Input parameters
//nrhs     The number of input mxArrays = 6 in this case.
//prhs[0]  Input parameter 1. Array (3xnTriangles) with all triangles in the 3D model of the first body.
//         Each column is a vector with 3 indices that correspond to the 3 vertices in the triangle.
//prhs[1]  Input parameter 2. Array (3xnvertices) with all vertices in the 3D model of the first body.
//         Each column is a vector with the (x,y,z) coordinates of the vertex.
//prhs[2]  Input parameter 3. Array (3xnTriangles) with all triangles in the 3D model of the second body.
//         Each column is a vector with 3 indices that correspond to the 3 vertices in the triangle.
//prhs[3]  Input parameter 4. Array (3xnvertices) with all vertices in the 3D model of the second body.
//         Each column is a vector with the (x,y,z) coordinates of the vertex.
//prhs[4]  Input parameter 5. Array (12xnTrnasformations) with all transformations to apply to the first body.
//         Each column is a vector with the 9 components of the rotation matrix (by columns) and the 3 components of the translation.
//prhs[5]  Input parameter 6. Array (12xnTrnasformations) with all transformations to apply to the second body.
//         Each column is a vector with the 9 components of the rotation matrix (by columns) and the 3 components of the translation.

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
  
  PQP_Model body1;
  PQP_Model body2;

  //Read in 3D body models
  createPQPModel(prhs[0], prhs[1],body1);
  createPQPModel(prhs[2], prhs[3], body2);

  PQP_REAL rotBody1[3][3],rotBody2[3][3];
  PQP_REAL transBody1[3],transBody2[3];
  double *trans1=mxGetPr(prhs[4]);
  double *trans2=mxGetPr(prhs[5]);
  
  int nTransformations = mxGetN(prhs[4]);
  int nTransformations2 = mxGetN(prhs[5]);
  if(nTransformations != nTransformations2)
    {
      mexPrintf("Number of transformations for both bodies must concide.\n"); 
      return;
    }

  nlhs = 1;
  mwSize dims[] = {nTransformations};  
  plhs[0] = mxCreateNumericArray(1, dims, mxDOUBLE_CLASS, mxREAL);
  double *out=mxGetPr(plhs[0]);
  for(int i=0; i<nTransformations; i++)
    {
      //Read in transformations
      rotBody1[0][0] = *trans1++;
      rotBody1[1][0] = *trans1++;
      rotBody1[2][0] = *trans1++;
      rotBody1[0][1] = *trans1++;
      rotBody1[1][1] = *trans1++;
      rotBody1[2][1] = *trans1++;
      rotBody1[0][2] = *trans1++;
      rotBody1[1][2] = *trans1++;
      rotBody1[2][2] = *trans1++;
      transBody1[0] = *trans1++;
      transBody1[1] = *trans1++;
      transBody1[2] = *trans1++;

      rotBody2[0][0] = *trans2++;
      rotBody2[1][0] = *trans2++;
      rotBody2[2][0] = *trans2++;
      rotBody2[0][1] = *trans2++;
      rotBody2[1][1] = *trans2++;
      rotBody2[2][1] = *trans2++;
      rotBody2[0][2] = *trans2++;
      rotBody2[1][2] = *trans2++;
      rotBody2[2][2] = *trans2++;
      transBody2[0] = *trans2++;
      transBody2[1] = *trans2++;
      transBody2[2] = *trans2++;

      //Collision detection
      PQP_CollideResult result;
      PQP_Collide(&result, rotBody1, transBody1, &body1, rotBody2, transBody2, &body2);
      if (result.Colliding())
	*out++ = 1.0;
      else
	*out++ = 0.0;	

    }
}

void createPQPModel(const mxArray *T, const mxArray *V, PQP_Model &body) 
{
  PQP_REAL p1[3],p2[3],p3[3];  

  int nTriangles = mxGetN(T);  //Number of triangles is the number of columns in the array structure.
  double *mexTriangles=mxGetPr(T);        //Pointer to triangle model data
  double *mexVertices=mxGetPr(V);        //Pointer to triangle model data
  
  double *p;
  body.BeginModel();   // begin the model
  
  for (int j=0; j<nTriangles ; j++)
    {
      //p = mexVertices[(int)3*(mexTriangles[3*j]-1)];
      p = mexVertices + 3 * (int)(*mexTriangles++ - 1); 
      p1[0]=*p++;//p[0]
      p1[1]=*p++;//p[1]
      p1[2]=*p;//p[2]
      
      //p = mexVertices[(int)3*(mexTriangles[3*j+1]-1)];
      p = mexVertices + 3 * (int)(*mexTriangles++ - 1); 
      p2[0]=*p++;
      p2[1]=*p++;
      p2[2]=*p;
      
      p = mexVertices + 3 * (int)(*mexTriangles++ - 1); 
      p3[0]=*p++;
      p3[1]=*p++;
      p3[2]=*p;
      
      body.AddTri(p1,p2,p3,j); // add triangle p
    }
  body.EndModel();	   // end (build) the model
}
