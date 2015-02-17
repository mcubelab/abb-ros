#include "fileio.h"
/////////////////////////////
// General Mat and Vec io
/////////////////////////////

void writeVec( FILE *f, Vec & vec) 
{
  fprintf(f,"%d\n",vec.nn);
  for (int i=0;i<vec.nn;++i)
  fprintf(f,"%.16g\n", vec[i]);
}

void writeMat( FILE *f, Mat & mat) 
{
  fprintf(f,"%d %d\n",mat.nn, mat.mm);
  for (int i=0;i<mat.nn;++i)
  {
    for (int j=0;j<mat.mm-1;++j)
      fprintf(f,"%.16g ", mat[i][j]);
    fprintf(f,"%.16g\n", mat[i][mat.mm]);
  }
}

void readVec( FILE *f, Vec & vec) 
{
  int n;
  if(fscanf(f,"%d\n", &n)!=1)
    return;
  vec = Vec(n);
  for (int i=0;i<vec.nn;++i)
    if(fscanf(f,"%lf", &vec[i])!=1)
      return;
}
void readMat( FILE *f, Mat & mat) 
{
  int n,m;
  if(fscanf(f,"%d %d\n", &n, &m)!=2)
    return;
  mat = Mat(n,m);
  for (int i=0;i<mat.nn;++i)
  {
    for (int j=0;j<mat.mm;++j)
      if(fscanf(f,"%lf", &mat[i][j])!=1)
        return;
  }
}

/////////////////////////////
// Vertex - Triangle files
/////////////////////////////
void readVTF(const char *File_Name, Mat &Vert, Mat &Tri)
{
  FILE *vtfFile;
  vtfFile = fopen(File_Name,"r");

  int nVert;
  if(fscanf(vtfFile,"%d\n",&nVert)!=1) //Number of Vertices
    return;	
  Vert = Mat(nVert,3);
  for (int i=0 ; i< nVert ; i++)
  {
    if(fscanf(vtfFile, "%lf %lf %lf\n", &Vert[i][0], &Vert[i][1], &Vert[i][2])!=3)
      return; 
  }  
  int nTri;
  if(fscanf(vtfFile,"%d\n",&nTri)!=1)	//Number of Triangles		
    return;
  Tri = Mat(nTri,3);
  for (int i=0 ; i< nTri ; i++)
  {
    if(fscanf(vtfFile, "%lf %lf %lf\n", &Tri[i][0], &Tri[i][1], &Tri[i][2])!=3)
      return;
  }
  fclose(vtfFile);
}

/////////////////////////////
// VRML files
/////////////////////////////
bool readWRL(const char *File_Name, Mat &Vert, Mat &Tri)
{
  int nTri=0, nVert=0;
  int nTotalTri=0, nTotalVert=0;
  char line[MAX_LINE];
  char * linePointer;
  double aux0, aux1, aux2;
  FILE *wrlFile;
  bool endLine;
  Mat auxVert;
  Mat auxTri;

  wrlFile = fopen(File_Name,"r");
  if(!wrlFile)
  {
    //printf("ALERT\n");
    return false;
  }
  /////////////////////////////////////////////////
  //Find the total number of Vertices and Triangles
  long iniPos=0;
  long endPos=0;
  while((iniPos=fileSearch(wrlFile, "point", iniPos)) != -1)
  {
    //First we need to find the number of vertices
    nVert=0;
    iniPos=fileSearch(wrlFile,"[",iniPos);
    endPos=fileSearch(wrlFile,"]",iniPos);
    fseek(wrlFile,iniPos + 1,SEEK_SET);
    while(ftell(wrlFile)<endPos)
    {
      if(fgets(line,MAX_LINE,wrlFile)!=NULL)
      {
        linePointer=&line[0];
        endLine=false;
        do
        {
          if(sscanf(linePointer,"%lf %lf %lf",&aux0, &aux1, &aux2)==3)
           nVert++;	      
          linePointer=strstr(linePointer,",");
	  if(linePointer==NULL)
            endLine=true;
          else
            linePointer++;
        }while(!endLine);
      }
      else
        return false; 
    }

    //Triangles
    nTri=0;
    iniPos = fileSearch(wrlFile, "coordIndex", iniPos);
    iniPos=fileSearch(wrlFile,"[",iniPos);
    endPos=fileSearch(wrlFile,"]",iniPos);
    fseek(wrlFile,iniPos + 1,SEEK_SET);
    while(ftell(wrlFile)<endPos)
    {
      if(fgets(line,MAX_LINE,wrlFile)!=NULL)
      {
        linePointer=&line[0];
        endLine=false;
        do
        {
          if(sscanf(linePointer,"%lf, %lf, %lf",&aux0, &aux1, &aux2)==3)
            nTri++;	      
          linePointer=strstr(linePointer,"-");
          if(linePointer==NULL)
            endLine=true;
          else
            linePointer+=3;
        }while(!endLine);
      }
      else
        return false;
    }
    nTotalVert+=nVert;
    nTotalTri+=nTri;
  }
 
  /////////////////////////////////////////////////
  //Read those same Vertices and Triangles
  Vert = Mat(nTotalVert,3);
  Tri = Mat(nTotalTri,3);
    
  iniPos=0;
  nVert=0;
  nTotalVert=0;
  nTri=0;
  nTotalTri=0;
  while((iniPos=fileSearch(wrlFile, "point", iniPos)) != -1)
  {
    //Now we read the vertices
    nVert=0;
    iniPos=fileSearch(wrlFile, "[",iniPos);
    endPos=fileSearch(wrlFile, "]", iniPos);
    fseek(wrlFile,iniPos + 1 ,SEEK_SET);
    while(ftell(wrlFile)<endPos)
    {
      if(fgets(line,MAX_LINE,wrlFile)!=NULL)
      {
        linePointer=&line[0];
        endLine=false;
        do
        {
          if(sscanf(linePointer,"%lf %lf %lf",&aux0, &aux1, &aux2)==3)
          {
            Vert[nTotalVert+nVert][0] = aux0;
            Vert[nTotalVert+nVert][1] = aux1;
            Vert[nTotalVert+nVert][2] = aux2;
            nVert++;	      
          }
          linePointer=strstr(linePointer,",");
          if(linePointer==NULL)
            endLine=true;
          else
            linePointer++;
        }while(!endLine);
      }
      else
        return false;
    }

    //Triangles
    nTri=0;
    iniPos = fileSearch(wrlFile, "coordIndex",iniPos);
    iniPos=fileSearch(wrlFile, "[",iniPos);
    endPos=fileSearch(wrlFile, "]", iniPos);
    fseek(wrlFile,iniPos +1 ,SEEK_SET);
    while(ftell(wrlFile)<endPos)
    {
      if(fgets(line,MAX_LINE,wrlFile)!=NULL)
      {
        linePointer=&line[0];
        endLine=false;
        do
        {
          if(sscanf(linePointer,"%lf, %lf, %lf",&aux0, &aux1, &aux2)==3)
          {
            Tri[nTotalTri + nTri][0] = aux0 + nTotalVert;
            Tri[nTotalTri + nTri][1] = aux1 + nTotalVert;
            Tri[nTotalTri + nTri][2] = aux2 + nTotalVert;
            nTri++;	      
          }
          linePointer=strstr(linePointer,"-");
          if(linePointer==NULL)
            endLine=true;
          else
            linePointer+=3;
        }while(!endLine);
      }
      else
        return false;
    }
    nTotalVert+=nVert;
    nTotalTri+=nTri;
  }
  fclose(wrlFile);
  return true;
}

//Searches for a string in a file and returns the position
long fileSearch(FILE* pFile, const char* searchString, long iniSearch)
{
  //make sure we were passed valid parameters
  if ((!pFile)||(!searchString))
    return -1;
 
  unsigned long fileSize=0;

  //get the size of the file
  fseek(pFile,0,SEEK_END);
  fileSize=ftell(pFile);
  fseek(pFile,0,SEEK_SET);

  //if the file is empty return -1
  if (!fileSize)
    return -1;

  //get the length of the string we're looking for
  unsigned long stringSize=strlen(searchString);
  if (stringSize>fileSize)
    return -1;

  //allocate the memory for the local buffer
  char* localBuffer=(char*)malloc(stringSize);
  if (!localBuffer)
    return -1;

  unsigned long currentPosition=iniSearch;
  while (currentPosition<fileSize-stringSize)
  {
    fseek(pFile,currentPosition,SEEK_SET);
    size_t result = fread(localBuffer,1,stringSize,pFile);
    if (result != stringSize)
      return -1;
  
    if (!memcmp(localBuffer,searchString,stringSize))
    {
      free(localBuffer);
      return currentPosition;
    }
    currentPosition++;
  }
  free(localBuffer);
  return -1;
} 
