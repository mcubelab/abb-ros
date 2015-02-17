#include "vrmlIO.h"

/////////////////////////////
// VRML files
/////////////////////////////
int readWRL(const char *File_Name, std::vector<std::vector<double> > &vertices, std::vector<std::vector<int> > &triangles)
{
    char *aux;
    char line[MAX_LINE];
    char *linePointer;
    FILE *wrlFile;
    bool endLine;
    
    //Auxiliary vertex and triangle structures
    std::vector<double> vertex (3,0.0);
    std::vector<int> triangle (3,0);
    
    //Open the VRML file
    wrlFile = fopen(File_Name,"r");
    if(!wrlFile)
      {
	printf("File not found.\n");
	return -1;
      }


    long iniPos=0;
    long endPos=0;
    int accumulatedNVertices = 0;
    while((iniPos=fileSearch(wrlFile, "point", iniPos)) != -1)
      {
	// Read the list of vertices
	iniPos=fileSearch(wrlFile, "[",iniPos);
	endPos=fileSearch(wrlFile, "]", iniPos);
	fseek(wrlFile,iniPos + 1 ,SEEK_SET);
	while(ftell(wrlFile)<endPos)
	  {
	    aux = fgets(line,MAX_LINE,wrlFile);
	    linePointer=&line[0];
	    endLine=false;
	    do{
	      if(sscanf(linePointer,"%lf %lf %lf",&vertex[0], &vertex[1], &vertex[2])==3)
		{
		  vertices.push_back(vertex);
		}
	      linePointer=strstr(linePointer,",");
	      if(linePointer==NULL)
		endLine=true;
	      else
		linePointer++;
	    }while(!endLine);
	  }
	
	// Read the list of triangles
	iniPos = fileSearch(wrlFile, "coordIndex",iniPos);
	iniPos = fileSearch(wrlFile, "[",iniPos);
	endPos = fileSearch(wrlFile, "]", iniPos);
	fseek(wrlFile,iniPos +1 ,SEEK_SET);
	while(ftell(wrlFile)<endPos)
	  {
	    aux = fgets(line,MAX_LINE,wrlFile);
	    linePointer=&line[0];
	    endLine=false;
	    do{
	      if(sscanf(linePointer,"%d, %d, %d",&triangle[0], &triangle[1], &triangle[2])==3)
		{
		  triangle[0]+=accumulatedNVertices;
		  triangle[1]+=accumulatedNVertices;
		  triangle[2]+=accumulatedNVertices;
		  triangles.push_back(triangle);
		}
	      linePointer=strstr(linePointer,"-");
	      if(linePointer==NULL)
		endLine=true;
	      else
		linePointer+=3;
	    }while(!endLine);
	  }
	accumulatedNVertices = (int) vertices.size();
      }
      fclose(wrlFile);
    return 1;
}

//Searches for a string in a file and returns the position
long fileSearch(FILE* pFile, const char* searchString, long iniSearch)
{
    //make sure we were passed valid parameters
    if ((!pFile)||(!searchString))
    {
        return -1;
    }

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
        unsigned long aux = fread(localBuffer,1,stringSize,pFile);
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
