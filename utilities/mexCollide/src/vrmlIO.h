//vrmlIO.h
#ifndef VRMLIO_H__INCLUDED
#define VRMLIO_H__INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#define MAX_LINE 1024


// Read a VRML file and returns vector structures with vertices and triangles
//
// vertices  [[vertex[0].x, vertex[0].y, vertex[0].z],
//            [vertex[1].x, vertex[1].y, vertex[1].z],
//           ...] 
// triangles [[triangle[0].vertex[0], triangle[0].vertex[1], triangle[0].vertex[2]],
//            [triangle[1].vertex[0], triangle[1].vertex[1], triangle[1].vertex[2]],
//           ...]
int readWRL(const char *File_Name, std::vector<std::vector<double> > &vertices, std::vector<std::vector<int> > &triangles);
long fileSearch(FILE* pFile, const char* searchString, long iniSearch=0);

#endif // VRMLIO_H__INCLUDED
