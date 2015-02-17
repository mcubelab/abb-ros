//fileio.h
#ifndef FILEIO_H__INCLUDED
#define FILEIO_H__INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <matVec/matVec.h>

#define MAX_LINE 512

//Helper function
long fileSearch(FILE* pFile, const char* searchString, long iniSearch=0);

// Generic Vec and Mat reading and writing
void writeVec( FILE *f, Vec & vec);
void readVec( FILE *f, Vec & vec);
void writeMat( FILE *f, Mat & mat);
void readMat( FILE *f, Mat & mat);

// Vertex-Triangle Files
void readVTF(const char *File_Name, Mat &Vert, Mat &Tri);

// VRML files 
bool readWRL(const char *File_Name, Mat& Vert, Mat& Tri);

#endif // FILEIO_H__INCLUDED
