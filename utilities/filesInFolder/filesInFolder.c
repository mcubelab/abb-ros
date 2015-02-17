#include <stdio.h>
#include <dirent.h>
#include <unistd.h>
#include <string.h>

#define MAX_BUFFER 256
#define FILELIST_NAME "fileList.txt"

int main(int argc, char *argv[])
{
  char folder[MAX_BUFFER];
  char fileName[MAX_BUFFER];
  FILE *f;

  //Check arguments
  if(argc==2)
    strcpy(folder,argv[1]);
  else
    getcwd(folder, MAX_BUFFER);  
  printf("FILES_IN_FOLDER: Parsing folder \"%s\"...\n",folder);

  //Open folder
  struct dirent *de=NULL;
  DIR *d=NULL;
  d=opendir(folder);
  if(d == NULL)
    {
      printf("FILES_IN_FOLDER: Could not open the folder.\n");
      return 1;
    }
  
  // Loop through files
  sprintf(fileName,"%s/%s",folder,FILELIST_NAME);
  printf("0\n");
  printf("%s\n",fileName);
  f = fopen(fileName,"w");
  //fprintf(f,"%s",folder);
  int i=1;
  while(de = readdir(d))
    {
      if((de->d_name[0] != '.') && !strstr(de->d_name,FILELIST_NAME))
	{
	  printf("%d\n",i);
	  fprintf(f,"\n");
	  fprintf(f,"%s",de->d_name);
	  i++;
	}
    }

  fclose(f);
  printf("FILES_IN_FOLDER: Done.\n");
  return 0;
}
