#include <math.h>

#include <matVec.h>
#include <signalProcessing.h>

#define MAX_BUFFER 256
#define SAMPLING_RATIO 5
#define INITIME 0
#define ENDTIME 15

int main()
{
  int i,j;
  char line[MAX_BUFFER];
  char fileName[MAX_BUFFER];
  
  int nSamples;
  int nResamples = round((ENDTIME-INITIME)*SAMPLING_RATIO + 1);
  
  Vec time;
  Vec resampledTime(nResamples);
  for (i=0; i<nResamples; i++)
    resampledTime[i] = (double)INITIME + (double)i/(double)SAMPLING_RATIO;
  
  Vec motor, enc1, enc2, enc3, enc4;
  int resampledMotor, resampledEnc1, resampledEnc2, resampledEnc3, resampledEnc4;
  double BWmotor, BWenc1, BWenc2, BWenc3, BWenc4,BWenc;
  
  FILE *file,*fileResampled;
  
  for (i=1;i<=200; i++)
    {
      sprintf(fileName,"../../filtered/signatures/graspingMarkersP2_2010_05_28_signature%03d.txt",i);
      file=fopen(fileName, "r");
      sprintf(fileName,"../signatures/graspingMarkersP2_2010_05_28_signature%03d.txt",i);
      fileResampled=fopen(fileName, "w");
      //First we need to find the number of data points
      nSamples=0;
      while (fgets(line, MAX_BUFFER,file)!=NULL)
	nSamples++;
   
      motor  = Vec(nSamples);
      enc1 = Vec(nSamples);
      enc2 = Vec(nSamples);
      enc3 = Vec(nSamples);
      enc4 = Vec(nSamples);
      time = Vec(nSamples);
      
      //Now we get to the beginning of the file and read it
      fseek(file,0,SEEK_SET);
      for (j=0; j<nSamples; j++)
	fscanf(file,"%lf %lf %lf %lf %lf %lf\n",&time[j],&motor[j],&enc1[j],&enc2[j],&enc3[j],&enc4[j]);
      fclose(file);
      
      
      //Now we interpolate
      if(i==1)
	{
	  BWmotor = LWRBandwidth(time, motor);
	  BWenc1 = LWRBandwidth(time, enc1); 
	  BWenc2 = LWRBandwidth(time, enc2); 
	  BWenc3 = LWRBandwidth(time, enc3); 
	  BWenc4 = LWRBandwidth(time, enc4); 
	  printf("%.3lf   %.3lf   %.3lf   %.3lf   %.3lf\n",BWmotor, BWenc1, BWenc2, BWenc3, BWenc4);
	}

      for (j=0; j<nResamples; j++)
	{
	  if(resampledTime[j]<(time[nSamples-1]*1.1))
	    {
	      BWmotor = 50.0;
	      BWenc1 = 100.0; 
	      BWenc2 = 100.0; 
	      BWenc3 = 100.0; 
	      BWenc4 = 100.0; 
	    }
	  else
	    {
	      BWmotor = 5.0;
	      BWenc1 = 5.0; 
	      BWenc2 = 5.0; 
	      BWenc3 = 5.0; 
	      BWenc4 = 5.0; 
	    }
	  resampledMotor = LWR(resampledTime[j], time, motor, BWmotor);
	  resampledEnc1 = round(LWR(resampledTime[j], time, enc1, BWenc1));
	  resampledEnc2 = round(LWR(resampledTime[j], time, enc2, BWenc2));
	  resampledEnc3 = round(LWR(resampledTime[j], time, enc3, BWenc3));
	  resampledEnc4 = round(LWR(resampledTime[j], time, enc4, BWenc4));
	  fprintf(fileResampled,"%.2lf %06d %04d %04d %04d %04d\n",resampledTime[j],resampledMotor, resampledEnc1, resampledEnc2, resampledEnc3, resampledEnc4);
	}
      fclose(fileResampled);      
    }
}
