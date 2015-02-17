#Makefile for processing data 

#Compiler: gcc 4.2

#Shared libraries
all: resampling plotGeneration inputMarkerPose

resampling: resampling.cpp
	c++ -O2 -Wall -ansi -o resampling resampling.cpp -lmatVec -lsignalProcessing
plotGeneration: plotGeneration.cpp
	c++ -O2 -Wall -ansi -o plotGeneration plotGeneration.cpp -framework sfml-window -framework sfml-graphics -framework sfml-system 
inputMarkerPose: inputMarkerPose.cpp
	c++ -O2 -Wall -ansi -o inputMarkerPose inputMarkerPose.cpp -lmatVec -framework sfml-window -framework sfml-system -framework sfml-graphics 

ex: all
	./resampling
	./plotGeneration