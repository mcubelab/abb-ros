#
#Makefile for mexCollide. MEX wrapper for PQP collision detection
#

CXX      = gcc
CXXMEX   = mex
CPPFLAGS = -Wall -O2 -fPIC -ffast-math
#-arch x86_64
OUT      = bin
PQPLIB   = -lPQP -L./pqp-1.3/lib -I./pqp-1.3/include

.PHONY: clean

all: pqp $(OUT)/vrmlIO.o $(OUT)/mexCollideLoadModel $(OUT)/mexCollideDetect cleanup

pqp:
	make -C pqp-1.3 

$(OUT)/vrmlIO.o: src/vrmlIO.h 
	$(CXX) $(CPPFLAGS) -c src/vrmlIO.cpp -o $(OUT)/vrmlIO.o

$(OUT)/mexCollideDetect: src/mexCollideDetect.cpp pqp
	$(CXXMEX) src/mexCollideDetect.cpp $(PQPLIB) -o $(OUT)/mexCollideDetect 

$(OUT)/mexCollideLoadModel: src/mexCollideLoadModel.cpp $(OUT)/vrmlIO.o
	$(CXXMEX) src/mexCollideLoadModel.cpp $(OUT)/vrmlIO.o -o $(OUT)/mexCollideLoadModel 

cleanup:
	rm $(OUT)/*.o

clean:
	rm $(OUT)/*.*
