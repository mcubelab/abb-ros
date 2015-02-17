#Makefile for demo Program for asynchronous robot control

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
LIBS     = -lmatVec
ODIR     = bin

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
# If mac, SFML is linked differently
SFML_LIBS = -framework sfml-network -framework sfml-system -framework sfml-window -framework sfml-graphics
else
# Link libraries normally in linux
SFML_LIBS = -lsfml-network -lsfml-system -lsfml-window -lsfml-graphics
endif

.PHONY: clean


all: $(ODIR)/planarMotionDemo 

$(ODIR)/planarMotionDemo: $(OBJS) src/planarMotionDemo.cpp
	$(CXX) $(CPPFLAGS) -o $(ODIR)/planarMotionDemo src/planarMotionDemo.cpp $(OBJS) $(LIBS) $(SFML_LIBS) ../../lib/libABBInterpreter.a -I../../../../../libraries/matVec/include -L../../../../../libraries/matVec/lib -lmatVec

clean:
	rm -f $(ODIR)/planarMotionDemo

ex: $(ODIR)/planarMotionDemo
	./$(ODIR)/planarMotionDemo
