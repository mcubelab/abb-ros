#Makefile for capturing data for robot motion tuning

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
ODIR     = bin
SRC	 = src
ABB_PATH = ../../../code/nodes/robot/ABBInterpreter

UNAME := $(shell uname)

ifeq ($(UNAME), Darwin)
# If mac, SFML is linked differently
SFML_LIBS = -framework sfml-network -framework sfml-system
else
# Link libraries normally in linux
SFML_LIBS = -lsfml-network -lsfml-system
endif

.PHONY: clean

all: abb $(ODIR)/motionTest

abb:
	make -C $(ABB_PATH)

$(ODIR)/motionTest: $(OBJS) $(SRC)/motionTest.cpp
	$(CXX) $(CPPFLAGS) -o $(ODIR)/motionTest $(SRC)/motionTest.cpp $(ABB_PATH)/lib/libABBInterpreter.a $(SFML_LIBS) -I$(ABB_PATH)/include

clean:
	rm -f $(ODIR)/motionTest

ex: all
	./$(ODIR)/motionTest
