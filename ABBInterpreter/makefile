#Makefile for ABBInterpreter library

PROJECT  = ABBInterpreter
CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
LIB      = lib/lib$(PROJECT).a
SDIR     = samples
OBJ    = ABBInterpreter.o

LBITS := $(shell getconf LONG_BIT)

# Add flags if system is 64-bit
#ifeq ($(LBITS),64)
#CPPFLAGS += -arch x86_64
#endif

.PHONY: clean

all: $(LIB) clean 

$(LIB): $(OBJ)
	mkdir -p lib
	ar -r $(LIB) $(OBJ)

#libraries

$(OBJ): include/ABBInterpreter.h
	$(CXX) $(CPPFLAGS) -c -o $(OBJ) src/ABBInterpreter.cpp -Iinclude	

clean:
	rm -rf $(OBJ) doc/html

purge: clean
	rm -rf $(LIB)



