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

all: $(LIB) include clean doc
#all: $(LIB) include clean doc

$(LIB): $(OBJ)
	ar -r $(LIB) $(OBJ)

#libraries

$(OBJ): src/ABBInterpreter.h
	$(CXX) $(CPPFLAGS) -c -o $(OBJ) src/ABBInterpreter.cpp	

include: $(LIB)
	cp src/ABBInterpreter.h include/ABBInterpreter.h

clean:
	rm -f $(OBJ)
	+make -C $(SDIR)/planarMotionDemo clean

doc: $(LIB)
	doxygen doc/DoxyfileHTML
#	doxygen doc/DoxyfilePDF
#	make -C doc/latex >log
#	rm log
#	mv doc/latex/refman.pdf doc/$(PROJECT)Doc.pdf

#sample program
samples: $(LIB) $(SDIR)/planarMotionDemo/planarMotionDemo

$(SDIR)/planarMotionDemo/planarMotionDemo:
	+make -C $(SDIR)/planarMotionDemo

ex: $(SDIR)/planarMotionDemo/bin/planarMotionDemo
	./$(SDIR)/planarMotionDemo/bin/planarMotionDemo
