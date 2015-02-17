#
#Makefile for scanPorts
#

CXX      = g++
CPPFLAGS = -O2 -Wall -ansi
OUT      = bin/scanPorts

.PHONY: clean

all: $(OUT)

$(OUT): src/scanPorts.cpp
	$(CXX) $(CPPFLAGS) -o $(OUT) src/scanPorts.cpp -lserialPort -L../../code/libraries/serialPort/lib
clean: 
	rm -f $(OUT)


#Extra
ex: $(OUT)
	./$(OUT) 1

