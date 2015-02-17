#!/bin/bash

#Process data from the PacketClient.exe into the format:
# Line 1: n (n=number of markers)
# Lines 2 to n+1: a,b,[c,d,e] (a=id, b=reported size, [c,d,e]=position vector. Note that c,d,e are floating point)

#if you turn on rigid bodies, this will give you the markers
#wine PacketClient.exe 192.168.56.101 192.168.56.1|egrep "(Marker [0-9]+)|(^Marker Count)"|egrep "id|^Marker Count"|sed -e 's/Marker Count: \([0-9]*\)/\1/' | sed -e 's/.*id=\([0-9]*\)\tsize=\([0-9\.]*\)\tpos=\(.*\)/\1,\2,\3/'


#if you have "unidentified markers", use this script
wine PacketClient.exe 192.168.56.101 192.168.56.1|egrep "(Marker [0-9]+)|(^Unidentified Marker Count)"|egrep "pos|^Unidentified Marker Count"|sed -e 's/Unidentified Marker Count : \([0-9]*\)/\1/' | sed -e 's/Marker \([0-9]*\) : pos = \(.*\)/\1,\2/'



