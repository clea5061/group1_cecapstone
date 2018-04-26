#!/bin/bash

g++ -std=c++11 -pthread -Wall -o rover rover.cpp ArduinoPWM.cpp RoverApp.cpp
