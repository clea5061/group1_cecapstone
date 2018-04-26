#!/usr/bin/bash

g++ -std=c++11 -Wall -o lane-detector -lopencv_core -lopencv_imgproc -lopencv_imgcodecs lane-detector.cpp
