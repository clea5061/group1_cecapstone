#!/bin/bash

g++ -std=c++11 -pthread -Wall -o base-station -lSDL2 -Iinclude src/base-station.cpp src/RemoteCtlApp.cpp
