cmake_minimum_required(VERSION 3.25)
project(TravellingSalesman)

set(CMAKE_CXX_STANDARD 23)

add_executable(TravellingSalesman src/main.cpp src/menu.h src/graph.h src/vertex.h src/vertex.cpp src/edge.cpp src/graph.cpp src/menu.cpp src/dataRepository.h src/dataRepository.cpp src/node.cpp src/UFDS.cpp src/coordinates.h src/coordinates.cpp)