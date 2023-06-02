cmake_minimum_required(VERSION 3.24)
project(TravellingSalesman)

set(CMAKE_CXX_STANDARD 23)

add_executable(TravellingSalesman
        src/main.cpp
        src/menu.h src/menu.cpp
        src/graph.h src/graph.cpp
        src/vertex.h src/vertex.cpp
        src/edge.h src/edge.cpp
        src/dataRepository.h src/dataRepository.cpp
        src/MutablePriorityQueue.h
        src/coordinates.h src/coordinates.cpp
        src/constants.h
        )
