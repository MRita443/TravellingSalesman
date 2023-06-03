#ifndef TRAVELLINGSALESMAN_GRAPH_H
#define TRAVELLINGSALESMAN_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <tuple>
#include <list>
#include <algorithm>
#include <memory>
#include <set>
#include "vertex.h"
#include "edge.h"
#include "UFDS.h"
#include "coordinates.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    std::vector<std::shared_ptr<Vertex>> vertexSet;    // vertex set
    std::vector<std::vector<double>> distanceMatrix;

  public:
    Graph();

    [[nodiscard]] unsigned int getNumVertex() const;

    [[nodiscard]] std::vector<std::shared_ptr<Vertex>> getVertexSet() const;

    [[nodiscard]] unsigned int getTotalEdges() const;

    [[nodiscard]] std::shared_ptr<Vertex> findVertex(const unsigned int &id) const;

    std::shared_ptr<Vertex> addVertex(const unsigned int &id, Coordinates c = {0, 0});

    void addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length);

    void visitedDFS(const std::shared_ptr<Vertex> &source);

    void dfsKruskalPath(const std::shared_ptr<Vertex> &source);

    void kruskal();

    std::pair<double, unsigned int*> tspBT();

    void tspRecursion(unsigned int *currentSolution, double currentSolutionDist, unsigned int currentNodeIdx,
                      double &bestSolutionDist, unsigned int *bestSolution, unsigned int n);

    bool inSolution(unsigned int j, const unsigned int *solution, unsigned int n);

    std::pair<unsigned int, unsigned int> getNextHeuristicEdge(std::vector<unsigned int> tour);

    [[nodiscard]] std::pair<std::vector<unsigned int>, double>
    getInsertionEdges(std::vector<unsigned int> tour, unsigned int newVertexId) const;


    double nearestInsertionLoop(unsigned int &start);

    std::shared_ptr<Edge> findEdge(const unsigned int &v1id, const unsigned int &v2id) const;

    std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>>
    getInsertionEdges(const std::list<Edge> &possibleEdges, const std::shared_ptr<Vertex> &newVertex) const;

    void initDistanceMatrix();

    std::vector<std::vector<double>> getDistanceMatrix();

    bool
    addBidirectionalEdge(const std::shared_ptr<Vertex> &source, const std::shared_ptr<Vertex> &dest, double length);
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
