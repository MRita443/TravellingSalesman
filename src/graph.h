#ifndef TRAVELLINGSALESMAN_GRAPH_H
#define TRAVELLINGSALESMAN_GRAPH_H

#include <vector>
#include <memory>
#include "vertex.h"
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

    [[nodiscard]] std::pair<std::vector<unsigned int>, double>
    getInsertionEdges(std::vector<unsigned int> tour, unsigned int newVertexId) const;


    double nearestInsertionLoop(unsigned int &start);

    void clearGraph();

    std::pair<unsigned int, unsigned int> getNextHeuristicEdge(std::vector<unsigned int> tour, UFDS tourSets);
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
