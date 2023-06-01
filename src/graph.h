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
#include "UFDS.h"
#include "coordinates.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    VertexPointerTable vertexSet;    // vertex set
    std::vector<std::vector<double>> distanceMatrix;

  public:
    Graph();

    [[nodiscard]] unsigned int getNumVertex() const;

    [[nodiscard]] VertexPointerTable getVertexSet() const;

    [[nodiscard]] unsigned int getTotalEdges() const;

    [[nodiscard]] std::shared_ptr<Vertex> findVertex(const unsigned int &id) const;

    std::shared_ptr<Vertex> addVertex(const unsigned int &id, Coordinates c = {0, 0});

    bool addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length);

    static void setSelectedEdge(const std::shared_ptr<Edge> &edge, bool selected);

    static void deactivateEdges(const std::vector<std::shared_ptr<Edge>> &edges);

    static void activateEdges(const std::vector<std::shared_ptr<Edge>> &Edges);

    void visitedDFS(const std::shared_ptr<Vertex> &source);

    void dfsKruskalPath(const std::shared_ptr<Vertex> &source);

    void kruskal();

    std::pair<double, unsigned int*> tspBT();

    void tspRecursion(unsigned int *currentSolution, double currentSolutionDist, unsigned int currentNodeIdx,
                      double &bestSolutionDist, unsigned int *bestSolution, unsigned int n);

    bool inSolution(unsigned int j, const unsigned int *solution, unsigned int n);

    long nearestInsertionLoop(const std::shared_ptr<Vertex> &start);

    void updateViableEdges(edgeSet &viableEdges, UFDS partialTour, unsigned int sourceId);

    std::shared_ptr<Edge> findEdge(const unsigned int &v1id, const unsigned int &v2id) const;

    std::pair<std::shared_ptr<Edge>, std::shared_ptr<Edge>>
    getInsertionEdges(const std::list<Edge> &possibleEdges, const std::shared_ptr<Vertex> &newVertex) const;

    void initDistanceMatrix();

    bool
    addBidirectionalEdge(const std::shared_ptr<Vertex> &source, const std::shared_ptr<Vertex> &dest, double length);
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
