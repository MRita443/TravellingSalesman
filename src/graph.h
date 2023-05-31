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
#include "vertex.h"
#include "node.h"
#include "UFDS.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    VertexPointerTable vertexSet;    // vertex set

  public:
    Graph();

    [[nodiscard]] unsigned int getNumVertex() const;

    [[nodiscard]] VertexPointerTable getVertexSet() const;

    [[nodiscard]] unsigned int getTotalEdges() const;

    [[nodiscard]] std::shared_ptr<Vertex> findVertex(const unsigned int &id) const;

    bool addVertex(const unsigned int &id);

    bool addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length);

    static void setSelectedEdge(const std::shared_ptr<Edge> &edge, bool selected);

    static void deactivateEdges(const std::vector<std::shared_ptr<Edge>> &edges);

    static void activateEdges(const std::vector<std::shared_ptr<Edge>> &Edges);

    void visitedDFS(const std::shared_ptr<Vertex> &source);

    void dfsKruskalPath(const std::shared_ptr<Vertex> &source);

    void kruskal();

    unsigned int tspBT(const unsigned int **dists, unsigned int n, unsigned int *path);

    void tspRecursion(unsigned int *currentSolution, unsigned int currentSolutionDist, unsigned int currentNodeIdx,
                      unsigned int &bestSolutionDist, unsigned int *bestSolution, unsigned int n,
                      const unsigned int **dists);

    bool inSolution(unsigned int j, const unsigned int *solution, unsigned int n);
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
