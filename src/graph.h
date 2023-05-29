#ifndef TRAVELLINGSALESMAN_GRAPH_H
#define TRAVELLINGSALESMAN_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <tuple>
#include <list>
#include <algorithm>
#include <list>

#include "vertex.h"
#include "station.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    std::vector<Vertex *> vertexSet;    // vertex set
    std::unordered_map<std::string, Vertex *> idToVertex;

  public:
    Graph();

    [[nodiscard]] Vertex *findVertex(const std::string &id) const; //

    bool addVertex(const std::string &id); //

    [[nodiscard]] unsigned int getNumVertex() const; //

    [[nodiscard]] std::vector<Vertex *> getVertexSet() const; //

    std::pair<Edge *, Edge *>
    addAndGetBidirectionalEdge(const std::string &source, const std::string &dest, unsigned int dist);

    static void activateEdges(const std::vector<Edge *> &Edges);

    static void deactivateEdges(const std::vector<Edge *> &edges);

    [[nodiscard]] unsigned int getTotalEdges() const; //

    void visitedDFS(Vertex *source);

};

#endif //TRAVELLINGSALESMAN_GRAPH_H