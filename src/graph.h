#ifndef TRAVELLINGSALESMAN_GRAPH_H
#define TRAVELLINGSALESMAN_GRAPH_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <tuple>
#include <list>
#include <algorithm>
#include "vertex.h"
#include "node.h"
#include "UFDS.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    std::vector<Vertex *> vertexSet;    // vertex set
    std::unordered_map<std::string, Vertex *> idToVertex;

  public:
    Graph();

    [[nodiscard]] Vertex *findVertex(const unsigned int &id) const;

    bool addVertex(const unsigned int &id);

    [[nodiscard]] unsigned int getNumVertex() const;

    [[nodiscard]] std::vector<Vertex *> getVertexSet() const;

    std::vector<Edge *> randomlySelectEdges(unsigned int numEdges);

    static void activateEdges(const std::vector<Edge *> &Edges);

    [[nodiscard]] unsigned int getTotalEdges() const;

    void visitedDFS(Vertex *source);

    static void deactivateEdges(const std::vector<Edge *> &edges);

    bool addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, unsigned int length);

    [[nodiscard]] unsigned int getNumVertex() const; //

    [[nodiscard]] std::vector<Vertex *> getVertexSet() const; //


    static void setSelectedEdge(const Edge *edge, bool selected);

    static void activateEdges(const std::vector<Edge *> &Edges);

    static void deactivateEdges(const std::vector<Edge *> &edges);

    [[nodiscard]] unsigned int getTotalEdges() const; //

    void visitedDFS(Vertex *source);

    void dfsKruskalPath(Vertex *source);

    static void kruskal();
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
