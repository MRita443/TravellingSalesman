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

#include "UFDS.h"
#include "vertex.h"
#include "station.h"

class Graph {
  private:
    unsigned int totalEdges = 0;
    std::vector<Vertex *> vertexSet;    // vertex set
    std::unordered_map<unsigned int, Vertex *> idToVertex;
    std::pair<unsigned int, std::list<Vertex *>> tour;

  public:
    Graph();

    [[nodiscard]] Vertex *findVertex(const unsigned int &id) const;

    [[nodiscard]] Edge *findEdge(const unsigned int &v1id, const unsigned int &v2id) const;

    bool addVertex(const unsigned int &id); //

    [[nodiscard]] unsigned int getNumVertex() const; //

    [[nodiscard]] std::vector<Vertex *> getVertexSet() const; //

    std::pair<Edge *, Edge *>
    addAndGetBidirectionalEdge(const unsigned int &source, const unsigned int &dest, unsigned int dist);

    static void setSelectedEdge(Edge *edge, bool selected);

    static void activateEdges(const std::vector<Edge *> &Edges);

    static void deactivateEdges(const std::vector<Edge *> &edges);

    [[nodiscard]] unsigned int getTotalEdges() const; //

    void visitedDFS(Vertex *source);

    bool isConnectable(Vertex *candidate) const;

    void preorderMSTTraversal(Vertex *source);

    void kruskal();

    void triangularTSPTour();

};

#endif //TRAVELLINGSALESMAN_GRAPH_H
