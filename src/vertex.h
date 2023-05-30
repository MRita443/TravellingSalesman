//
// Created by rita on 12-03-2023.
//

#ifndef TRAVELLINGSALESMAN_VERTEX_H
#define TRAVELLINGSALESMAN_VERTEX_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include "edge.h"

class Edge;

class Vertex {
  public:
    explicit Vertex(const unsigned int &id);

    [[nodiscard]] unsigned int getId() const;

    [[nodiscard]] std::vector<Edge *> getAdj() const;

    [[nodiscard]] bool isVisited() const;

    [[nodiscard]] bool isProcessing() const;

    [[nodiscard]] unsigned int getIndegree() const;

    [[nodiscard]] int getCost() const;

    [[nodiscard]] Edge *getPath() const;

    [[nodiscard]] std::vector<Edge *> getIncoming() const;

    void setId(const unsigned int &id);

    void setVisited(bool visited);

    void setProcesssing(bool processing);

    void setIndegree(unsigned int indegree);

    void setCost(int dist);

    void setPath(Edge *path);

    Edge *addEdge(Vertex *dest, unsigned int w);

    bool removeEdge(const unsigned int &destID);

  private:
    unsigned int id;                // identifier
    std::vector<Edge *> adj;  // outgoing edges

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    unsigned int indegree = 0; // used by topsort
    int cost;
    Edge *path = nullptr;
    std::vector<Edge *> incoming; // incoming edges

};


#endif //TRAVELLINGSALESMAN_VERTEX_H
