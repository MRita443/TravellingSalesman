#ifndef TRAVELLINGSALESMAN_VERTEX_H
#define TRAVELLINGSALESMAN_VERTEX_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include "edge.h"

#define INF std::numeric_limits<double>::max()

class Edge;
enum class Service: unsigned int;

class Vertex {
public:
    explicit Vertex(unsigned int id);

    [[nodiscard]] unsigned int getId() const;

    [[nodiscard]] std::vector<Edge *> getAdj() const;

    [[nodiscard]] bool isVisited() const;

    [[nodiscard]] bool isProcessing() const;

    [[nodiscard]] unsigned int getIndegree() const;

    [[nodiscard]] int getDist() const;

    [[nodiscard]] Edge *getPath() const;

    [[nodiscard]] std::vector<Edge *> getIncoming() const;

    void setId(unsigned int info);

    void setVisited(bool visited);

    void setProcesssing(bool processing);

    void setIndegree(unsigned int indegree);

    void setDist(int dist);

    void setPath(Edge *path);

    Edge *addEdge(Vertex *dest, unsigned int w);

    bool removeEdge(const unsigned int& destID);

private:
    unsigned int id;                // identifier
    std::vector<Edge *> adj;  // outgoing edges

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    unsigned int indegree; // used by topsort
    int dist; //TODO replace by unsorted_map to store distances between every pair of vertices
    Edge *path = nullptr;
    std::vector<Edge *> incoming; // incoming edges

};


#endif //TRAVELLINGSALESMAN_VERTEX_H
