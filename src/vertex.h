#ifndef TRAVELLINGSALESMAN_VERTEX_H
#define TRAVELLINGSALESMAN_VERTEX_H

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include "edge.h"
#include "constants.h"
#include "coordinates.h"

class Edge;

class Vertex {
  public:
    Vertex(const unsigned int &id, Coordinates c = {0, 0});

    [[nodiscard]] unsigned int getId() const;

    [[nodiscard]] std::vector<std::shared_ptr<Edge>> getAdj() const;

    [[nodiscard]] bool isVisited() const;

    [[nodiscard]] bool isProcessing() const;

    [[nodiscard]] unsigned int getIndegree() const;

    [[nodiscard]] double getDist() const;

    [[nodiscard]] std::shared_ptr<Edge> getPath() const;

    [[nodiscard]] std::vector<std::shared_ptr<Edge>> getIncoming() const;

    void setId(const unsigned int &id);

    void setVisited(bool visited);

    void setProcesssing(bool processing);

    void setIndegree(unsigned int indegree);

    void setDist(double dist);

    void setPath(std::shared_ptr<Edge> path);

    [[nodiscard]] const Coordinates &getCoordinates() const;

    void setCoordinates(const Coordinates &coordinates);

    std::shared_ptr<Edge> addEdge(const std::shared_ptr<Vertex> &dest, double length);

    bool removeEdge(const unsigned int &destID);


  private:
    unsigned int id;                // identifier
    std::vector<std::shared_ptr<Edge>> adj;  // outgoing edges
    Coordinates coordinates;

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    unsigned int indegree = 0; // used by topsort

    double dist; //TODO replace by unsorted_map to store distances between every pair of vertices
    std::shared_ptr<Edge> path = nullptr;
    std::vector<std::shared_ptr<Edge>> incoming; // incoming edges

    unsigned int selectedCount = 0;

};

struct VertexPointerHash {
    std::size_t operator()(const std::shared_ptr<Vertex> &vertex) const {
        return std::hash<unsigned int>()(vertex->getId());
    }
};

struct VertexPointerEquals {
    bool operator()(const std::shared_ptr<Vertex> &vertex1, const std::shared_ptr<Vertex> &vertex2) const {
        return vertex1->getId() == vertex2->getId();
    }
};

typedef std::unordered_set<std::shared_ptr<Vertex>, VertexPointerHash, VertexPointerEquals> VertexPointerTable;


#endif //TRAVELLINGSALESMAN_VERTEX_H
