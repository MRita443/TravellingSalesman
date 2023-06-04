#ifndef TRAVELLINGSALESMAN_VERTEX_H
#define TRAVELLINGSALESMAN_VERTEX_H

#include <limits>
#include <unordered_set>
#include <memory>
#include "constants.h"
#include "coordinates.h"

class Vertex {
  public:
    explicit Vertex(const unsigned int &id, Coordinates c = {0, 0});

    [[nodiscard]] unsigned int getId() const;

    [[nodiscard]] bool isVisited() const;

    [[nodiscard]] unsigned int getIndegree() const;


    [[nodiscard]] unsigned int getPath() const;

    void setId(const unsigned int &id);

    void setVisited(bool visited);

    void setIndegree(unsigned int indegree);


    void setPath(unsigned int path);

    [[nodiscard]] const Coordinates &getCoordinates() const;

    void setCoordinates(const Coordinates &coordinates);

  private:
    unsigned int id;                // identifier
    Coordinates coordinates;

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    unsigned int indegree = 0; // used by topsort

    unsigned int path = -1; //ID of the preceding vertex
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
