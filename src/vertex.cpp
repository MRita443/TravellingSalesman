#include "vertex.h"

Vertex::Vertex(const unsigned int &id, Coordinates c) : id(id), coordinates(c) {}

/**
 * Adds a new outgoing edge to the Vertex, with a given destination and length
 * Time Complexity: O(1)
 * @param d - Pointer to the destination Vertex
 * @param length - Edge length
 * @return Pointer to the new Edge created
 */
/*std::shared_ptr<Edge> Vertex::addEdge(const std::shared_ptr<Vertex> &d, double length) {
    auto newEdge = std::make_shared<Edge>(std::make_shared<Vertex>(*this), d, length);
    adj.push_back(newEdge);
    d->indegree++;
    return newEdge;
}*/

/**
 * Removes an outgoing edge, with a given destination, from the Vertex
 * Time Complexity: O(indegree(v) * outdegree(v))
 * @param destID - Id of the destination Vertex of the Edge to be removed
 * @return True if successful, and false if no such Edge exists
 */
/*
bool Vertex::removeEdge(const unsigned int &destID) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        std::shared_ptr<Edge> edge = *it;
        std::shared_ptr<Vertex> dest = edge->getDest();
        if (dest->getId() == destID) {
            it = adj.erase(it);
            // Also remove the corresponding edge from the incoming list
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        } else {
            it++;
        }
    }
    return removedEdge;
}
*/

unsigned int Vertex::getId() const {
    return this->id;
}


bool Vertex::isVisited() const {
    return this->visited;
}

unsigned int Vertex::getIndegree() const {
    return this->indegree;
}


unsigned int Vertex::getPath() const {
    return this->path;
}

void Vertex::setId(const unsigned int &id) {
    this->id = id;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

void Vertex::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}


void Vertex::setPath(unsigned int path) {
    this->path = path;
}


const Coordinates &Vertex::getCoordinates() const {
    return coordinates;
}

void Vertex::setCoordinates(const Coordinates &coordinates) {
    Vertex::coordinates = coordinates;
}

