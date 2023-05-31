#include "vertex.h"

#include <utility>


Vertex::Vertex(const unsigned int &id) : id(id), dist(constants::INF) {}

/**
 * Adds a new outgoing edge to the Vertex, with a given destination and length
 * Time Complexity: O(1)
 * @param d - Pointer to the destination Vertex
 * @param length - Edge length
 * @return Pointer to the new Edge created
 */
std::shared_ptr<Edge> Vertex::addEdge(const std::shared_ptr<Vertex>& d, double length) {
    auto newEdge = std::make_shared<Edge>(std::make_shared<Vertex>(*this), d, length);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    d->indegree++;
    return newEdge;
}

/**
 * Removes an outgoing edge, with a given destination, from the Vertex
 * Time Complexity: O(indegree(v) * outdegree(v))
 * @param destID - Id of the destination Vertex of the Edge to be removed
 * @return True if successful, and false if no such Edge exists
 */
bool Vertex::removeEdge(const unsigned int &destID) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        std::shared_ptr<Edge>edge = *it;
        std::shared_ptr<Vertex>dest = edge->getDest();
        if (dest->getId() == destID) {
            it = adj.erase(it);
            // Also remove the corresponding edge from the incoming list
            auto it2 = dest->incoming.begin();
            while (it2 != dest->incoming.end()) {
                if ((*it2)->getOrig()->getId() == id) {
                    it2 = dest->incoming.erase(it2);
                } else {
                    it2++;
                }
            }
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        } else {
            it++;
        }
    }
    return removedEdge;
}

unsigned int Vertex::getId() const {
    return this->id;
}

std::vector<std::shared_ptr<Edge>> Vertex::getAdj() const {
    return this->adj;
}

bool Vertex::isVisited() const {
    return this->visited;
}

bool Vertex::isProcessing() const {
    return this->processing;
}

unsigned int Vertex::getIndegree() const {
    return this->indegree;
}


std::shared_ptr<Edge>Vertex::getPath() const {
    return this->path;
}

std::vector<std::shared_ptr<Edge>> Vertex::getIncoming() const {
    return this->incoming;
}

void Vertex::setId(const unsigned int &id) {
    this->id = id;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

void Vertex::setProcesssing(bool processing) {
    this->processing = processing;
}

void Vertex::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}

void Vertex::setDist(double dist) {
    this->dist = dist;
}

void Vertex::setPath(std::shared_ptr<Edge>path) {
    this->path = std::move(path);
}

double Vertex::getDist() const {
    return this->dist;
}

