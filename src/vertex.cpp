//
// Created by rita on 12-03-2023.
//

#include "vertex.h"


Vertex::Vertex(const unsigned int &id) : id(id) {}

/**
 * Adds a new outgoing edge to the Vertex, with a given destination and capacity
 * Time Complexity: O(1)
 * @param d - Pointer to the destination Vertex
 * @param w - Edge capacity
 * @param service - Service of the Edge
 * @return Pointer to the new Edge created
 */
Edge *Vertex::addEdge(Vertex *d, unsigned int w) {
    auto newEdge = new Edge(this, d, w);
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
        Edge *edge = *it;
        Vertex *dest = edge->getDest();
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
            delete edge;
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

std::vector<Edge *> Vertex::getAdj() const {
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

int Vertex::getCost() const {
    return this->cost;
}

Edge *Vertex::getPath() const {
    return this->path;
}

std::vector<Edge *> Vertex::getIncoming() const {
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

void Vertex::setCost(int cost) {
    this->cost = cost;
}

void Vertex::setPath(Edge *path) {
    this->path = path;
}