#include "vertex.h"

Vertex::Vertex(const unsigned int &id, Coordinates c) : id(id), coordinates(c) {}

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


double Vertex::haversineDistance(const std::shared_ptr<Vertex> &other) {
    return coordinates.distanceTo(other->getCoordinates());
}//TODO reimplement

const Coordinates &Vertex::getCoordinates() const {
    return coordinates;
}

void Vertex::setCoordinates(const Coordinates &coordinates) {
    Vertex::coordinates = coordinates;
}
