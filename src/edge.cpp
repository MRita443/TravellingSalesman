#include "edge.h"

Edge::Edge(Vertex *orig, Vertex *dest, unsigned int distance) {
    this->orig = orig;
    this->dest = dest;
    this->distance = distance;
}

Vertex *Edge::getDest() const {
    return this->dest;
}

unsigned int Edge::getDist() const {
    return this->distance;
}

Vertex *Edge::getOrig() const {
    return this->orig;
}

Edge *Edge::getReverse() const {
    return this->reverse;
}

bool Edge::isSelected() const {
    return this->selected;
}

void Edge::setSelected(bool s) {
    this->selected = s;
}

void Edge::setReverse(Edge *r) {
    this->reverse = r;
}

void Edge::print() const {
    std::cout << orig->getId() << " <-> " << dest->getId() << std::endl;
}
