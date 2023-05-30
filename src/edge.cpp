#include "edge.h"

Edge::Edge(Vertex *orig, Vertex *dest, unsigned int length) {
    this->orig = orig;
    this->dest = dest;
    this->length = length;
}

Vertex *Edge::getDest() const {
    return this->dest;
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

double Edge::getLength() const {
    return length;
}

void Edge::setLength(double length) {
    this->length = length;
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
