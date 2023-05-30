//
// Created by rita on 12-03-2023.
//

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

unsigned int Edge::getLength() const {
    return length;
}

void Edge::setLength(int length) {
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



