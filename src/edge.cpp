#include "edge.h"

Edge::Edge(std::shared_ptr<Vertex>orig, std::shared_ptr<Vertex> dest, double length) {
    this->orig = std::move(orig);
    this->dest = std::move(dest);
    this->length = length;
}

std::shared_ptr<Vertex>Edge::getDest() const {
    return this->dest;
}

std::shared_ptr<Vertex>Edge::getOrig() const {
    return this->orig;
}

std::shared_ptr<Edge>Edge::getReverse() const {
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

void Edge::setReverse(std::shared_ptr<Edge>r) {
    this->reverse = std::move(r);
}

bool Edge::operator<(const Edge &rhs) const{
    return this->length < rhs.getLength();
}

void Edge::print() const {
    std::cout << orig->getId() << " <-> " << dest->getId() << std::endl;
}

bool Edge::operator==(const Edge &rhs) const {
    return orig == rhs.orig &&
           dest == rhs.dest &&
           selected == rhs.selected &&
           reverse == rhs.reverse &&
           length == rhs.length;
}

bool Edge::operator!=(const Edge &rhs) const {
    return !(rhs == *this);
}
