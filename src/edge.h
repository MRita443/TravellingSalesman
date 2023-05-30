#ifndef TRAVELLINGSALESMAN_EDGE_H
#define TRAVELLINGSALESMAN_EDGE_H

#include <memory>
#include <set>
#include "vertex.h"

class Vertex;

class Edge {
  public:
    Edge(std::shared_ptr<Vertex> orig, std::shared_ptr<Vertex> dest, unsigned int w);

    [[nodiscard]] std::shared_ptr<Vertex> getDest() const;

    [[nodiscard]] bool isSelected() const; //isOpen

    [[nodiscard]] std::shared_ptr<Vertex> getOrig() const;

    [[nodiscard]] std::shared_ptr<Edge> getReverse() const;

    void setSelected(bool s);

    void setReverse(std::shared_ptr<Edge> r);

    void print() const;

    [[nodiscard]] double getLength() const;

    void setLength(double length);

    bool operator==(const Edge &rhs) const;

    bool operator!=(const Edge &rhs) const;

  private:
    std::shared_ptr<Vertex> orig;
    std::shared_ptr<Vertex> dest; // destination vertex

    // auxiliary fields
    bool selected = true;
    std::shared_ptr<Edge> reverse = nullptr;

    double length;
};


struct EdgeLowestLength {
    bool operator()(const Edge &edge1, const Edge &edge2) const {
        return edge1.getLength() < edge2.getLength();
    }
};

typedef std::set<Edge, EdgeLowestLength> edgeSet;

#endif //TRAVELLINGSALESMAN_EDGE_H
