#ifndef TRAVELLINGSALESMAN_EDGE_H
#define TRAVELLINGSALESMAN_EDGE_H

#include <memory>
#include "vertex.h"

class Vertex;

class Edge {
  public:
    Edge(Vertex *orig, Vertex *dest, unsigned int w);

    [[nodiscard]] Vertex *getDest() const;

    [[nodiscard]] bool isSelected() const; //isOpen

    [[nodiscard]] Vertex *getOrig() const;

    [[nodiscard]] Edge *getReverse() const;

    void setSelected(bool s);

    void setReverse(Edge *r);

    void print() const;

    [[nodiscard]] double getLength() const;

    void setLength(double length);

  private:
    Vertex *orig;
    Vertex *dest; // destination vertex

    // auxiliary fields
    bool selected = true;
    Edge *reverse = nullptr;

    double length;
};

#endif //TRAVELLINGSALESMAN_EDGE_H
