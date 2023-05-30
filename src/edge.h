#ifndef TRAVELLINGSALESMAN_EDGE_H
#define TRAVELLINGSALESMAN_EDGE_H

#include <memory>
#include "vertex.h"

class Vertex;

class Edge {
  public:
    Edge(Vertex *orig, Vertex *dest, unsigned int w);

    [[nodiscard]] Vertex *getDest() const;

    [[nodiscard]] unsigned int getDist() const;

    [[nodiscard]] bool isSelected() const; //isOpen

    [[nodiscard]] Vertex *getOrig() const;

    [[nodiscard]] Edge *getReverse() const;

    void setSelected(bool s) const;

    void setReverse(Edge *r);

    void print() const;

  private:
    Vertex *orig;
    Vertex *dest; // destination vertex
    unsigned int distance;

    // auxiliary fields
    bool selected = true;
    Edge *reverse = nullptr;

};

#endif //TRAVELLINGSALESMAN_EDGE_H
