//
// Created by rita on 12-03-2023.
//

#ifndef TRAVELLINGSALESMAN_EDGE_H
#define TRAVELLINGSALESMAN_EDGE_H

#include <memory>
#include "vertex.h"

class Vertex;

enum class Service : unsigned int {
    STANDARD = 0,
    ALFA_PENDULAR = 1,
    VERY_EXPENSIVE = 2
};

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

    [[nodiscard]] unsigned int getLength() const;

    void setLength(int length);

  private:
    Vertex *orig;
    Vertex *dest; // destination vertex

    // auxiliary fields
    bool selected = true;
    Edge *reverse = nullptr;

    unsigned int length;

};

#endif //TRAVELLINGSALESMAN_EDGE_H
