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

    [[nodiscard]] unsigned int getCapacity() const;

    [[nodiscard]] bool isSelected() const; //isOpen

    [[nodiscard]] Vertex *getOrig() const;

    [[nodiscard]] Edge *getReverse() const;

    [[nodiscard]] Service getService() const;

    [[nodiscard]] unsigned int getFlow() const;

    [[nodiscard]] int getCost() const;

    Edge *getCorrespondingEdge() const;

    void setSelected(bool s);

    void setReverse(Edge *r);

    void setService(Service s);

    void setFlow(unsigned int f);

    void setCapacity(unsigned int c);

    void setCorrespondingEdge(Edge *correspondingEdge);

    void setCost(int cost);

    void print() const;

    void initializeCost();

    int getLength() const;

    void setLength(int length);

  private:
    Vertex *orig;
    Vertex *dest; // destination vertex

    // auxiliary fields
    bool selected = true;
    Edge *reverse = nullptr;

    int length;

};

#endif //TRAVELLINGSALESMAN_EDGE_H
