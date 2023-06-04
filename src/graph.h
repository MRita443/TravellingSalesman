#ifndef TRAVELLINGSALESMAN_GRAPH_H
#define TRAVELLINGSALESMAN_GRAPH_H

#include <vector>
#include <memory>
#include <list>
#include "UFDS.h"
#include "vertex.h"
#include "coordinates.h"

class Graph {
  protected:
    struct tour_t {
        double distance;
        std::list<std::shared_ptr<Vertex>> course;
    };

    tour_t tour = {0, {}};
    unsigned int totalEdges = 0;
    std::vector<std::shared_ptr<Vertex>> vertexSet;    // vertex set
    std::vector<std::vector<bool>> selectedEdges;
    std::vector<std::vector<double>> distanceMatrix;

  public:
    Graph();

    [[nodiscard]] double findEdge(const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2) const;

    [[nodiscard]] unsigned int getNumVertex() const;

    [[nodiscard]] std::vector<std::shared_ptr<Vertex>> getVertexSet() const;

    [[nodiscard]] unsigned int getTotalEdges() const;

    [[nodiscard]] std::shared_ptr<Vertex> findVertex(const unsigned int &id) const;

    std::shared_ptr<Vertex> addVertex(const unsigned int &id, Coordinates c = {0, 0});

    void addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length);

    void visitedDFS(const std::shared_ptr<Vertex> &source);

    int addToTour(const std::shared_ptr<Vertex>& stop);

    int preorderMSTTraversal(const std::shared_ptr<Vertex>& source);

    void prim();

    void triangularTSPTour();

    void printTour();

    std::pair<double, unsigned int*> tspBT();

    void tspRecursion(unsigned int *currentSolution, double currentSolutionDist, unsigned int currentNodeIdx,
                      double &bestSolutionDist, unsigned int *bestSolution, unsigned int n);

    static bool inSolution(unsigned int j, const unsigned int *solution, unsigned int n);

    [[nodiscard]] std::pair<std::vector<unsigned int>, double>
    getInsertionEdges(std::vector<unsigned int> tour, unsigned int newVertexId) const;


    double nearestInsertionLoop(unsigned int &start);

    void clearGraph();

    std::pair<unsigned int, unsigned int> getNextHeuristicEdge(std::vector<unsigned int> tour, UFDS tourSets);

    [[nodiscard]] double getTourDistance() const;
};

#endif //TRAVELLINGSALESMAN_GRAPH_H
