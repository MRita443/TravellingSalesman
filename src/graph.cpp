#include "graph.h"
#include <climits>

Graph::Graph() = default;

unsigned int Graph::getNumVertex() const {
    return (unsigned int) vertexSet.size();
}

std::vector<std::shared_ptr<Vertex>> Graph::getVertexSet() const {
    return vertexSet;
}

unsigned int Graph::getTotalEdges() const {
    return totalEdges;
}

/**
 * Finds the vertex with a given id
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param id - Id of the vertex to be found
 * @return Pointer to the found Vertex, or nullptr if none was found
 */
std::shared_ptr<Vertex> Graph::findVertex(const unsigned int &id) const {
    return vertexSet.size() <= id ? nullptr : vertexSet[id];
}

/**
 * Adds a vertex with a given id to the Graph, representing a given Node
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param id - Id of the Vertex to add
 * @return True if successful, and false if a vertex with the given id already exists
 */
std::shared_ptr<Vertex> Graph::addVertex(const unsigned int &id, Coordinates c) {
    std::shared_ptr<Vertex> newVertex = nullptr;
    if (vertexSet.size() <= id) {
        newVertex = std::make_shared<Vertex>(id, c);
        vertexSet.push_back(newVertex);
    }
    return newVertex;
}

/**
 * Adds a bidirectional edge to the Graph between the vertices with id source and dest, and a given length
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param source - Id of the source Vertex
 * @param dest - Id of the destination Vertex
 * @param length - Length of the Edge to be added
 */
void
Graph::addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length) {
    if (distanceMatrix.size() <= source) distanceMatrix.resize(source + 1, {constants::INF});
    if (distanceMatrix[source].size() <= dest) distanceMatrix[source].resize(dest + 1, constants::INF);
    distanceMatrix[source][dest] = length;

    if (distanceMatrix.size() <= dest) distanceMatrix.resize(dest + 1, {constants::INF});
    if (distanceMatrix[dest].size() <= source) distanceMatrix[dest].resize(source + 1, constants::INF);
    distanceMatrix[dest][source] = length;
    totalEdges++;
}


/**
 * @brief Takes an Edge pointer and sets the selected state of that edge and its reverse to the given value
 * 
 * @param edge - Pointer to the edge to be set
 * @param selected - Value to set the selected state to
 */
void Graph::setSelectedEdge(const std::shared_ptr<Edge> &edge, bool selected) {
    edge->setSelected(selected);
    edge->getReverse()->setSelected(selected);
}

/**
 * Takes a vector of Edge pointers and sets the selected state of those edges and their reverses to false
 * Time Complexity: O(size(edges))
 * @param edges - Vector of Edge pointers to be deactivated
 */
void Graph::deactivateEdges(const std::vector<std::shared_ptr<Edge>> &edges) {
    for (const std::shared_ptr<Edge> &edge: edges) {
        setSelectedEdge(edge, false);
    }
}

/**
 * Takes a vector of Edge pointers and sets the selected state of those edges and their reverses to true
 * Time Complexity: O(size(edges))
 * @param edges - Vector of Edge pointers to be activated
 */
void Graph::activateEdges(const std::vector<std::shared_ptr<Edge>> &edges) {
    for (const std::shared_ptr<Edge> &edge: edges) {
        setSelectedEdge(edge, true);
    }
}

/**
 * DFS traversal variation that sets the visited attribute to true of the vertices the DFS traverses to
 * Time Complexity: O(|V|+|E|)
 * @param source - Vertex where the DFS starts
*/
/*void Graph::visitedDFS(const std::shared_ptr<Vertex> &source) {
    source->setVisited(true);
    for (const std::shared_ptr<Edge> &e: source->getAdj()) {
        if (!e->getDest()->isVisited()) {
            visitedDFS(e->getDest());
        }
    }
}*/


/**
 * @brief Builds a MST using Kruskal's algorithm
 * Time Complexity: O(|E|log|E|)
 */
/*void Graph::kruskal() {
    std::list<std::shared_ptr<Edge>> edges;
    for (const std::shared_ptr<Vertex>& v: vertexSet) {
        //v->setVisited(false); //TODO: Check if this is necessary
        for (const std::shared_ptr<Edge>& e: v->getAdj()) {
            edges.push_back(e);
            e->setSelected(false);
        }
    }
    UFDS ufds(vertexSet.size());

    edges.sort([](const std::shared_ptr<Edge> &e1, const std::shared_ptr<Edge> &e2) {
        return e1->getLength() < e2->getLength();
    });

    for (const std::shared_ptr<Edge> &e: edges) {
        if (!isSameSet(e->getOrig()->getId(), e->getDest()->getId())) {
            e->setSelected(true);
            linkSets(e->getOrig);
        }
    }
}*/

/**
 * @brief Iterates through the vertex set using DFS, respecting if an edge is selected or not
 * Registers the path taken in the vertex's path attribute
 * Time Complexity: O(|V|+|E|)
 * @param source - Vertex where the DFS starts
 */
/*void Graph::dfsKruskalPath(const std::shared_ptr<Vertex> &source) {
    source->setVisited(true);
    for (const std::shared_ptr<Edge> &e: source->getAdj()) {
        if (!e->getDest()->isVisited() && e->isSelected()) {
            e->getDest()->setPath(e);
            dfsKruskalPath(e->getDest());
        }
    }
}*/


bool Graph::inSolution(unsigned int j, const unsigned int *solution, unsigned int n) {
    for (int i = 0; i < n; i++) {
        if (solution[i] == j) {
            return true;
        }
    }
    return false;
}

void
Graph::tspRecursion(unsigned int *currentSolution, unsigned int currentSolutionDist,
                    unsigned int currentNodeIdx,
                    unsigned int &bestSolutionDist, unsigned int *bestSolution, unsigned int n,
                    const unsigned int **dists) {
    if (currentNodeIdx == n) {
        //Could need to verify here if last node connects to first

        //Add dist from last node back to zero and check if it's an improvement
        if (currentSolutionDist + dists[currentSolution[currentNodeIdx - 1]][0] < bestSolutionDist) {
            bestSolutionDist = currentSolutionDist + dists[currentSolution[currentNodeIdx - 1]][0];
            for (int i = 0; i < n; i++) {
                bestSolution[i] = currentSolution[i];
            }
        }
        return;

    }
    //Check if node is already in path
    for (int i = 1; i < n; i++) {
        if (dists[currentSolution[currentNodeIdx - 1]][i] + currentSolutionDist < bestSolutionDist) {
            if (!inSolution(i, currentSolution, currentNodeIdx)) {
                currentSolution[currentNodeIdx] = i;
                tspRecursion(currentSolution, dists[currentSolution[currentNodeIdx - 1]][i] + currentSolutionDist,
                             currentNodeIdx + 1, bestSolutionDist, bestSolution, n, dists);
            }
        }
    }
}

/**
 * Finds the edge connecting two vertices
 * Time Complexity: O(|E|) (average case) | O(|V|*|E|) (worst case)
 * @param v1id - Id of the first vertex
 * @param v2id - Id of the second vertex
 * @return Pointer to the edge that connects both vertices
 */
/*std::shared_ptr<Edge> Graph::findEdge(const unsigned int &v1id, const unsigned int &v2id) const {
    for (auto e: findVertex(v1id)->getAdj()) {
        if (e->getDest()->getId() == v2id) return e;
    }
    return nullptr;
}*/


unsigned int Graph::tspBT(const unsigned int **dists, unsigned int n, unsigned int path[]) {
    unsigned int currentSolution[n];
    currentSolution[0] = 0;
    unsigned int bestSolutionDist = UINT_MAX;
    tspRecursion(currentSolution, 0, 1, bestSolutionDist, path, n, dists);
    return bestSolutionDist;
}

double Graph::nearestInsertionLoop(unsigned int &start) {
    double distance = 0;
    std::vector<unsigned int> tour;

    //Get shortest adjacent edge
    std::vector<double> adjacent = distanceMatrix[start];
    auto it = std::min_element(adjacent.begin(), adjacent.end());
    unsigned int minEdgeIndex = std::distance(adjacent.begin(), it);

    //Initialize the partial tour with the chosen vertex and its closest neighbour
    tour.push_back(start);
    tour.push_back(minEdgeIndex);
    // TODO(?): Set path of adjacent edge to be the start vertex
    distance += distanceMatrix[start][minEdgeIndex];

    //Two cities are already in the tour, repeat for the leftover cities
    for (int i = 2; i < vertexSet.size(); i++) {
        std::pair<unsigned int, unsigned int> nextEdge = getNextHeuristicEdge(tour);
        unsigned int newVertexId = nextEdge.second;

        auto insertionEdges = getInsertionEdges(tour, newVertexId);

        unsigned int closingVertex = insertionEdges.first.back();
        auto closingVertexIt = std::find(tour.begin(), tour.end(), closingVertex);
        tour.insert(closingVertexIt, newVertexId); //Insert new vertex in between the two old vertices

        //Remove the length of the edge that was replaced, and add the length of the two new edges
        distance =
                distance - distanceMatrix[insertionEdges.first[0]][insertionEdges.first.back()] + insertionEdges.second;
    }
    //The two untied edges will always be the starting two vertices
    return distance + distanceMatrix[minEdgeIndex][start];
}

std::pair<unsigned int, unsigned int> Graph::getNextHeuristicEdge(std::vector<unsigned int> tour) {
    double smallestLength = constants::INF;
    std::pair<unsigned int, unsigned int> edgeExtremities;

    for (auto id: tour) {
        for (int i = 0; i < distanceMatrix[id].size(); i++) {
            //If it's an edge to a vertex not yet in the tour
            if (std::find(tour.begin(), tour.end(), i) == tour.end()) {
                if (distanceMatrix[id][i] < smallestLength) {
                    smallestLength = distanceMatrix[id][i];
                    edgeExtremities = {id, i};
                }
            }
        }
    }
    return edgeExtremities;
}


std::pair<std::vector<unsigned int>, double>
Graph::getInsertionEdges(std::vector<unsigned int> tour, const unsigned int newVertexId) const {
    std::pair<std::vector<unsigned int>, double> result = {{}, constants::INF};

    for (int i = 0; i < tour.size() - 1; i++) {
        //If there are two edges that could replace the current one, connecting its ends to the new vertex
        double currentDistance = distanceMatrix[tour[i]][newVertexId] + distanceMatrix[newVertexId][tour[i + 1]];
        if (currentDistance < result.second) {
            result.second = currentDistance;
            result.first = {tour[i], newVertexId, tour[i + 1]};
        }
    }
    return result;
}

void Graph::clearGraph() {
    distanceMatrix = {};
    vertexSet = {};
    totalEdges = 0;
}



