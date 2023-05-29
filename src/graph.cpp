#include "graph.h"
#include <climits>

Graph::Graph() = default;

unsigned int Graph::getNumVertex() const {
    return (unsigned int) vertexSet.size();
}

std::vector<Vertex *> Graph::getVertexSet() const {
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
Vertex *Graph::findVertex(const std::string &id) const {
    auto it = idToVertex.find(id);
    if (it == idToVertex.end()) { return nullptr; }
    return it->second;
}

/**
 * Adds a vertex with a given id to the Graph, representing a given station
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param id - Id of the Vertex to add
 * @return True if successful, and false if a vertex with the given id already exists
 */
bool Graph::addVertex(const std::string &id) {
    if (findVertex(id) != nullptr)
        return false;
    vertexSet.push_back(new Vertex(id));
    idToVertex[id] = vertexSet.back();
    return true;
}

/**
 * Adds and returns a bidirectional edge to the Graph between the vertices with id source and dest, with a capacity of c, representing a Service s
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param source - Id of the source Vertex
 * @param dest - Id of the destination Vertex
 * @param dist - Distance of the Edge to be added
 * @return Pair containing a pointer to the created Edge and to its reverse
 */
std::pair<Edge *, Edge *>
Graph::addAndGetBidirectionalEdge(const std::string &source, const std::string &dest, unsigned int dist) {
    auto v1 = findVertex(source);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return {nullptr, nullptr};

    auto e1 = v1->addEdge(v2, dist);
    auto e2 = v2->addEdge(v1, dist);
    e1->setReverse(e2);
    e2->setReverse(e1);

    totalEdges++;
    return {e1, e2};
}

/**
 * Takes a vector of edge pointers and sets the selected state of those edges and their reverses to false
 * Time Complexity: O(size(edges))
 * @param edges - Vector of edge pointers to be deactivated
 */
void Graph::deactivateEdges(const std::vector<Edge *> &edges) {
    for (Edge *edge: edges) {
        edge->setSelected(false);
        edge->getReverse()->setSelected(false);
    }
}

/**
 * Takes a vector of edge pointers and sets the selected state of those edges and their reverses to true
 * Time Complexity: O(size(edges))
 * @param edges - Vector of edge pointers to be activated
 */
void Graph::activateEdges(const std::vector<Edge *> &edges) {
    for (Edge *edge: edges) {
        edge->setSelected(true);
        edge->getReverse()->setSelected(true);
    }
}

/**
 * DFS traversal variation that sets the visited attribute to true of the nodes the DFS traverses to
 * Time Complexity: O(|V|+|E|)
 * @param source - Vertex where the DFS starts
 */
void Graph::visitedDFS(Vertex *source) {
    source->setVisited(true);
    for (Edge const *e: source->getAdj()) {
        if (!e->getDest()->isVisited()) {
            visitedDFS(e->getDest());
        }
    }
}

bool inSolution(unsigned int j, unsigned int *solution, unsigned int n) {
    for (int i = 0; i < n; i++) {
        if (solution[i] == j) {
            return true;
        }
    }
    return false;
}

void
tspRecursion(unsigned int *currentSolution, unsigned int currentSolutionDist,
             unsigned int currentNodeIdx,
             unsigned int &bestSolutionDist, unsigned int *bestSolution, unsigned int n, const unsigned int **dists) {
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

unsigned int tspBT(const unsigned int **dists, unsigned int n, unsigned int path[]) {
    unsigned int currentSolution[n];
    currentSolution[0] = 0;
    unsigned int bestSolutionDist = UINT_MAX;
    tspRecursion(currentSolution, 0, 1, bestSolutionDist, path, n, dists);
    return bestSolutionDist;
}