#include "graph.h"
#include <climits>

Graph::Graph() = default;

unsigned int Graph::getNumVertex() const {
    return (unsigned int) vertexSet.size();
}

VertexPointerTable Graph::getVertexSet() const {
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
    auto it = vertexSet.find(std::make_shared<Vertex>(id));
    return it != vertexSet.end() ? *it : nullptr;
}

/**
 * Adds a vertex with a given id to the Graph, representing a given NODE
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param id - Id of the Vertex to add
 * @return True if successful, and false if a vertex with the given id already exists
 */
bool Graph::addVertex(const unsigned int &id) {
    return vertexSet.insert(std::make_shared<Vertex>(id)).second;
}

/**
 * Adds a bidirectional edge to the Graph between the vertices with id source and dest, and a given length
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param source - Id of the source Vertex
 * @param dest - Id of the destination Vertex
 * @param length - Length of the Edge to be added
 */
bool
Graph::addBidirectionalEdge(const unsigned int &source, const unsigned int &dest, double length) {
    auto v1 = findVertex(source);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;

    auto e1 = v1->addEdge(v2, length);
    auto e2 = v2->addEdge(v1, length);

    e1->setReverse(e2);
    e2->setReverse(e1);

    totalEdges++;
    return true;
}


/**
 * @brief Takes an Edge pointer and sets the selected state of that edge and its reverse to the given value
 * 
 * @param edge - Pointer to the edge to be set
 * @param selected - Value to set the selected state to
 */
void Graph::setSelectedEdge(const std::shared_ptr<Edge>& edge, bool selected) {
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
 * DFS traversal variation that sets the visited attribute to true of the nodes the DFS traverses to
 * Time Complexity: O(|V|+|E|)
 * @param source - Vertex where the DFS starts
*/
void Graph::visitedDFS(const std::shared_ptr<Vertex> &source) {
    source->setVisited(true);
    for (const std::shared_ptr<Edge> &e: source->getAdj()) {
        if (!e->getDest()->isVisited()) {
            visitedDFS(e->getDest());
        }
    }
}


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
void Graph::dfsKruskalPath(const std::shared_ptr<Vertex> &source) {
    source->setVisited(true);
    for (const std::shared_ptr<Edge>& e: source->getAdj()) {
        if (!e->getDest()->isVisited() && e->isSelected()) {
            e->getDest()->setPath(e);
            dfsKruskalPath(e->getDest());
        }
    }
}


bool inSolution(unsigned int j, const unsigned int *solution, unsigned int n) {
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