#include "graph.h"

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
 * Finds the edge connecting two vertices
 * Time Complexity: O(|E|) (average case) | O(|V|*|E|) (worst case)
 * @param v1id - Id of the first vertex
 * @param v2id - Id of the second vertex
 * @return Pointer to the edge that connects both vertices
 */
std::shared_ptr<Edge> Graph::findEdge(const unsigned int &v1id, const unsigned int &v2id) const{
    for (std::shared_ptr<Edge> &e : findVertex(v1id)->getAdj()){
        if (e->getDest()->getId() == v2id) return e;
    }
    return nullptr;
}

/**
 * Adds a vertex with a given id to the Graph, representing a given Node
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
 * Time Complexity: O(1)
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
void Graph::kruskal(){
    std::list<std::shared_ptr<Edge>> edges; //TODO: better generate this list (edgeSet like vertexSet?)
    for (const std::shared_ptr<Vertex>& v: vertexSet){
        v->setVisited(false); // util for DFS later on
        for (const std::shared_ptr<Edge> &e: v->getAdj()){
            edges.push_back(e);
            e->setSelected(false);
        }
    }
    UFDS ufds((unsigned int)vertexSet.size());

    edges.sort([](const std::shared_ptr<Edge> &e1, const std::shared_ptr<Edge> &e2) {
        return e1->getLength() < e2->getLength();
    });

    for (const std::shared_ptr<Edge> &e: edges) {
        if (!ufds.isSameSet(e->getOrig()->getId(), e->getDest()->getId())) {
            e->setSelected(true);
            ufds.linkSets(e->getOrig()->getId(), e->getDest()->getId());
        }
    }
}

/**
 * Checks if the last vertex on the tour attribute is adjacent to a given vertex
 * Time Complexity: O(|E|) (worst case)
 * @param candidate - Vertex that is going to be checked
 * @return true if the candidate vertex is adjacent to the tour's last vertex, false otherwise
 */
bool Graph::isConnectable(std::shared_ptr<Vertex> &candidate) const {
    for (std::shared_ptr<Edge> &e : (*tour.course.rbegin())->getAdj()){
        if (e->getDest() == candidate) return true;
    }
    return false;
}

 /**
  * Adds a vertex to the tour structure and updates the total distance
  * Time Complexity: O(|E|) (average case) | O(|V|*|E|) (worst case)
  * @param stop - Vertex to add
  */
void Graph::addToTour(std::shared_ptr<Vertex> &stop) {
    if (!tour.course.empty()){
        tour.distance += findEdge((*(tour.course).rbegin())->getId(), stop->getId())->getLength();
    }
    tour.course.push_back(stop);
}

/**
 * Adds the distance between the first and last nodes so the tour "closes"
 * Time Complexity: O(|E|) (average case) | O(|V|*|E|) (worst case)
 */
void Graph::tieDownTour(){
    tour.distance += findEdge((*tour.course.begin())->getId(), (*tour.course.rbegin())->getId())->getLength();
}

/**
 * @brief Iterates through the vertex set using DFS, respecting if an edge is selected or not
 * Time Complexity: O(|V|+|E|)
 * @param source - Vertex where the DFS starts
 */
void Graph::preorderMSTTraversal(std::shared_ptr<Vertex> source){
    source->setVisited(true);
    addToTour(source);
    for (std::shared_ptr<Edge> &e : source->getAdj()){
        bool unvisited = !(e->getDest()->isVisited());
        bool selected = e->isSelected();
        //bool connected_to_last = isConnectable(e->getDest());
        if(unvisited && selected /*&& connected_to_last*/){
            preorderMSTTraversal(e->getDest());
        }
    }
}

/**
 * Calculates an approximation of the TSP, using the triangular approximation heuristic
 * Time Complexity: //TODO
 */
void Graph::triangularTSPTour(){
    /*
    -Build MST
    -Get pre-order of the mst as vector
    -Iterate that order getting total dist
    */
    kruskal();
    tour = {0,{}};
    preorderMSTTraversal(*(vertexSet.begin()));
    tieDownTour();
    return; //results are updated in tour
}


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

unsigned int Graph::tspBT(const unsigned int **dists, unsigned int n, unsigned int path[]) {
    unsigned int currentSolution[n];
    currentSolution[0] = 0;
    unsigned int bestSolutionDist = UINT_MAX;
    tspRecursion(currentSolution, 0, 1, bestSolutionDist, path, n, dists);
    return bestSolutionDist;
}
