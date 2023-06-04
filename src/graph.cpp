#include "graph.h"

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
 * Time Complexity: O(1)
 * @param id - Id of the vertex to be found
 * @return Pointer to the found Vertex, or nullptr if none was found
 */
std::shared_ptr<Vertex> Graph::findVertex(const unsigned int &id) const {
    return vertexSet.size() <= id ? nullptr : vertexSet[id];
}

/**
 * Finds length of the edge connecting two vertices (if it doesn't explicitly exist, it returns the haversine distance)
 * Time Complexity: O(1)
 * @param v1id - Pointer to the first vertex
 * @param v2id - Pointer to the second vertex
 */
double Graph::findEdge(const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2) const{
    unsigned int v1id = v1->getId();
    unsigned int v2id = v2->getId();

    if (v1id == v2id) return -2;
    if (distanceMatrix[v1id][v2id] != constants::INF) return distanceMatrix[v1id][v2id];
    else { //haversine function
        return v1->haversineDistance(v2);
    }
}

/**
 * Adds a vertex with a given id to the Graph, representing a given Node
 * Time Complexity: O(1) (average case) | O(|V|) (worst case)
 * @param id - Id of the Vertex to add
 * @return True if successful, and false if a vertex with the given id already exists
 */
std::shared_ptr<Vertex> Graph::addVertex(const unsigned int &id, Coordinates c) {
    std::shared_ptr<Vertex> newVertex = nullptr;
    if (vertexSet.size() <= id) { vertexSet.resize(id + 1); }
    newVertex = std::make_shared<Vertex>(id, c);
    vertexSet[id] = newVertex;

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
 * DFS traversal variation that sets the visited attribute to true of the vertices the DFS traverses to
 * Time Complexity: O(|V|^2)
 * @param source - Vertex where the DFS starts
*/
void Graph::visitedDFS(const std::shared_ptr<Vertex> &source) {
    source->setVisited(true);
    for (size_t i = 0; i<vertexSet.size(); i++){
        if (distanceMatrix[source->getId()][i] != constants::INF && source->getId() != i) { //edge existe e não é para si mesma
            std::shared_ptr<Vertex> v = findVertex(i);
            if (!v->isVisited()) {
                visitedDFS(v);
            }
        }
    }
}


/**
 * @brief Builds a MST using Prim's algorithm
 * Time Complexity: O(|V|^2)
 */
void Graph::prim(){
    MutablePriorityQueue<Vertex> q;
    std::vector<std::vector<bool>> newMatrix(vertexSet.size(), std::vector<bool>(vertexSet.size(), false));
    this->selectedEdges = newMatrix;

    std::shared_ptr<Vertex> start = findVertex(0);

    for (std::shared_ptr<Vertex> &v : vertexSet) {
        v->setDist(constants::INF);
        v->setVisited(false);
        v->setPath(nullptr);
    }
    start->setDist(0);
    q.insert(start);

    while (!q.empty()){
        //extrai no mais perto da mst, marca-o como visitado e guarda a edge
        std::shared_ptr<Vertex> currentVertex = q.extractMin();
        if (currentVertex->getPath() != nullptr) {
            selectedEdges[currentVertex->getId()][currentVertex->getPath()->getId()] = true;
            selectedEdges[currentVertex->getPath()->getId()][currentVertex->getId()] = true;
        }
        currentVertex->setVisited(true);

        //procura vizinho por visitar
        for (size_t i = 0; i<vertexSet.size(); i++){
            if (distanceMatrix[currentVertex->getId()][i] == constants::INF || i == currentVertex->getId()) continue;
            std::shared_ptr<Vertex> dest = findVertex(i);
            if (!dest->isVisited()){
                //atualiza dados
                double oldDist = dest->getDist();
                if (distanceMatrix[currentVertex->getId()][i] < oldDist){
                    dest->setPath(currentVertex);
                    dest->setDist(distanceMatrix[currentVertex->getId()][i]);
                    oldDist == constants::INF ? q.insert(dest) : q.decreaseKey(dest);
                }
            }
        }
    }
}

 /**
  * Adds a vertex to the tour structure and updates the total distance
  * Time Complexity: O(1)
  * @param stop - Vertex to add
  * @return execution errors (0 if none, -1 if couldn't calculate Edge length, -2 if self-loop)
  */
int Graph::addToTour(std::shared_ptr<Vertex> stop) {
    if (!tour.course.empty()){
        if ((*(tour.course).rbegin())->getId() == stop->getId()){
            //selfloop
            return -2;
        }
        double aresta = findEdge((*(tour.course).rbegin()), stop);
        if (aresta == -1){
            //rejeitar tour
            return aresta;
        }
        tour.distance += aresta;
    }
    tour.course.push_back(stop);
    return 0;
}

/**
 * @brief Iterates through the vertex set using DFS, respecting if an edge is selected or not
 * Time Complexity: O(|V|^2)
 * @param source - Vertex where the DFS starts
 * @return execution errors (0 if none, -1 if couldn't calculate Edge length, -2 if self-loop)
 */
int Graph::preorderMSTTraversal(std::shared_ptr<Vertex> source){
    source->setVisited(true);
    int exec_val = addToTour(source);
    if (exec_val != 0) return exec_val;

    for (size_t i = 0; i<vertexSet.size(); i++){
        std::shared_ptr<Vertex> dest = findVertex(i);

        if(!(dest->isVisited()) && selectedEdges[source->getId()][i]){
            preorderMSTTraversal(dest);
        }
    }

    if (source->getId() == 0) return addToTour(source);
    return 0;
}

/**
 * Calculates an approximation of the TSP, using the triangular approximation heuristic
 * Time Complexity: O(|V|^2)
 */
void Graph::triangularTSPTour(){
    /*
    -Build MST
    -Get pre-order of the mst as vector
    -Iterate that order getting total dist
    */
    tour = {0,{}};
    prim();

    for (std::shared_ptr<Vertex> &v : vertexSet) v->setVisited(false);

    int exec_val = preorderMSTTraversal(findVertex(0));
    switch (exec_val){
        case 0:
            printTour();
            break;
        case -1:
            printf("Couldn't calculate approximation of TSP for this graph!\n");
            break;
        case -2:
            printf("Course would contain self-loop!\n");
            break;
    }

    return; //results are updated in tour
}

/**
 * Displays Tour's Vertices by order and its total distance
 * Time Complexity: O(|V|)
 */
void Graph::printTour(){
    printf("Tour's total Distance: %f\n", tour.distance);
    printf("Path taken: ");
    for (std::shared_ptr<Vertex> v : tour.course){
        printf(" %d", v->getId());
    }
    printf("\n");
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



