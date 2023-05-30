#include "dataRepository.h"

using namespace std;

DataRepository::DataRepository() = default;

const nodePointerTable &DataRepository::getNodes() const {
    return nodes;
}

void DataRepository::setNodes(const nodePointerTable &nodes) {
    DataRepository::nodes = nodes;
}

/**
 * Adds a new entry to the unordered_set of Nodes, creating the corresponding Node object
 * Time Complexity: O(1) (average case) | O(size(stations)) (worst case)
 * @param id - Id of the Node to be created
 * @param latitude - Latitude of the Node to created
 * @param longitude - Longitude of the Node to be created
 * @param name - Name of the Node to be created
 * @return New Node object created
 */
Node &
DataRepository::addNodeEntry(unsigned int id, double latitude, double longitude, const std::string &name) {
    std::shared_ptr<Node> newNode(new Node(id, latitude, longitude, name));
    nodes.insert(newNode);
    sumLatitude += latitude;
    sumLongitude += longitude;
    return *newNode;
}

/**
 * Finds the Node object with the given id
 * Time Complexity: O(1) (average case) | O(size(nodes)) (worst case)
 * @param id - Id of the Node to be returned
 * @return Pointer to the Node found, or nullptr if none was found
 */
shared_ptr<Node> DataRepository::findNode(const unsigned int &id) {
    auto it = nodes.find(make_shared<Node>(id));
    return it != nodes.end() ? *it : nullptr;
}

long DataRepository::getAverageLatitude() {
    return nodes.empty() ? 0 : (long) sumLatitude / (long) nodes.size();
}

long DataRepository::getAverageLongitude() {
    return nodes.empty() ? 0 : (long) sumLongitude / (long) nodes.size();
}
