//
// Created by rita on 17-03-2023.
//

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
 * Adds a new entry to the unordered_set of Stations, creating the corresponding Node object
 * Time Complexity: O(1) (average case) | O(size(stations)) (worst case)
 * @param name - Name of the station to be created
 * @param district - District of the Node to be created
 * @param municipality - Municipality of the Node to be created
 * @param township - Township of the Node to be created
 * @param line - Line of the station to be created
 * @return New Node object created
 */
Node &
DataRepository::addNodeEntry(unsigned int id, double latitude, double longitude, const std::string &name) {
    std::shared_ptr<Node> newNode(new Node(id, latitude, longitude, name));
    nodes.insert(newNode);
    return *newNode;
}


/**
 * Finds the Node object with the given name
 * Time Complexity: O(1) (average case) | O(size(stations)) (worst case)
 * @param name - Name of the Node to be returned
 * @return optional<Node> value which will contain the Node object, or be empty if no such Node was found
 */
shared_ptr<Node> DataRepository::findNode(const unsigned int &id) {
    auto it = nodes.find(make_shared<Node>(id));
    return it != nodes.end() ? *it : nullptr;
}
