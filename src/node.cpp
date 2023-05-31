//
// Created by rita on 28-02-2023.
//

#include "node.h"

Node::Node() = default;


Node::Node(unsigned int id, double latitude, double longitude, std::string name) : id(id),
                                                                                   latitude(latitude),
                                                                                   longitude(longitude),
                                                                                   name(std::move(
                                                                                           name)) { distToNodes[id] = 0; }
//Getters

const std::string &Node::getName() const {
    return name;
}

void Node::setName(const std::string &name) {
    Node::name = name;
}

Node &Node::operator=(const Node &node) = default;

const unsigned int &Node::getId() const {
    return id;
}

const double &Node::getLatitude() const {
    return latitude;
}

void Node::setLatitude(const double &latitude) {
    Node::latitude = latitude;
}

const double &Node::getLongitude() const {
    return longitude;
}

void Node::setLongitude(const double &longitude) {
    Node::longitude = longitude;
}

const std::unordered_map<unsigned int, double> &Node::getDistToNodes() const {
    return distToNodes;
}

void Node::setDistToNodes(const std::unordered_map<unsigned int, double> &distToNodes) {
    Node::distToNodes = distToNodes;
}

const double &Node::getDistToNode(unsigned int id) const {
    auto it = distToNodes.find(id);
    if (it != distToNodes.end()) return it->second;
    return constants::INF;
}

bool Node::addDistToNodeEntry(unsigned int id, double dist) {
    return distToNodes.insert(std::make_pair(id, dist)).second;
}






