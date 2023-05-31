#ifndef TRAVELLINGSALESMAN_DATAREPOSITORY_H
#define TRAVELLINGSALESMAN_DATAREPOSITORY_H

#include <algorithm>
#include <iostream>
#include <optional>
#include <memory>
#include "node.h"

class DataRepository {

  private:
    nodePointerTable nodes;

  public:
    DataRepository();

    const nodePointerTable &getNodes() const;

    void setNodes(const nodePointerTable &nodes);

    std::shared_ptr<Node> findNode(const unsigned int &id);

    Node &addNodeEntry(unsigned int id, double latitude = constants::INF, double longitude = constants::INF,
                       const std::string &name = "");

};


#endif //TRAVELLINGSALESMAN_DATAREPOSITORY_H
