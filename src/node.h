#ifndef TRAVELLINGSALESMAN_NODE_H
#define TRAVELLINGSALESMAN_NODE_H

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <utility>
#include "constants.h"

class Node {
  private:
    unsigned int id;
    double latitude{};
    double longitude{};
    std::string name;
    std::unordered_map<unsigned int, double> distToNodes;

  public:
    Node();

    explicit Node(unsigned int id, double latitude = constants::INF, double longitude = constants::INF,
                  std::string name = "");

    const unsigned int &getId() const;

    const double &getLatitude() const;

    void setLatitude(const double &latitude);

    const double &getLongitude() const;

    void setLongitude(const double &longitude);

    [[nodiscard]] const std::string &getName() const;

    void setName(const std::string &name);

    const std::unordered_map<unsigned int, double> &getDistToNodes() const;

    void setDistToNodes(const std::unordered_map<unsigned int, double> &distToNodes);

    Node &operator=(const Node &Node);

    const double &getDistToNode(unsigned int id) const;

    bool addDistToNodeEntry(unsigned int id, double dist);

};


struct NodeHash {
    std::size_t operator()(const Node &Node) const {
        return std::hash<unsigned int>()(Node.getId());
    }
};

struct NodeEquals {
    bool operator()(const Node &Node1, const Node &Node2) const {
        return Node1.getId() == Node2.getId();
    }
};


struct NodePointerHash {
    std::size_t operator()(const std::shared_ptr<Node> Node) const {
        return std::hash<unsigned int>()(Node->getId());
    }
};

struct NodePointerEquals {
    bool operator()(const std::shared_ptr<Node> Node1, const std::shared_ptr<Node> Node2) const {
        return Node1->getId() == Node2->getId();
    }
};

typedef std::unordered_set<Node, NodeHash, NodeEquals> nodeTable;

typedef std::unordered_set<std::shared_ptr<Node>, NodePointerHash, NodePointerEquals> nodePointerTable;

template<typename T>
using nodeMap = std::unordered_map<Node, T, NodeHash, NodeEquals>;


#endif //TRAVELLINGSALESMAN_NODE_H
