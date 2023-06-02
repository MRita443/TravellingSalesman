#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph3/nodes.csv";
    std::string const edgesFilePath = "../dataset/Real-world-Graphs/graph3/edges.csv";

    Menu menu;
    menu.heuristicMenu();
    return 0;
}