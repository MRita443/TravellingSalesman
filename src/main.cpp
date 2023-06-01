
#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Toy-Graphs/graph1/nodes.csv";
    std::string const edgesFilePath = "../dataset/Extra_Fully_Connected_Graphs/edges_800.csv";

    Menu menu;
    menu.extractFileInfo(edgesFilePath);
    return 0;
}