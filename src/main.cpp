
#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Toy-Graphs/graph1/nodes.csv";
    std::string const edgesFilePath = "../dataset/Toy-Graphs/tourism.csv";

    Menu menu;
    menu.extractFileInfo(edgesFilePath);
    return 0;
}