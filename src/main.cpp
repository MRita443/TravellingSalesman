#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph1/vertices.csv";
    std::string const edgesFilePath = "../dataset/Real-world-Graphs/graph1/edges.csv";

    Menu menu;
    menu.extractFileInfo(edgesFilePath);
    return 0;
}