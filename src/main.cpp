#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph1/vertices.csv";
    std::string const edgesFilePath = "../dataset/Extra_Fully_Connected_Graphs/edges_25.csv";

    Menu menu;
    menu.extractFileInfo(edgesFilePath);
    return 0;
}