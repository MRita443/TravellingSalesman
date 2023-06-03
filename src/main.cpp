#include "menu.h"

int main() {
<<<<<<< HEAD
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph1/nodes.csv";
    std::string const edgesFilePath = "../dataset/Real-world-Graphs/graph3/edges.csv";
=======
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph1/vertices.csv";
    std::string const edgesFilePath = "../dataset/Toy-Graphs/tourism.csv";
>>>>>>> backtracking-commented

    Menu menu;
    menu.extractFileInfo(edgesFilePath);
    return 0;
}