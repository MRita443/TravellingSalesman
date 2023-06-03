
#include "menu.h"

int main() {
    std::string const nodesFilePath = "../dataset/Real-world-Graphs/graph1/nodes.csv";
    std::string const edgesFilePath = "../dataset/Real-world-Graphs/graph1/edges.csv";
    //std::string const edgesFilePath = "../dataset/Extra_Fully_Connected_Graphs/edges_900.csv";

    Menu menu;
    menu.extractFileInfo( edgesFilePath
                          ,nodesFilePath
                          );
    return 0;
}