#ifndef TRAVELLINGSALESMAN_MENU_H
#define TRAVELLINGSALESMAN_MENU_H

#include <iostream>
#include <string>
#include <iomanip>
#include <limits>
#include <fstream>
#include <sstream>
#include <cmath>
#include <unordered_set>
#include <chrono>
#include "graph.h"
#include "dataRepository.h"

class Menu {
  private:
    DataRepository dataRepository;
    Graph graph;
    unsigned static const COLUMN_WIDTH;
    unsigned static const COLUMNS_PER_LINE;

  public:
    Menu();

    void extractNodesFile(const std::string &filename);

    void extractEdgesFile(const std::string &filename, bool hasDescriptors = true, bool hasLabels = false);

    void extractFileInfo(const std::string &edgesFilename, const std::string &nodesFilename = "");

    void initializeMenu();

    unsigned int serviceMetricsMenu();

    unsigned int costOptMenu();

    unsigned int failuresMenu();

    void mainMenu();

    std::vector<Edge *> edgeFailureMenu();

    static bool checkInput(unsigned int checkLength = 0);

    static void nodeDoesntExist();

};


#endif //TRAVELLINGSALESMAN_MENU_H
