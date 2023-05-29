//
// Created by rita on 28-02-2023.
//

#include "menu.h"
#include "node.h"

using namespace std;

unsigned const Menu::COLUMN_WIDTH = 50;
unsigned const Menu::COLUMNS_PER_LINE = 3;

Menu::Menu() = default;

/**
 * Delegates initialization of the menu, calling the appropriate functions for information extraction and output
 */
/*void Menu::initializeMenu() {
    extractFileInfo();
    mainMenu();
}*/

/**
 * Delegates extracting file info, calling the appropriate functions for each file
 * Time Complexity: O(n*v), where n is the number of lines of network.csv and v is the number of lines in stations.csv
 */
void Menu::extractFileInfo(const std::string &edgesFilename, const std::string &nodesFilename) {
    if (!nodesFilename.empty()) {
        extractNodesFile(nodesFilename);
    }
    if (edgesFilename == "../dataset/Toy-Graphs/tourism.csv") extractEdgesFile(edgesFilename, true, true);
    if (edgesFilename.find("Extra_Fully_Connected_Graphs") != std::string::npos) {
        extractEdgesFile(edgesFilename, false);
    } else extractEdgesFile(edgesFilename);
}

/**
 * Checks if the input given by the user is appropriate or not
 * Time Complexity: O(1)
 * @param checkLength - Integer indicating if the length of the input should be checked or not, and, if so, its valid max length
 * @return Returns true if the input is appropriate and false if it isn't
 */
bool Menu::checkInput(unsigned int checkLength) {

    /*
    checkLength = 0 Don't check length
    checkLength = 1 Check for length 1
    checkLength = 2 Check for max length 2
    ...
    */

    if (!cin) // User didn't input what expected
    {
        cin.clear(); // Reset failbit
        cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); //Skip bad input
        cout << "Please enter an appropriate input." << endl;
        return false;
    }

    if (checkLength > 0) {
        string line;
        getline(cin, line);
        if (line.length() >= checkLength) {
            cout << "Please enter an appropriate input." << endl;
            return false;
        }
    }
    return true;
}

/**
 * Outputs to the screen a message indicating that the given Node doesn't exist
 * Time Complexity: O(1)
 */
void Menu::nodeDoesntExist() {
    cout << "A node with this id doesn't exist!" << endl;
}

/**
 * Outputs main menu screen and calls other menu screens according to user input
 */
/*void Menu::mainMenu() {

    unsigned char commandIn = '\0';
    string line;

    while (commandIn != 'q') {
        if (commandIn == '\0') { //If program just started or returned from a different menu, print header

            //Header
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << setfill('-') << right << "RAILWAY NETWO";
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << left << "RK MANAGEMENT" << endl;

            cout << setw(COLUMN_WIDTH) << setfill(' ') << "Basic Service Metrics: [1]" << setw(COLUMN_WIDTH)
                 << "Operation Cost Optimization: [2]" << setw(COLUMN_WIDTH)
                 << "Reliability and Sensitivity to Line Failures: [3]" << endl;
            cout << setw(COLUMN_WIDTH) << "Quit: [q]" << endl;
        }
        cout << endl << "Press the appropriate key to the function you'd like to access: ";
        cin >> commandIn;
        if (!checkInput(1)) {
            commandIn = '\0';
            continue;
        }
        switch (commandIn) {
            case '1': {
                commandIn = serviceMetricsMenu();
                break;
            }
            case '2': {
                commandIn = costOptMenu();
                break;
            }
            case '3': {
                commandIn = failuresMenu();
                break;
            }
            case 'q': {
                cout << "Thank you for using our Railway Network Management System!";
                break;
            }
            default: {
                cout << "Please press one of listed keys." << endl;
                break;
            }
        }
    }
}*/

/**
 * Extracts and stores the information of a nodes file
 * Time Complexity: 0(n) (average case) | O(n²) (worst case), where n is the number of lines of stations.csv
 */
void Menu::extractEdgesFile(const std::string &filename, bool hasDescriptors, bool hasLabels) {
    {
        ifstream nodes(filename);

        string currentParam, currentLine;
        string originName, destinationName;
        unsigned int originId, destinationId;
        double distance;

        int counter = 0;

        getline(nodes, currentParam); //Ignore first line with just descriptors

        while (getline(nodes, currentLine)) {
            currentLine.erase(currentLine.end() - 1); //Remove \r
            istringstream iss(currentLine);
            while (getline(iss, currentParam, ',')) {
                switch (counter++) {
                    case 0: {
                        originId = stoul(currentParam);
                        break;
                    }
                    case 1: {
                        destinationId = stoul(currentParam);
                        break;
                    }
                    case 2: {
                        distance = stod(currentParam);
                        if (!hasLabels) counter = 0;
                        break;
                    }
                    case 3: {
                        originName = currentParam;
                        break;
                    }
                    case 4: {
                        destinationName = currentParam;
                        counter = 0;
                        break;
                    }
                }
                if (counter == 0) {
                    shared_ptr<Node> origin = dataRepository.findNode(originId);
                    shared_ptr<Node> destination = dataRepository.findNode(destinationId);

                    if (!origin) {
                        Node originNode;
                        if (!graph.addVertex(originId)) break;
                        if (hasLabels) originNode = dataRepository.addNodeEntry(originId, 0, 0, originName);
                        else originNode = dataRepository.addNodeEntry(originId);
                        originNode.addDistToNodeEntry(destinationId, distance);
                        if (!originNode.addDistToNodeEntry(destinationId, distance)) break;
                    } else if (!origin->addDistToNodeEntry(destinationId, distance)) break;

                    if (!destination) {
                        Node destinationNode;
                        if (!graph.addVertex(destinationId)) break;
                        if (hasLabels)
                            destinationNode = dataRepository.addNodeEntry(destinationId, 0, 0, destinationName);
                        else destinationNode = dataRepository.addNodeEntry(destinationId);
                        if (!destinationNode.addDistToNodeEntry(originId, distance)) break;
                    } else if (!destination->addDistToNodeEntry(originId, distance)) break;

                    if (!graph.addBidirectionalEdge(originId, destinationId, distance)) break;

                }
            }
        }
    }
}


/**
 * Extracts and stores the information of a nodes file
 * Time Complexity: 0(n) (average case) | O(n²) (worst case), where n is the number of lines of stations.csv
 */
void Menu::extractNodesFile(const std::string &filename) {
    {
        ifstream nodes(filename);

        string currentParam, currentLine;
        unsigned int id;
        double longitude, latitude;

        int counter = 0;

        getline(nodes, currentParam); //Ignore first line with just descriptors

        while (getline(nodes, currentLine)) {
            currentLine.erase(currentLine.end() - 1); //Remove \r
            istringstream iss(currentLine);
            while (getline(iss, currentParam, ',')) {
                switch (counter++) {
                    case 0: {
                        id = stoul(currentParam);
                        break;
                    }
                    case 1: {
                        longitude = stod(currentParam);
                        break;
                    }
                    case 2: {
                        latitude = stod(currentParam);
                        counter = 0;
                        break;
                    }
                }
                if (counter == 0) {
                    if (!graph.addVertex(id)) break;
                    dataRepository.addNodeEntry(id, latitude, longitude);
                }
            }
        }
    }
}



/**
 * Outputs basic service metrics menu screen and decides graph function calls according to user input
 * @return - Last inputted command, or '\0' for previous menu command
 */
/*unsigned int Menu::serviceMetricsMenu() {
    unsigned char commandIn = '\0';

    while (commandIn != 'q') {
        if (commandIn == '\0') {
            //Header
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << setfill('-') << right << "BASIC SERVI";
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << left << "CE METRICS" << endl;
            cout << setw(COLUMN_WIDTH) << setfill(' ') << "Two specific stations: [1]" << setw(COLUMN_WIDTH)
                 << "All valid pairs of stations: [2]" << setw(COLUMN_WIDTH) << "Reaching a specific station: [3]"
                 << endl;
            cout << setw(COLUMN_WIDTH) << setfill(' ') << "Top districts: [4]" << setw(COLUMN_WIDTH)
                 << "Top townships: [5]" << setw(COLUMN_WIDTH) << "Top municipalities: [6]" << endl;
            cout << setw(COLUMN_WIDTH) << "Back: [b]" << setw(COLUMN_WIDTH) << "Quit: [q]" << endl;
        }

        while (commandIn != 'q') {
            cout << endl << "Please select how to input the location whose max number of trains you'd like to check: ";
            cin >> commandIn;

            if (!checkInput(1)) {
                commandIn = '\0';
                continue;
            }
            switch (commandIn) {
                case '1': {
                    string departureName;
                    cout << "Enter the name of the departure station: ";
                    getline(cin, departureName);
                    if (!checkInput()) break;
                    optional<Station> departureStation = dataRepository.findStation(departureName);
                    if (!departureStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    string arrivalName;
                    cout << "Enter the name of the arrival station: ";
                    getline(cin, arrivalName);
                    if (!checkInput()) break;
                    optional<Station> arrivalStation = dataRepository.findStation(arrivalName);
                    if (!arrivalStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }
                    cout << graph.edmondsKarp({departureName}, arrivalName, residualGraph)
                         << " trains can simultaneously travel between "
                         << departureName
                         << " and " << arrivalName << "." << endl;
                    break;
                }
                case '2': {
                    pair<list<pair<string, string>>, unsigned int> result = graph.calculateNetworkMaxFlow(
                            residualGraph);
                    for (const pair<string, string> &p: result.first) {
                        cout << result.second << " trains can simultaneously travel between "
                             << p.first << " and " << p.second << "." << endl;
                    }
                    break;
                }
                case '3': {
                    string arrivalName;
                    cout << "Enter the name of the arrival station: ";
                    getline(cin, arrivalName);
                    if (!checkInput()) break;
                    optional<Station> arrivalStation = dataRepository.findStation(arrivalName);
                    if (!arrivalStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }
                    cout
                            << graph.incomingFlux(arrivalName, residualGraph) << " trains can simultaneously arrive at "
                            << arrivalName << "." << endl;
                    break;
                }
                case '4': {
                    unsigned int numDistricts;
                    cout << "Enter the number of districts you'd like to see: ";
                    cin >> numDistricts;
                    if (!checkInput()) break;
                    if (numDistricts > dataRepository.getDistrictToStations().size()) {
                        cout << "The network only has " << dataRepository.getDistrictToStations().size()
                             << " districts!" << endl;
                        break;
                    }
                    std::vector<std::pair<std::string, double>> result = graph.topGroupings(
                            dataRepository.getDistrictToStations(), residualGraph);

                    cout << endl << setw(COLUMN_WIDTH) << setfill(' ')
                         << "List of districts by average number of incoming trains capacity" << endl;

                    for (int i = 0; i < numDistricts; i++) {
                        stringstream value;
                        value << fixed << setprecision(2) << result[i].second;

                        if (result[i].first.empty()) result[i].first = "NO DISTRICT";
                        cout << setw(4) << to_string(i + 1) << setw(COLUMN_WIDTH / 2) << left
                             << " | " + value.str() + " trains" << result[i].first << endl;
                    }

                    break;
                }
                case '5': {
                    unsigned int numTownships;
                    cout << "Enter the number of townships you'd like to see: ";
                    cin >> numTownships;
                    if (!checkInput()) break;
                    if (numTownships > dataRepository.getTownshipToStations().size()) {
                        cout << "The network only has " << dataRepository.getTownshipToStations().size()
                             << " townships!" << endl;
                        break;
                    }
                    std::vector<std::pair<std::string, double>> result = graph.topGroupings(
                            dataRepository.getTownshipToStations(), residualGraph);

                    cout << endl << setw(COLUMN_WIDTH) << setfill(' ')
                         << "List of townships by average number of incoming trains capacity" << endl;

                    for (int i = 0; i < numTownships; i++) {
                        stringstream value;
                        value << fixed << setprecision(2) << result[i].second;

                        if (result[i].first.empty()) result[i].first = "NO TOWNSHIP";
                        cout << setw(4) << to_string(i + 1) << setw(COLUMN_WIDTH / 2) << left
                             << " | " + value.str() + " trains" << result[i].first << endl;
                    }

                    break;
                }
                case '6': {
                    unsigned int numMunicipalities;
                    cout << "Enter the number of municipalities you'd like to see: ";
                    cin >> numMunicipalities;
                    if (!checkInput()) break;
                    if (numMunicipalities > dataRepository.getMunicipalityToStations().size()) {
                        cout << "The network only has " << dataRepository.getMunicipalityToStations().size()
                             << " municipalities!" << endl;
                        break;
                    }
                    std::vector<std::pair<std::string, double>> result = graph.topGroupings(
                            dataRepository.getMunicipalityToStations(), residualGraph);

                    cout << endl << setw(COLUMN_WIDTH) << setfill(' ')
                         << "List of municipalities by average number of incoming trains capacity" << endl;

                    for (int i = 0; i < numMunicipalities; i++) {
                        stringstream value;
                        value << fixed << setprecision(2) << result[i].second;

                        if (result[i].first.empty()) result[i].first = "NO MUNICIPALITY";
                        cout << setw(4) << to_string(i + 1) << setw(COLUMN_WIDTH / 2) << left
                             << " | " + value.str() + " trains" << result[i].first << endl;
                    }

                    break;
                }
                case 'b': {
                    return '\0';
                }
                case 'q': {
                    cout << "Thank you for using our Railway Network Management System!" << endl;
                    break;
                }
                default:
                    cout << "Please press one of listed keys." << endl;
                    break;
            }
        }
    }
    return commandIn;
}*/


/**
 * Outputs cost optimization menu screen and decides graph function calls according to user input
 * @return - Last inputted command, or '\0' for previous menu command
 */
/*
unsigned int Menu::costOptMenu() {
    unsigned char commandIn = '\0';

    while (commandIn != 'q') {
        if (commandIn == '\0') {
            //Header
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << setfill('-') << right << "OPERATION COST";
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << left << " OPTIMIZATION" << endl;
            cout << setw(COLUMN_WIDTH) << setfill(' ') << "Two specific stations: [1]" << setw(COLUMN_WIDTH) << endl;
            cout << setw(COLUMN_WIDTH) << "Back: [b]" << setw(COLUMN_WIDTH) << "Quit: [q]" << endl;
        }

        while (commandIn != 'q') {
            cout << endl << "Please select how to input the location whose max number of trains you'd like to check: ";
            cin >> commandIn;

            if (!checkInput(1)) {
                commandIn = '\0';
                continue;
            }
            switch (commandIn) {
                case '1': {
                    string departureName;
                    cout << "Enter the name of the departure station: ";
                    getline(cin, departureName);
                    if (!checkInput()) break;
                    optional<Station> departureStation = dataRepository.findStation(departureName);
                    if (!departureStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    string arrivalName;
                    cout << "Enter the name of the arrival station: ";
                    getline(cin, arrivalName);
                    if (!checkInput()) break;
                    optional<Station> arrivalStation = dataRepository.findStation(arrivalName);
                    if (!arrivalStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }
                    pair<unsigned int, unsigned int> result = graph.minCostMaxFlow(departureName, arrivalName,
                                                                                   residualGraph);

                    cout << "Maintaining the network active at its maximum, " << result.first
                         << " trains can travel simultaneously between " << departureName << " and " << arrivalName
                         << ", at a minimum cost of " << result.second << "€." << endl;
                    break;
                }
                case 'b': {
                    return '\0';
                }
                case 'q': {
                    cout << "Thank you for using our Railway Network Management System!" << endl;
                    break;
                }
                default:
                    cout << "Please press one of listed keys." << endl;
                    break;
            }
        }
    }
    return commandIn;
}
*/

/**
 * Outputs cost optimization menu screen and decides graph function calls according to user input
 * @return - Last inputted command, or '\0' for previous menu command
 */
/*unsigned int Menu::failuresMenu() {
    unsigned char commandIn = '\0';

    while (commandIn != 'q') {
        if (commandIn == '\0') {
            //Header
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << setfill('-') << right << "LINE FA";
            cout << setw(COLUMN_WIDTH * COLUMNS_PER_LINE / 2) << left << "ILURES" << endl;
            cout << setw(COLUMN_WIDTH) << setfill(' ') << "Two specific stations: [1]" << setw(COLUMN_WIDTH)
                 << "Top affected stations: [2]" << endl;
            cout << setw(COLUMN_WIDTH) << "Back: [b]" << setw(COLUMN_WIDTH) << "Quit: [q]" << endl;
        }

        while (commandIn != 'q') {
            cout << endl
                 << "Please select how to input the location whose reduced connectivity max number of trains you'd like to check: ";
            cin >> commandIn;

            if (!checkInput(1)) {
                commandIn = '\0';
                continue;
            }
            switch (commandIn) {
                case '1': {
                    string departureName;
                    cout << "Enter the name of the departure station: ";
                    getline(cin, departureName);
                    if (!checkInput()) break;
                    optional<Station> departureStation = dataRepository.findStation(departureName);
                    if (!departureStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    string arrivalName;
                    cout << "Enter the name of the arrival station: ";
                    getline(cin, arrivalName);
                    if (!checkInput()) break;
                    optional<Station> arrivalStation = dataRepository.findStation(arrivalName);
                    if (!arrivalStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    vector<Edge *> deactivatedEdges = edgeFailureMenu();
                    if (deactivatedEdges.empty()) break;

                    pair<unsigned int, unsigned int> result =
                            graph.maxFlowDeactivatedEdges(deactivatedEdges, {departureName}, arrivalName,
                                                          residualGraph);
                    double reductionValue = result.first == 0 ? 0 : 100 - ((result.second * 1.0) / result.first) * 100;
                    cout << "The maximum number of trains travelling between "
                         << departureName
                         << " and " << arrivalName << " was altered from " << result.first << " to " << result.second
                         << ", in a " << fixed << setprecision(2) << reductionValue << "% reduction." << endl;
                    break;
                }
                case '2': {
                    unsigned int numStations;
                    cout << "Enter the number of stations you'd like to see: ";
                    cin >> numStations;
                    if (!checkInput()) break;
                    if (numStations > graph.getNumVertex()) {
                        cout << "The network only has " << graph.getNumVertex()
                             << " stations!" << endl;
                        break;
                    }

                    vector<Edge *> deactivatedEdges = edgeFailureMenu();
                    if (deactivatedEdges.empty()) break;

                    std::vector<std::pair<std::string, std::pair<unsigned int, unsigned int>>> result = graph.topReductions(
                            deactivatedEdges, residualGraph);

                    cout << setw(COLUMN_WIDTH) << setfill(' ')
                         << "List of stations by reduction number of incoming trains capacity" << endl << endl;


                    cout << setw(4) << "NUM" << setw(COLUMN_WIDTH / 2 + 10) << left << " | REDUCTION";
                    cout << setw(COLUMN_WIDTH / 2) << "REGULAR" << setw(COLUMN_WIDTH / 2) << left
                         << "REDUCED";
                    cout << "STATION" << endl;

                    for (int i = 0; i < numStations; i++) {
                        stringstream original;
                        original << fixed << setprecision(2) << result[i].second.first;
                        stringstream reduced;
                        reduced << fixed << setprecision(2) << result[i].second.second;
                        stringstream reduction;
                        double reductionValue = result[i].second.first == 0 ? 0 : 100 -
                                                                                  ((result[i].second.second * 1.0) /
                                                                                   result[i].second.first) * 100;
                        reduction << fixed << setprecision(2)
                                  << reductionValue;

                        cout << setw(4) << to_string(i + 1) << setw(10)
                             << " | " + reduction.str() << setw(COLUMN_WIDTH / 2) << left << " %";
                        cout << setw(COLUMN_WIDTH / 2) << result[i].second.first << setw(COLUMN_WIDTH / 2) << left
                             << result[i].second.second;
                        cout << result[i].first << endl;
                    }
                    break;
                }
                case 'b': {
                    return '\0';
                }
                case 'q': {
                    cout << "Thank you for using our Railway Network Management System!" << endl;
                    break;
                }
                default:
                    cout << "Please press one of listed keys." << endl;
                    break;
            }
        }
    }
    return commandIn;
}*/


/**
 * Outputs edge failure selection menu screen and returns a vector containing all the select edges for the given inputs
 * @return - vector<Edge*> containing all the Edges to be deactivated
 *
 */
/*vector<Edge *> Menu::edgeFailureMenu() {
    unsigned char commandIn;
    vector<Edge *> edges;

    cout << setw(COLUMN_WIDTH) << setfill(' ') << "Random rails: [1]" << setw(COLUMN_WIDTH)
         << "Specific rails: [2]" << endl;

    while (true) {
        cout << "Please select the rails you'd like to deactivate: ";
        cin >> commandIn;

        if (!checkInput(1)) {
            continue;
        }
        switch (commandIn) {
            case '1': {
                unsigned int numEdges;
                cout << "Please enter how many rails you'd like to deactivate: ";
                cin >> numEdges;
                if (!checkInput()) break;
                if (numEdges > graph.getTotalEdges()) {
                    cout << "The network only contains " << graph.getTotalEdges() << " rails!" << endl;
                    break;
                }
                vector<Edge *> deactivatedEdges = graph.randomlySelectEdges(numEdges);

                if (!deactivatedEdges.empty()) {
                    cout << "Deactivating the following rails: " << endl;
                    for (Edge *e: deactivatedEdges) {
                        e->print();
                    }
                } else cout << "Please provide edges for deactivation!" << endl;
                return deactivatedEdges;
            }
            case '2': {
                vector<Edge *> deactivatedEdges;

                while (true) {
                    string departureName;
                    cout << "Enter the name of the departure station, or q to finish: ";
                    getline(cin, departureName);
                    if (!checkInput()) break;

                    if (departureName == "q") break;

                    optional<Station> departureStation = dataRepository.findStation(departureName);
                    if (!departureStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    string arrivalName;
                    cout << "Enter the name of the arrival station, or q to finish: ";
                    getline(cin, arrivalName);
                    if (!checkInput()) break;

                    if (arrivalName == "q") break;

                    optional<Station> arrivalStation = dataRepository.findStation(arrivalName);
                    if (!arrivalStation.has_value()) {
                        stationDoesntExist();
                        break;
                    }

                    Vertex *departureVertex = graph.findVertex(departureName);
                    vector<Edge *> adjacentEdges = departureVertex->getAdj();
                    auto currentEdge = std::find_if(adjacentEdges.begin(),
                                                    adjacentEdges.end(),
                                                    [arrivalName](Edge *e) {
                                                        return e->getDest()->getId() == arrivalName;
                                                    });

                    if (currentEdge == adjacentEdges.end()) {
                        cout << "The two stations specified are not directly connected!" << endl;
                        continue;
                    } else deactivatedEdges.push_back(*currentEdge);
                }
                if (!deactivatedEdges.empty()) {
                    cout << "Deactivating the following rails: " << endl;
                    for (Edge *e: deactivatedEdges) {
                        e->print();
                    }
                } else cout << "Please provide edges for deactivation!" << endl;
                return deactivatedEdges;
            }
            default:
                cout << "Please press one of listed keys." << endl;
                break;
        }
    }
}*/
