//
// Created by rita on 28-02-2023.
//

#ifndef TRAVELLINGSALESMAN_STATION_H
#define TRAVELLINGSALESMAN_STATION_H

#include <string>
#include <unordered_map>
#include <unordered_set>

class Station {
  private:
    unsigned int id;
    double latitude;
    double longitude;
    std::string name;
  public:
    Station();

    Station(unsigned int id, double latitude, double longitude, std::string name);

    const unsigned int &getId() const;

    void setId(const unsigned int &id);

    const double &getLatitude() const;

    void setLatitude(const double &latitude);

    const double &getLongitude() const;

    void setLongitude(const double &longitude);

    [[nodiscard]] const std::string &getName() const;

    void setName(const std::string &name);

    Station &operator=(const Station &station);
};


struct StationHash {
    std::size_t operator()(const Station &Station) const {
        return std::hash<unsigned int>()(Station.getId());
    }
};

struct StationEquals {
    bool operator()(const Station &Station1, const Station &Station2) const {
        return Station1.getId() == Station2.getId();
    }
};

typedef std::unordered_set<Station, StationHash, StationEquals> stationTable;

template<typename T>
using StationMap = std::unordered_map<Station, T, StationHash, StationEquals>;


#endif //TRAVELLINGSALESMAN_STATION_H
