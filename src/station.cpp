//
// Created by rita on 28-02-2023.
//

#include <utility>
#include "station.h"

Station::Station() = default;

Station::Station(unsigned int id, double latitude, double longitude, std::string name) : id(id),
                                                                                                latitude(latitude),
                                                                                                longitude(longitude),
                                                                                                name(std::move(name)) {}

//Getters

const std::string &Station::getName() const {
    return name;
}

void Station::setName(const std::string &name) {
    Station::name = name;
}

Station &Station::operator=(const Station &station) = default;

const unsigned int &Station::getId() const {
    return id;
}

void Station::setId(const unsigned int &id) {
    Station::id = id;
}

const double &Station::getLatitude() const {
    return latitude;
}

void Station::setLatitude(const double &latitude) {
    Station::latitude = latitude;
}

const double &Station::getLongitude() const {
    return longitude;
}

void Station::setLongitude(const double &longitude) {
    Station::longitude = longitude;
}




