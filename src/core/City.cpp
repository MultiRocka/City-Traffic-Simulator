#include "core/City.h"

using namespace std;

City::City() : name("Unnamed City") {
}

City::City(const string& name) : name(name) {
}

void City::setName(const string& newName) {
    name = newName;
}

string City::getName() const {
    return name;
}

void City::addIntersection(const Intersection& intersection) {
    intersections.push_back(intersection);
}

void City::addRoadSegment(const RoadSegment& roadSegment) {
    roadSegments.push_back(roadSegment);
}

const vector<Intersection>& City::getIntersections() const {
    return intersections;
}

const vector<RoadSegment>& City::getRoadSegments() const {
    return roadSegments;
}

Intersection* City::findIntersectionById(int id) {
    for (auto& intersection : intersections) {
        if (intersection.getId() == id) {
            return &intersection;
        }
    }

    return nullptr;
}

RoadSegment* City::findRoadSegmentById(int id) {
    for (auto& roadSegment : roadSegments) {
        if (roadSegment.getId() == id) {
            return &roadSegment;
        }
    }

    return nullptr;
}
