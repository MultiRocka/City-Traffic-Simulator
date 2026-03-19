#pragma once

#include <string>
#include <vector>
#include "core/Intersection.h"
#include "core/RoadSegment.h"

using namespace std;

class City {
public:
    City();
    City(const string& name);

    void setName(const string& newName);
    string getName() const;

    void addIntersection(const Intersection& intersection);
    void addRoadSegment(const RoadSegment& roadSegment);

    const vector<Intersection>& getIntersections() const;
    const vector<RoadSegment>& getRoadSegments() const;

    Intersection* findIntersectionById(int id);
    RoadSegment* findRoadSegmentById(int id);

    const Intersection* findIntersectionById(int id) const;
    const RoadSegment* findRoadSegmentById(int id) const;

private:
    string name;
    vector<Intersection> intersections;
    vector<RoadSegment> roadSegments;
};
