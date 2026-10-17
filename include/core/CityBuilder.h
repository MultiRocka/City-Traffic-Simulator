#pragma once

#include <string>
#include "core/City.h"
#include "core/MapDescription.h"

using namespace std;

class CityBuilder {
public:
    // Builds outCity from description. On failure, outCity is left empty and error is set.
    // Crossings in the description are validated against road lengths but not stored on City
    // (Simulation still owns PedestrianCrossing instances).
    // Length is authoritative from MapRoadDesc::length; intermediate points are geometry only.
    // Optional consistency: if intermediates are present, polyline length (from→pts→to)
    // must be within polylineLengthTolerance meters of length (default 1.0).
    static bool build(const MapDescription& description, City& outCity, string& error);

    static double polylineLengthTolerance;
};
