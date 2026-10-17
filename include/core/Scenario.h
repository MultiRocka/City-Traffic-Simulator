#pragma once

#include "core/City.h"
#include "core/MapDescription.h"
#include "core/Simulation.h"

class Scenario {
public:
    // Stage 7 default demo: one clear MapDescription → CityBuilder path (importer-ready).
    static MapDescription buildDemoMapDescription();
    static void setupDemoCity(City& city, Simulation& sim);

    // Historical regression grid (kept for older tests / manual load).
    static void setupIntersectionTest(City& city, Simulation& sim);
};
