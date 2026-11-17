#pragma once

#include "core/MapDescription.h"
#include "osm/ImportReport.h"

// Etap 2: post-import geometry continuity passes.
// Collapse micro-stubs that only exist because a degree-2 soft junction
// (signal / stop / give_way) sits within a few meters of a real fork.
// Does NOT invent dual-carriage connectors or proximity joins across layers.

struct MapGeometryContinuityStats {
    int mergedDirectedRoads = 0;
    int removedIntersections = 0;
    int remainingVeryShortLt5m = 0;
};

class MapGeometryContinuity {
public:
    // Default threshold matches Etap 1 "very short" flag (< 5 m).
    static constexpr double kDefaultMinKeepLengthM = 5.0;

    // Mutates map in place. Updates report notes when report != nullptr.
    // Safe to call on synthetic MapDescriptions (no OSM required).
    static MapGeometryContinuityStats mergeShortDegree2Stubs(
        MapDescription& map,
        ImportReport* report = nullptr,
        double minKeepLengthM = kDefaultMinKeepLengthM
    );
};
