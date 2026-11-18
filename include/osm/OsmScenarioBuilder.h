#pragma once

#include "core/City.h"
#include "core/MapDescription.h"
#include "core/Simulation.h"
#include "core/VehicleGenerator.h"
#include "osm/ImportReport.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using namespace std;

// Stage 4: MapDescription (+ ImportReport) → City + Simulation attachments.
// Signal timing is GENERATED (not from OSM). Routes use PathFinder on directed edges.
// Honors MapConnectionDesc turn disallows and RoadSegment::allowAutoDemand from Stage 3.
// Does not invent extra restrictions beyond the imported map.

enum class TrafficIntensityPreset {
    Custom = 0,
    Small = 1,
    Medium = 2,
    Large = 3
};

struct OsmScenarioConfig {
    uint32_t seed = VehicleGenerator::defaultSeed;

    // Finite demand — not an endless flood (unless continuousDemand).
    int maxGenerators = 8;
    int vehiclesPerGenerator = 5;
    double spawnIntervalSeconds = 2.5;
    double spawnSafeDistance = 6.0;
    double minVehicleGap = 4.0;

    // Etap 3: continuous urban demand. When true, generators keep spawning (no finite
    // per-generator quota). Soft concurrent cap is maxActiveVehicles (0 = unlimited).
    // Vehicles are never deleted to free slots; no unbounded spawn request queue.
    bool continuousDemand = false;
    int maxActiveVehicles = 0;

    // Prefer spatially spread starts (reduces entrance pile-up vs reach-sorted only).
    bool spatiallyDistributeStarts = true;

    // Named intensity presets (visible limit + intensity). Custom leaves fields as set.
    TrafficIntensityPreset intensityPreset = TrafficIntensityPreset::Custom;

    // Generated PhaseController greens (seconds per approach phase).
    double phaseGreenSeconds = 8.0;

    // PedestrianManager yield-refusal (Unsignalized crossings).
    double yieldRefusalProbability = 0.05;

    // Cap how many start/goal pairs we try when building routes.
    int maxRouteSearchAttempts = 240;

    // Prefer moderate hop counts on large extracts (0 max = unlimited / legacy longest).
    int minRouteHops = 2;
    int maxRouteHops = 12;

    // Cap Unsignalized OSM crossings attached for ped demand (0 = unlimited).
    // Prefer crossings on generator-route roads when available.
    int maxAttachedCrossings = 32;
    // Base ped request interval uses crossing inactiveDuration (seconds).
    double crossingInactiveDuration = 18.0;

    // Cap generated PhaseControllers (0 = unlimited). Prefer intersections on route roads.
    // Dense OSM extracts often tag many traffic_signals; unlimited sequential greens
    // creates long cycles that prevent credible completion in offline checks.
    int maxGeneratedPhaseControllers = 24;

    // Factory helpers — set continuousDemand, maxActive, generators, interval.
    static OsmScenarioConfig presetSmall(uint32_t seed = VehicleGenerator::defaultSeed);
    static OsmScenarioConfig presetMedium(uint32_t seed = VehicleGenerator::defaultSeed);
    static OsmScenarioConfig presetLarge(uint32_t seed = VehicleGenerator::defaultSeed);
};

struct OsmScenarioResult {
    bool ok = false;
    string error;

    MapDescription map;
    ImportReport report;

    size_t attachedCrossingCount = 0;
    size_t generatedPhaseControllerCount = 0;
    size_t attachedTrafficLightCount = 0;
    size_t attachedGeneratorCount = 0;
    size_t routeCount = 0;
    size_t largestComponentRoadCount = 0;
    size_t distinctStartRoadCount = 0;

    // Human-readable notes (generated signals, routing limits, etc.).
    vector<string> notes;
};

class OsmScenarioBuilder {
public:
    // Build City via CityBuilder and attach crossings / generated signals / generators
    // onto a fresh Simulation (caller should pass a cleared or staging Simulation).
    // Does not hardcode Demo City ids.
    static OsmScenarioResult attach(
        const MapDescription& map,
        const ImportReport& report,
        City& outCity,
        Simulation& outSim,
        const OsmScenarioConfig& config = {}
    );

    // Parse + convert + attach. On failure, outCity / outSim are left unchanged
    // (prepare on staging objects, then swap).
    static OsmScenarioResult loadOsmFile(
        const string& path,
        City& city,
        Simulation& sim,
        const OsmScenarioConfig& config = {}
    );

    // Convert already-parsed OpenStreetMap JSON path through OsmToMapDescription, then attach.
    static OsmScenarioResult loadFromConvertResult(
        const MapDescription& map,
        const ImportReport& report,
        bool convertOk,
        const string& convertError,
        City& city,
        Simulation& sim,
        const OsmScenarioConfig& config = {}
    );
};
