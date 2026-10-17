#include "osm/OsmScenarioBuilder.h"

#include "core/CityBuilder.h"
#include "core/PathFinder.h"
#include "core/PedestrianCrossing.h"
#include "core/Phase.h"
#include "core/PhaseController.h"
#include "core/RoadSegment.h"
#include "core/TrafficLight.h"
#include "osm/OsmToMapDescription.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace {

uint32_t mixSeed(uint32_t seed, uint32_t salt) {
    return seed ^ (salt * 0x9E3779B9u);
}

// Simple LCG for deterministic generator/route selection (independent of VehicleGenerator).
class Lcg {
public:
    explicit Lcg(uint32_t seed) : state(seed == 0u ? 1u : seed) {
    }

    uint32_t next() {
        state = state * 1664525u + 1013904223u;
        return state;
    }

    size_t nextIndex(size_t n) {
        if (n == 0) {
            return 0;
        }
        return static_cast<size_t>(next() % static_cast<uint32_t>(n));
    }

private:
    uint32_t state;
};

vector<int> incomingRoadIds(const City& city, int intersectionId) {
    vector<int> ids;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getToIntersectionId() == intersectionId) {
            ids.push_back(road.getId());
        }
    }
    return ids;
}

// Weakly connected components on the undirected intersection graph.
vector<unordered_set<int>> intersectionComponents(const City& city) {
    unordered_map<int, vector<int>> adj;
    for (const auto& intersection : city.getIntersections()) {
        adj[intersection.getId()] = {};
    }
    for (const auto& road : city.getRoadSegments()) {
        const int a = road.getFromIntersectionId();
        const int b = road.getToIntersectionId();
        adj[a].push_back(b);
        adj[b].push_back(a);
    }

    unordered_set<int> visited;
    vector<unordered_set<int>> components;
    for (const auto& intersection : city.getIntersections()) {
        const int start = intersection.getId();
        if (visited.count(start) != 0) {
            continue;
        }
        unordered_set<int> component;
        queue<int> q;
        q.push(start);
        visited.insert(start);
        while (!q.empty()) {
            const int node = q.front();
            q.pop();
            component.insert(node);
            for (int neighbor : adj[node]) {
                if (visited.insert(neighbor).second) {
                    q.push(neighbor);
                }
            }
        }
        components.push_back(move(component));
    }

    sort(components.begin(), components.end(),
         [](const unordered_set<int>& a, const unordered_set<int>& b) {
             return a.size() > b.size();
         });
    return components;
}

unordered_set<int> roadsInIntersectionSet(const City& city, const unordered_set<int>& nodes) {
    unordered_set<int> roads;
    for (const auto& road : city.getRoadSegments()) {
        if (nodes.count(road.getFromIntersectionId()) != 0 &&
            nodes.count(road.getToIntersectionId()) != 0) {
            roads.insert(road.getId());
        }
    }
    return roads;
}

unordered_set<int> directedReachable(const City& city, int startRoadId, const unordered_set<int>& allow) {
    unordered_set<int> reached;
    if (allow.count(startRoadId) == 0 || city.findRoadSegmentById(startRoadId) == nullptr) {
        return reached;
    }
    queue<int> q;
    q.push(startRoadId);
    reached.insert(startRoadId);
    while (!q.empty()) {
        const int current = q.front();
        q.pop();
        for (int nextId : city.getOutgoingRoadIds(current)) {
            if (allow.count(nextId) == 0) {
                continue;
            }
            if (reached.insert(nextId).second) {
                q.push(nextId);
            }
        }
    }
    return reached;
}

double roadCruiseSpeed(const City& city, int roadId) {
    const RoadSegment* road = city.findRoadSegmentById(roadId);
    if (road == nullptr || road->getSpeedLimit() <= 0.0) {
        return 8.0;
    }
    return road->getSpeedLimit();
}

void attachCrossings(
    const MapDescription& map,
    Simulation& sim,
    const OsmScenarioConfig& config,
    const unordered_set<int>& preferRoadIds,
    size_t& attachedCount
) {
    attachedCount = 0;

    vector<MapCrossingDesc> preferred;
    vector<MapCrossingDesc> other;
    preferred.reserve(map.crossings.size());
    other.reserve(map.crossings.size());
    for (const auto& crossingDesc : map.crossings) {
        if (!preferRoadIds.empty() && preferRoadIds.count(crossingDesc.roadSegmentId) != 0) {
            preferred.push_back(crossingDesc);
        } else {
            other.push_back(crossingDesc);
        }
    }

    vector<MapCrossingDesc> ordered;
    ordered.reserve(map.crossings.size());
    ordered.insert(ordered.end(), preferred.begin(), preferred.end());
    ordered.insert(ordered.end(), other.begin(), other.end());

    const int cap = config.maxAttachedCrossings;
    const size_t limit =
        (cap <= 0) ? ordered.size() : min(ordered.size(), static_cast<size_t>(cap));

    // Even stride when we must thin a dense extract (keeps spatial coverage).
    const size_t stride = (cap > 0 && ordered.size() > limit && limit > 0)
        ? max<size_t>(1, ordered.size() / limit)
        : 1;

    unordered_set<int> usedIds;
    for (size_t i = 0; i < ordered.size() && attachedCount < limit; i += stride) {
        const MapCrossingDesc& crossingDesc = ordered[i];
        if (!usedIds.insert(crossingDesc.id).second) {
            continue;
        }

        const double inactive = config.crossingInactiveDuration > 0.0
            ? config.crossingInactiveDuration
            : crossingDesc.inactiveDuration;

        PedestrianCrossing crossing(
            crossingDesc.id,
            crossingDesc.roadSegmentId,
            crossingDesc.positionOnRoad,
            inactive,
            crossingDesc.activeDuration,
            crossingDesc.startActive
        );
        // OSM has no timed cycle — Unsignalized so yield-refusal config applies.
        crossing.setMode(CrossingMode::Unsignalized);
        sim.addPedestrianCrossing(crossing);
        ++attachedCount;
    }

    // If stride skipped too many, fill remaining slots from the head of ordered.
    for (size_t i = 0; i < ordered.size() && attachedCount < limit; ++i) {
        const MapCrossingDesc& crossingDesc = ordered[i];
        if (!usedIds.insert(crossingDesc.id).second) {
            continue;
        }
        const double inactive = config.crossingInactiveDuration > 0.0
            ? config.crossingInactiveDuration
            : crossingDesc.inactiveDuration;
        PedestrianCrossing crossing(
            crossingDesc.id,
            crossingDesc.roadSegmentId,
            crossingDesc.positionOnRoad,
            inactive,
            crossingDesc.activeDuration,
            crossingDesc.startActive
        );
        crossing.setMode(CrossingMode::Unsignalized);
        sim.addPedestrianCrossing(crossing);
        ++attachedCount;
    }
}

void attachGeneratedSignals(
    City& city,
    Simulation& sim,
    const OsmScenarioConfig& config,
    const unordered_set<int>& preferIntersectionIds,
    OsmScenarioResult& result
) {
    int nextControllerId = 1;
    int nextLightId = 1;

    vector<int> preferred;
    vector<int> other;
    for (const auto& intersection : city.getIntersections()) {
        if (intersection.getPriorityMode() != PriorityMode::Signalized) {
            continue;
        }
        if (!preferIntersectionIds.empty() &&
            preferIntersectionIds.count(intersection.getId()) != 0) {
            preferred.push_back(intersection.getId());
        } else {
            other.push_back(intersection.getId());
        }
    }
    sort(preferred.begin(), preferred.end());
    sort(other.begin(), other.end());

    vector<int> ordered;
    ordered.insert(ordered.end(), preferred.begin(), preferred.end());
    ordered.insert(ordered.end(), other.begin(), other.end());

    const int cap = config.maxGeneratedPhaseControllers;
    const size_t limit =
        (cap <= 0) ? ordered.size() : min(ordered.size(), static_cast<size_t>(cap));

    size_t signalizedSeen = 0;
    for (int intersectionId : ordered) {
        ++signalizedSeen;
        if (result.generatedPhaseControllerCount >= limit) {
            continue;
        }

        vector<int> approaches = incomingRoadIds(city, intersectionId);
        if (approaches.empty()) {
            result.notes.push_back(
                "Signalized intersection " + to_string(intersectionId) +
                " has no incoming roads; skipped generated phases"
            );
            continue;
        }

        // SAFE GENERATED plan (not from OSM timing): one approach green at a time.
        // Guarantees no simultaneous conflicting greens.
        PhaseController controller(nextControllerId, intersectionId);
        int phaseId = 1;
        for (int approachRoadId : approaches) {
            controller.addPhase(Phase(phaseId, config.phaseGreenSeconds, {approachRoadId}));
            sim.addTrafficLight(TrafficLight(
                nextLightId++,
                approachRoadId,
                intersectionId,
                true
            ));
            ++result.attachedTrafficLightCount;
            ++phaseId;
        }
        sim.addPhaseController(controller);
        ++result.generatedPhaseControllerCount;
        ++nextControllerId;
    }

    if (result.generatedPhaseControllerCount > 0) {
        ostringstream note;
        note << "Generated " << result.generatedPhaseControllerCount
             << " PhaseController(s) with sequential per-approach greens ("
             << config.phaseGreenSeconds
             << "s each). Timing is NOT from OSM — safe default only."
             << " Cap=" << (cap <= 0 ? string("unlimited") : to_string(cap))
             << " of " << signalizedSeen << " Signalized intersection(s).";
        result.notes.push_back(note.str());
    }
}

vector<vector<int>> buildRoutes(
    const City& city,
    const unordered_set<int>& componentRoads,
    const OsmScenarioConfig& config,
    Lcg& rng,
    size_t& largestReachable
) {
    vector<vector<int>> routes;
    largestReachable = 0;
    if (componentRoads.empty()) {
        return routes;
    }

    PathFinder pathFinder;
    vector<int> roadList(componentRoads.begin(), componentRoads.end());
    sort(roadList.begin(), roadList.end());

    unordered_set<int> demandComponent;
    for (int candidate : componentRoads) {
        const RoadSegment* cand = city.findRoadSegmentById(candidate);
        if (cand != nullptr && cand->getAllowAutoDemand()) {
            demandComponent.insert(candidate);
        }
    }

    auto routeUsesOnlyDemandRoads = [&](const vector<int>& route) {
        for (int roadId : route) {
            if (demandComponent.count(roadId) == 0) {
                return false;
            }
        }
        return true;
    };

    // Prefer starts with the largest directed reachable set inside the component.
    struct StartInfo {
        int roadId = -1;
        size_t reach = 0;
    };
    vector<StartInfo> starts;
    starts.reserve(roadList.size());
    for (int roadId : roadList) {
        if (demandComponent.count(roadId) == 0) {
            continue;
        }
        const size_t reach = directedReachable(city, roadId, demandComponent).size();
        if (reach > largestReachable) {
            largestReachable = reach;
        }
        if (reach >= 1) {
            starts.push_back({roadId, reach});
        }
    }
    sort(starts.begin(), starts.end(), [](const StartInfo& a, const StartInfo& b) {
        if (a.reach != b.reach) {
            return a.reach > b.reach;
        }
        return a.roadId < b.roadId;
    });

    unordered_set<uint64_t> seenPairs;
    auto pairKey = [](int a, int b) -> uint64_t {
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };

    const int want = max(0, config.maxGenerators);
    int attempts = 0;
    size_t startCursor = 0;

    while (static_cast<int>(routes.size()) < want && attempts < config.maxRouteSearchAttempts) {
        ++attempts;
        if (starts.empty()) {
            break;
        }

        const StartInfo& start = starts[startCursor % starts.size()];
        ++startCursor;

        const unordered_set<int> reachable =
            directedReachable(city, start.roadId, demandComponent);
        vector<int> goals;
        goals.reserve(reachable.size());
        for (int goalId : reachable) {
            if (goalId == start.roadId) {
                continue;
            }
            goals.push_back(goalId);
        }
        if (goals.empty()) {
            // Single-road component: allow a one-segment route if valid.
            vector<int> solo = {start.roadId};
            if (city.isValidRoute(solo)) {
                const uint64_t key = pairKey(start.roadId, start.roadId);
                if (seenPairs.insert(key).second) {
                    routes.push_back(solo);
                }
            }
            continue;
        }

        // Prefer moderate hop counts (credible on large extracts). maxRouteHops<=0 → legacy longest.
        const int minHops = max(1, config.minRouteHops);
        const int maxHops = config.maxRouteHops;
        vector<int> best;
        int bestScore = -1;
        const size_t samples = min<size_t>(12, goals.size());
        for (size_t s = 0; s < samples; ++s) {
            const int goalId = goals[rng.nextIndex(goals.size())];
            const uint64_t key = pairKey(start.roadId, goalId);
            if (seenPairs.count(key) != 0) {
                continue;
            }
            vector<int> route = pathFinder.findShortestRoute(city, start.roadId, goalId);
            if (route.empty() || !city.isValidRoute(route) || !routeUsesOnlyDemandRoads(route)) {
                continue;
            }
            // Truncate long paths so large extracts stay completable in finite sim time.
            if (maxHops > 0 && static_cast<int>(route.size()) > maxHops) {
                route.resize(static_cast<size_t>(maxHops));
                if (!city.isValidRoute(route) || !routeUsesOnlyDemandRoads(route)) {
                    continue;
                }
            }
            const int hops = static_cast<int>(route.size());
            // Score: within [minHops, maxHops] prefer longer; below minHops still usable but lower.
            int score = hops;
            if (hops < minHops) {
                score = hops - (minHops - hops) * 2;
            }
            if (best.empty() || score > bestScore) {
                best = route;
                bestScore = score;
            }
        }

        if (best.empty()) {
            continue;
        }

        const int endRoad = best.back();
        const uint64_t key = pairKey(start.roadId, endRoad);
        if (!seenPairs.insert(key).second) {
            continue;
        }
        routes.push_back(best);
    }

    return routes;
}

void attachGenerators(
    const City& city,
    Simulation& sim,
    const OsmScenarioConfig& config,
    OsmScenarioResult& result
) {
    const vector<unordered_set<int>> components = intersectionComponents(city);
    unordered_set<int> componentRoads;
    if (!components.empty()) {
        componentRoads = roadsInIntersectionSet(city, components.front());
    }
    result.largestComponentRoadCount = componentRoads.size();

    Lcg rng(mixSeed(config.seed, 0xA11ACCEu));
    size_t largestReachable = 0;
    const vector<vector<int>> routes =
        buildRoutes(city, componentRoads, config, rng, largestReachable);

    ostringstream routingNote;
    routingNote << "Routing limited to largest weakly-connected intersection component ("
                << componentRoads.size() << " directed roads; max directed reach from a start="
                << largestReachable << "). PathFinder uses directed edges + City turn disallows "
                << "(MapConnectionDesc from enforced OSM no_*/only_* via-node restrictions). "
                << "Auto demand skips roads with allowAutoDemand=false (destination/conditional "
                << "access). turn:lanes / via-way / conditional restrictions remain unsupported.";
    result.notes.push_back(routingNote.str());

    int generatorIndex = 0;
    for (const auto& route : routes) {
        if (route.empty() || !city.isValidRoute(route)) {
            continue;
        }

        const double speed = roadCruiseSpeed(city, route.front());
        VehicleGenerator generator(
            config.spawnIntervalSeconds,
            speed,
            config.vehiclesPerGenerator,
            config.spawnSafeDistance
        );
        generator.setSeed(mixSeed(config.seed, static_cast<uint32_t>(17 + generatorIndex * 31)));
        generator.setRoute(route);

        const RoadSegment* first = city.findRoadSegmentById(route.front());
        if (first != nullptr) {
            generator.setLaneCountHint(first->getLaneCount());
        }

        sim.addVehicleGenerator(generator);
        ++result.attachedGeneratorCount;
        ++result.routeCount;
        ++generatorIndex;
    }

    if (result.attachedGeneratorCount == 0) {
        result.notes.push_back(
            "No valid generator routes found in the largest useful component; "
            "simulation has zero demand."
        );
    }
}

void copySimOutputTargets(const Simulation& from, Simulation& to) {
    // Simulation does not expose getters for output paths; Stage 5 session re-sets them.
    // Keep this hook for future if needed. For now, staging sims use default paths.
    (void)from;
    (void)to;
}

} // namespace

OsmScenarioResult OsmScenarioBuilder::attach(
    const MapDescription& map,
    const ImportReport& report,
    City& outCity,
    Simulation& outSim,
    const OsmScenarioConfig& config
) {
    OsmScenarioResult result;
    result.map = map;
    result.report = report;

    string error;
    if (!CityBuilder::build(map, outCity, error)) {
        result.ok = false;
        result.error = error.empty() ? "CityBuilder::build failed" : error;
        outCity = City();
        return result;
    }

    outSim.clear();
    outSim.setMinVehicleGap(config.minVehicleGap);

    // Generators first so signals/crossings can prefer the demand corridor.
    attachGenerators(outCity, outSim, config, result);

    unordered_set<int> routeRoads;
    unordered_set<int> routeIntersections;
    for (const auto& generator : outSim.getVehicleGenerators()) {
        for (int roadId : generator.getRoute()) {
            routeRoads.insert(roadId);
            if (const RoadSegment* road = outCity.findRoadSegmentById(roadId)) {
                routeIntersections.insert(road->getFromIntersectionId());
                routeIntersections.insert(road->getToIntersectionId());
            }
        }
    }

    attachGeneratedSignals(outCity, outSim, config, routeIntersections, result);
    attachCrossings(map, outSim, config, routeRoads, result.attachedCrossingCount);

    outSim.setPedestrianYieldRefusalProbability(config.yieldRefusalProbability);
    outSim.setPedestrianSeed(mixSeed(config.seed, 0x50u));
    outSim.setLaneBlockSeed(mixSeed(config.seed, 0x60u));
    outSim.captureVehicleSpawnBaselines();

    result.ok = true;
    {
        ostringstream pedNote;
        pedNote << "PedestrianCrossing modes set to Unsignalized; yieldRefusalProbability="
                << config.yieldRefusalProbability
                << "; attached=" << result.attachedCrossingCount
                << " of " << map.crossings.size()
                << " map crossings (cap="
                << (config.maxAttachedCrossings <= 0 ? string("unlimited")
                                                     : to_string(config.maxAttachedCrossings))
                << ", inactiveDuration=" << config.crossingInactiveDuration << "s)";
        result.notes.push_back(pedNote.str());
    }
    return result;
}

OsmScenarioResult OsmScenarioBuilder::loadFromConvertResult(
    const MapDescription& map,
    const ImportReport& report,
    bool convertOk,
    const string& convertError,
    City& city,
    Simulation& sim,
    const OsmScenarioConfig& config
) {
    OsmScenarioResult result;
    result.map = map;
    result.report = report;

    if (!convertOk) {
        result.ok = false;
        result.error = convertError.empty() ? "OSM convert failed" : convertError;
        return result; // city / sim unchanged
    }

    City stagingCity;
    Simulation stagingSim;
    copySimOutputTargets(sim, stagingSim);

    result = attach(map, report, stagingCity, stagingSim, config);
    if (!result.ok) {
        return result; // city / sim unchanged
    }

    city = move(stagingCity);
    sim = move(stagingSim);
    return result;
}

OsmScenarioResult OsmScenarioBuilder::loadOsmFile(
    const string& path,
    City& city,
    Simulation& sim,
    const OsmScenarioConfig& config
) {
    const OsmToMapResult converted = OsmToMapDescription::convertFile(path);
    return loadFromConvertResult(
        converted.map,
        converted.report,
        converted.ok,
        converted.error,
        city,
        sim,
        config
    );
}
