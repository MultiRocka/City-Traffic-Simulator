#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/CollisionDetector.h"
#include "core/MapDescription.h"
#include "core/Simulation.h"
#include "core/Vehicle.h"
#include "core/VehicleGenerator.h"
#include "osm/OsmScenarioBuilder.h"
#include "osm/OsmToMapDescription.h"
#include "ui/CityLayout.h"
#include "ui/RenderComposer.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace std;
namespace fs = std::filesystem;

namespace {

int g_failures = 0;
int g_passed = 0;

void expect(bool condition, const string& name) {
    if (condition) {
        ++g_passed;
        cout << "PASS  " << name << "\n";
    } else {
        ++g_failures;
        cout << "FAIL  " << name << "\n";
    }
}

void ensureOutputDir() {
    fs::create_directories("tests/output");
}

string fixturePath(const string& name) {
    return string("tests/fixtures/geometry/") + name;
}

string existingOsmFixture(const string& name) {
    return string("data/osm/fixtures/") + name;
}

struct LayerDefects {
    vector<string> osmTopology;
    vector<string> physicalGeometry;
    vector<string> lanePaths;
    vector<string> turnConnections;
    vector<string> vehicleTrajectories;
    vector<string> renderLod;
};

struct GeometryMetrics {
    string caseName;
    string sourcePath;
    bool convertOk = false;
    bool cityOk = false;
    string error;

    // OSM topology
    size_t intersections = 0;
    size_t roads = 0;
    size_t crossings = 0;
    size_t disallowedTurns = 0;
    int maxNodeDegree = 0;
    size_t deadEndRoads = 0;
    size_t weaklyConnectedComponents = 0;

    // Physical geometry
    double minLengthM = 0.0;
    double maxLengthM = 0.0;
    double maxAbsLengthMinusChordM = 0.0;
    double maxAbsLengthMinusPolylineM = 0.0;
    int roadsWithIntermediates = 0;
    int veryShortRoadsLt5m = 0;
    int polylineMismatchBeyondTolerance = 0;

    // Lane paths (layout)
    double minLaneSeparationM = 0.0;
    double maxLaneSeparationErrorM = 0.0;
    int multiLaneRoadsSampled = 0;
    int oppositePairNearOverlap = 0;

    // Turn connections
    size_t geometricTurnPairs = 0;
    size_t allowedTurnPairs = 0;
    size_t uTurnSameSegment = 0;

    // Vehicle trajectories (short probe)
    bool trajectoryProbeRan = false;
    double maxStepJumpM = 0.0;
    int positionOverrunTicks = 0;
    int completedProbeVehicles = 0;

    // Render / LOD (numeric snapshot)
    size_t renderRoads = 0;
    size_t emptyCenterlines = 0;
    double maxRenderMotionDeltaM = 0.0;

    LayerDefects defects;
};

double polylineLen(const vector<Vector2D>& pts) {
    double sum = 0.0;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        sum += pts[i].distanceTo(pts[i + 1]);
    }
    return sum;
}

size_t countWeakComponents(const City& city) {
    map<int, vector<int>> adj;
    for (const auto& road : city.getRoadSegments()) {
        adj[road.getFromIntersectionId()].push_back(road.getToIntersectionId());
        adj[road.getToIntersectionId()].push_back(road.getFromIntersectionId());
    }
    set<int> seen;
    size_t components = 0;
    for (const auto& node : city.getIntersections()) {
        if (!seen.insert(node.getId()).second) {
            continue;
        }
        ++components;
        vector<int> stack = {node.getId()};
        while (!stack.empty()) {
            const int cur = stack.back();
            stack.pop_back();
            for (int nxt : adj[cur]) {
                if (seen.insert(nxt).second) {
                    stack.push_back(nxt);
                }
            }
        }
    }
    return components;
}

GeometryMetrics measureMap(const string& caseName, const MapDescription& desc, const string& sourcePath) {
    GeometryMetrics m;
    m.caseName = caseName;
    m.sourcePath = sourcePath;
    m.convertOk = true;
    m.intersections = desc.intersections.size();
    m.roads = desc.roads.size();
    m.crossings = desc.crossings.size();
    m.disallowedTurns = desc.connections.size();

    std::map<int, const MapIntersectionDesc*> byId;
    for (const auto& node : desc.intersections) {
        byId[node.id] = &node;
    }

    std::map<int, int> degree;
    m.minLengthM = desc.roads.empty() ? 0.0 : desc.roads.front().length;
    for (const auto& road : desc.roads) {
        degree[road.fromIntersectionId]++;
        degree[road.toIntersectionId]++;
        m.minLengthM = min(m.minLengthM, road.length);
        m.maxLengthM = max(m.maxLengthM, road.length);
        if (road.length < 5.0) {
            ++m.veryShortRoadsLt5m;
        }

        const auto* from = byId[road.fromIntersectionId];
        const auto* to = byId[road.toIntersectionId];
        if (from == nullptr || to == nullptr) {
            m.defects.osmTopology.push_back(
                "road " + to_string(road.id) + " references missing intersection"
            );
            continue;
        }

        const double chord = from->position.distanceTo(to->position);
        m.maxAbsLengthMinusChordM = max(m.maxAbsLengthMinusChordM, fabs(road.length - chord));

        vector<Vector2D> poly;
        poly.push_back(from->position);
        poly.insert(poly.end(), road.intermediatePoints.begin(), road.intermediatePoints.end());
        poly.push_back(to->position);
        if (!road.intermediatePoints.empty()) {
            ++m.roadsWithIntermediates;
            const double polyL = polylineLen(poly);
            const double diff = fabs(road.length - polyL);
            m.maxAbsLengthMinusPolylineM = max(m.maxAbsLengthMinusPolylineM, diff);
            if (diff > CityBuilder::polylineLengthTolerance) {
                ++m.polylineMismatchBeyondTolerance;
                m.defects.physicalGeometry.push_back(
                    "road " + to_string(road.id) + " length vs polyline diff=" + to_string(diff) + " m"
                );
            }
        }

        // Chord much shorter than length with no intermediates: layout will compress motion.
        if (road.intermediatePoints.empty() && road.length > chord + 1.0 && chord > 1e-6) {
            m.defects.physicalGeometry.push_back(
                "road " + to_string(road.id) + " length=" + to_string(road.length) +
                " exceeds chord=" + to_string(chord) + " without intermediates (layout stretch)"
            );
        }
        if (road.intermediatePoints.empty() && chord > road.length + 1.0) {
            m.defects.physicalGeometry.push_back(
                "road " + to_string(road.id) + " chord=" + to_string(chord) +
                " exceeds length=" + to_string(road.length) + " (layout compress)"
            );
        }
    }
    for (const auto& kv : degree) {
        m.maxNodeDegree = max(m.maxNodeDegree, kv.second);
    }

    City city;
    string error;
    m.cityOk = CityBuilder::build(desc, city, error);
    m.error = error;
    if (!m.cityOk) {
        m.defects.osmTopology.push_back("CityBuilder failed: " + error);
        return m;
    }

    m.weaklyConnectedComponents = countWeakComponents(city);
    if (m.weaklyConnectedComponents > 1) {
        m.defects.osmTopology.push_back(
            "weakly-connected components=" + to_string(m.weaklyConnectedComponents)
        );
    }

    CityLayout layout;
    m.minLaneSeparationM = 1e9;
    for (const auto& road : city.getRoadSegments()) {
        const auto outgoing = city.getOutgoingRoadIds(road.getId());
        if (outgoing.empty()) {
            ++m.deadEndRoads;
        }
        for (int toId : outgoing) {
            ++m.geometricTurnPairs;
            if (city.areRoadsConnected(road.getId(), toId)) {
                ++m.allowedTurnPairs;
            } else {
                m.defects.turnConnections.push_back(
                    "geometric turn blocked " + to_string(road.getId()) + "->" + to_string(toId)
                );
            }
        }
        if (city.areRoadsGeometricallyConnected(road.getId(), road.getId())) {
            ++m.uTurnSameSegment;
        }

        const int lanes = road.getLaneCount();
        if (lanes >= 2) {
            ++m.multiLaneRoadsSampled;
            const Vector2D p0 = layout.getRoadPosition(city, road.getId(), road.getLength() * 0.5, 0, lanes);
            const Vector2D p1 = layout.getRoadPosition(city, road.getId(), road.getLength() * 0.5, 1, lanes);
            const double sep = p0.distanceTo(p1);
            m.minLaneSeparationM = min(m.minLaneSeparationM, sep);
            const double expected = SimConstants::LANE_WIDTH;
            m.maxLaneSeparationErrorM = max(m.maxLaneSeparationErrorM, fabs(sep - expected));
            if (sep < expected * 0.5) {
                m.defects.lanePaths.push_back(
                    "road " + to_string(road.getId()) + " lane separation too small: " + to_string(sep)
                );
            }
        }
    }
    if (m.multiLaneRoadsSampled == 0) {
        m.minLaneSeparationM = 0.0;
    }

    // Dual-carriage / opposite pair proximity (shared name prefix heuristic + parallel endpoints).
    for (size_t i = 0; i < city.getRoadSegments().size(); ++i) {
        for (size_t j = i + 1; j < city.getRoadSegments().size(); ++j) {
            const auto& a = city.getRoadSegments()[i];
            const auto& b = city.getRoadSegments()[j];
            if (a.getFromIntersectionId() == b.getToIntersectionId() &&
                a.getToIntersectionId() == b.getFromIntersectionId()) {
                // Opposite directed pair on same corridor — expected for two-way.
                continue;
            }
            const Intersection* aFrom = city.findIntersectionById(a.getFromIntersectionId());
            const Intersection* aTo = city.findIntersectionById(a.getToIntersectionId());
            const Intersection* bFrom = city.findIntersectionById(b.getFromIntersectionId());
            const Intersection* bTo = city.findIntersectionById(b.getToIntersectionId());
            if (aFrom == nullptr || aTo == nullptr || bFrom == nullptr || bTo == nullptr) {
                continue;
            }
            const double midDist = Vector2D(
                (aFrom->getPosition().x + aTo->getPosition().x) * 0.5,
                (aFrom->getPosition().y + aTo->getPosition().y) * 0.5
            ).distanceTo(Vector2D(
                (bFrom->getPosition().x + bTo->getPosition().x) * 0.5,
                (bFrom->getPosition().y + bTo->getPosition().y) * 0.5
            ));
            // Parallel oneways closer than ~half lane width with no shared nodes: visual island risk.
            if (midDist > 0.05 && midDist < SimConstants::LANE_WIDTH * 0.5 &&
                a.getFromIntersectionId() != b.getFromIntersectionId() &&
                a.getFromIntersectionId() != b.getToIntersectionId() &&
                a.getToIntersectionId() != b.getFromIntersectionId() &&
                a.getToIntersectionId() != b.getToIntersectionId()) {
                ++m.oppositePairNearOverlap;
                m.defects.lanePaths.push_back(
                    "near-overlap parallel roads " + to_string(a.getId()) + "/" + to_string(b.getId()) +
                    " midDist=" + to_string(midDist)
                );
            }
        }
    }

    // Trajectory probe: place one vehicle on the longest road if possible.
    double longest = 0.0;
    int probeRoad = -1;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLength() > longest) {
            longest = road.getLength();
            probeRoad = road.getId();
        }
    }
    if (probeRoad >= 0 && longest >= 8.0) {
        Simulation sim;
        vector<int> route = {probeRoad};
        // Extend route one hop if available.
        const auto outs = city.getOutgoingRoadIds(probeRoad);
        if (!outs.empty()) {
            route.push_back(outs.front());
        }
        Vehicle v(1, 8.0);
        v.setRoute(route);
        v.setDesiredSpeed(8.0);
        v.setMaxSpeed(8.0);
        sim.addVehicle(v);
        sim.start();

        CityLayout trajLayout;
        Vector2D prev = trajLayout.getRoadPosition(city, probeRoad, 0.0);
        m.trajectoryProbeRan = true;
        for (int step = 0; step < 200; ++step) {
            sim.update(0.1, city);
            for (const auto& vehicle : sim.getVehicles()) {
                if (vehicle.isFinished()) {
                    ++m.completedProbeVehicles;
                    continue;
                }
                const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
                if (road != nullptr && vehicle.getPositionOnRoad() > road->getLength() + 0.5) {
                    ++m.positionOverrunTicks;
                    m.defects.vehicleTrajectories.push_back(
                        "position overrun road=" + to_string(road->getId()) +
                        " pos=" + to_string(vehicle.getPositionOnRoad()) +
                        " len=" + to_string(road->getLength())
                    );
                }
                const Vector2D now = trajLayout.getRoadPosition(
                    city,
                    vehicle.getCurrentRoadSegmentId(),
                    vehicle.getPositionOnRoad(),
                    vehicle.getLaneIndex(),
                    road != nullptr ? road->getLaneCount() : 1
                );
                const double jump = prev.distanceTo(now);
                // At 8 m/s * 0.1 s = 0.8 m expected; jumps > 25 m indicate discontinuous layout mapping.
                m.maxStepJumpM = max(m.maxStepJumpM, jump);
                if (jump > 25.0) {
                    m.defects.vehicleTrajectories.push_back(
                        "world jump " + to_string(jump) + " m at t=" + to_string(sim.getTime())
                    );
                }
                prev = now;
            }
        }
    }

    RenderComposer composer;
    Simulation emptySim;
    // Snapshot roads from city via a tiny sim attachment not required — use CityRenderer path:
    RenderSnapshot snap = composer.buildSnapshot(city, emptySim);
    m.renderRoads = snap.roads.size();
    for (const auto& roadInfo : snap.roads) {
        if (roadInfo.centerline.size() < 2) {
            ++m.emptyCenterlines;
            m.defects.renderLod.push_back(
                "empty/short centerline roadId=" + to_string(roadInfo.roadId)
            );
        }
        const RoadSegment* road = city.findRoadSegmentById(roadInfo.roadId);
        if (road == nullptr || road->getLength() <= 1e-9) {
            continue;
        }
        // Compare mid-motion layout vs mid-centerline sample.
        const Vector2D motionMid = layout.getRoadPosition(city, road->getId(), road->getLength() * 0.5);
        Vector2D renderMid = roadInfo.centerline.front();
        if (roadInfo.centerline.size() >= 2) {
            const double total = polylineLen(roadInfo.centerline);
            double walked = 0.0;
            const double target = total * 0.5;
            renderMid = roadInfo.centerline.back();
            for (size_t i = 0; i + 1 < roadInfo.centerline.size(); ++i) {
                const double seg = roadInfo.centerline[i].distanceTo(roadInfo.centerline[i + 1]);
                if (walked + seg >= target) {
                    const double local = seg > 1e-9 ? (target - walked) / seg : 0.0;
                    renderMid = Vector2D(
                        roadInfo.centerline[i].x + (roadInfo.centerline[i + 1].x - roadInfo.centerline[i].x) * local,
                        roadInfo.centerline[i].y + (roadInfo.centerline[i + 1].y - roadInfo.centerline[i].y) * local
                    );
                    break;
                }
                walked += seg;
            }
        }
        const double delta = motionMid.distanceTo(renderMid);
        m.maxRenderMotionDeltaM = max(m.maxRenderMotionDeltaM, delta);
        if (delta > 1.0) {
            m.defects.renderLod.push_back(
                "render vs motion mid delta=" + to_string(delta) + " road=" + to_string(road->getId())
            );
        }
    }

    return m;
}

GeometryMetrics measureOsmFile(const string& caseName, const string& path) {
    GeometryMetrics m;
    m.caseName = caseName;
    m.sourcePath = path;
    if (!fs::exists(path)) {
        m.error = "missing file";
        m.defects.osmTopology.push_back("fixture missing: " + path);
        return m;
    }
    const OsmToMapResult converted = OsmToMapDescription::convertFile(path);
    m.convertOk = converted.ok;
    if (!converted.ok) {
        m.error = converted.error;
        m.defects.osmTopology.push_back("convert failed: " + converted.error);
        return m;
    }
    return measureMap(caseName, converted.map, path);
}

MapDescription makeSyntheticStraight1Plus1() {
    MapDescription map;
    map.name = "synth_straight_1_1";
    MapIntersectionDesc a{1, Vector2D(0, 0), "", PriorityMode::None, {}};
    MapIntersectionDesc b{2, Vector2D(100, 0), "", PriorityMode::None, {}};
    map.intersections = {a, b};
    MapRoadDesc ab{10, 1, 2, 100.0, 13.9, 1, {}, "synth/ab", true};
    MapRoadDesc ba{11, 2, 1, 100.0, 13.9, 1, {}, "synth/ba", true};
    map.roads = {ab, ba};
    return map;
}

MapDescription makeSyntheticGentleCurve() {
    MapDescription map;
    map.name = "synth_gentle_curve";
    MapIntersectionDesc a{1, Vector2D(0, 0), "", PriorityMode::None, {}};
    MapIntersectionDesc b{2, Vector2D(80, 10), "", PriorityMode::None, {}};
    map.intersections = {a, b};
    vector<Vector2D> mids = {Vector2D(25, 8), Vector2D(55, 12)};
    vector<Vector2D> poly = {a.position};
    poly.insert(poly.end(), mids.begin(), mids.end());
    poly.push_back(b.position);
    MapRoadDesc road{1, 1, 2, polylineLen(poly), 11.0, 1, mids, "synth/curve", true};
    map.roads = {road};
    return map;
}

MapDescription makeSyntheticSharpTurn() {
    MapDescription map;
    map.name = "synth_sharp_turn";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(60, 0), "", PriorityMode::None, {}},
        {3, Vector2D(60, 60), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 60.0, 10.0, 1, {}, "synth/in", true},
        {2, 2, 3, 60.0, 10.0, 1, {}, "synth/out", true}
    };
    return map;
}

MapDescription makeSyntheticVeryShort() {
    MapDescription map;
    map.name = "synth_very_short";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(3, 0), "", PriorityMode::None, {}},
        {3, Vector2D(6, 0), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 3.0, 8.0, 1, {}, "synth/s1", true},
        {2, 2, 3, 3.0, 8.0, 1, {}, "synth/s2", true}
    };
    return map;
}

MapDescription makeSyntheticTJunction() {
    MapDescription map;
    map.name = "synth_t_junction";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(50, 0), "", PriorityMode::None, {}},
        {3, Vector2D(100, 0), "", PriorityMode::None, {}},
        {4, Vector2D(50, 50), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 50.0, 10.0, 1, {}, "", true},
        {2, 2, 1, 50.0, 10.0, 1, {}, "", true},
        {3, 2, 3, 50.0, 10.0, 1, {}, "", true},
        {4, 3, 2, 50.0, 10.0, 1, {}, "", true},
        {5, 4, 2, 50.0, 10.0, 1, {}, "", true},
        {6, 2, 4, 50.0, 10.0, 1, {}, "", true}
    };
    return map;
}

MapDescription makeSyntheticFourWay() {
    MapDescription map;
    map.name = "synth_four_way";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(50, 0), "", PriorityMode::None, {}},
        {3, Vector2D(100, 0), "", PriorityMode::None, {}},
        {4, Vector2D(50, 50), "", PriorityMode::None, {}},
        {5, Vector2D(50, -50), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 50, 12, 1, {}, "", true}, {2, 2, 1, 50, 12, 1, {}, "", true},
        {3, 2, 3, 50, 12, 1, {}, "", true}, {4, 3, 2, 50, 12, 1, {}, "", true},
        {5, 4, 2, 50, 12, 1, {}, "", true}, {6, 2, 4, 50, 12, 1, {}, "", true},
        {7, 5, 2, 50, 12, 1, {}, "", true}, {8, 2, 5, 50, 12, 1, {}, "", true}
    };
    return map;
}

MapDescription makeSyntheticForkMerge() {
    MapDescription map;
    map.name = "synth_fork_merge";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(40, 0), "", PriorityMode::None, {}},
        {3, Vector2D(80, 20), "", PriorityMode::None, {}},
        {4, Vector2D(80, -20), "", PriorityMode::None, {}},
        {5, Vector2D(120, 0), "", PriorityMode::None, {}},
        {6, Vector2D(160, 0), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 40, 10, 1, {}, "", true},
        {2, 2, 3, sqrt(40.0 * 40.0 + 20.0 * 20.0), 10, 1, {}, "", true},
        {3, 2, 4, sqrt(40.0 * 40.0 + 20.0 * 20.0), 10, 1, {}, "", true},
        {4, 3, 5, sqrt(40.0 * 40.0 + 20.0 * 20.0), 10, 1, {}, "", true},
        {5, 4, 5, sqrt(40.0 * 40.0 + 20.0 * 20.0), 10, 1, {}, "", true},
        {6, 5, 6, 40, 10, 1, {}, "", true}
    };
    return map;
}

MapDescription makeSyntheticLaneChange2to1() {
    MapDescription map;
    map.name = "synth_lane_count_change";
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(80, 0), "", PriorityMode::None, {}},
        {3, Vector2D(160, 0), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 80, 12, 2, {}, "synth/2lane", true},
        {2, 2, 3, 80, 12, 1, {}, "synth/1lane", true}
    };
    return map;
}

MapDescription makeSyntheticLanes(int forward, int backward, const string& name) {
    MapDescription map;
    map.name = name;
    map.intersections = {
        {1, Vector2D(0, 0), "", PriorityMode::None, {}},
        {2, Vector2D(120, 0), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 120, 13.9, forward, {}, name + "/fwd", true},
        {2, 2, 1, 120, 13.9, backward, {}, name + "/bwd", true}
    };
    return map;
}

MapDescription makeSyntheticDualCarriage() {
    MapDescription map;
    map.name = "synth_dual_carriage";
    map.intersections = {
        {1, Vector2D(0, 4), "", PriorityMode::None, {}},
        {2, Vector2D(120, 4), "", PriorityMode::None, {}},
        {3, Vector2D(120, -4), "", PriorityMode::None, {}},
        {4, Vector2D(0, -4), "", PriorityMode::None, {}}
    };
    map.roads = {
        {1, 1, 2, 120, 16, 2, {}, "eb", true},
        {2, 3, 4, 120, 16, 2, {}, "wb", true}
    };
    return map;
}

MapDescription makeSyntheticCrossingIsland() {
    MapDescription map = makeSyntheticDualCarriage();
    map.name = "synth_crossing_island";
    MapCrossingDesc c1{1, 1, 60.0, 18.0, 4.0, false};
    MapCrossingDesc c2{2, 2, 60.0, 18.0, 4.0, false};
    map.crossings = {c1, c2};
    return map;
}

void appendDefects(ostream& out, const string& layer, const vector<string>& items) {
    out << "  [" << layer << "] count=" << items.size() << "\n";
    for (const auto& d : items) {
        out << "    - " << d << "\n";
    }
}

void writeMetrics(ostream& out, const GeometryMetrics& m) {
    out << "\n## " << m.caseName << "\n";
    out << "source=" << m.sourcePath << "\n";
    out << "convert_ok=" << (m.convertOk ? 1 : 0) << " city_ok=" << (m.cityOk ? 1 : 0) << "\n";
    if (!m.error.empty()) {
        out << "error=" << m.error << "\n";
    }
    out << "topology: intersections=" << m.intersections
        << " roads=" << m.roads
        << " crossings=" << m.crossings
        << " disallowed_turns=" << m.disallowedTurns
        << " max_degree=" << m.maxNodeDegree
        << " dead_ends=" << m.deadEndRoads
        << " components=" << m.weaklyConnectedComponents << "\n";
    out << "physical: min_len=" << m.minLengthM
        << " max_len=" << m.maxLengthM
        << " max_|len-chord|=" << m.maxAbsLengthMinusChordM
        << " max_|len-poly|=" << m.maxAbsLengthMinusPolylineM
        << " with_intermediates=" << m.roadsWithIntermediates
        << " very_short_lt5m=" << m.veryShortRoadsLt5m
        << " poly_mismatch=" << m.polylineMismatchBeyondTolerance << "\n";
    out << "lanes: multi_sampled=" << m.multiLaneRoadsSampled
        << " min_sep=" << m.minLaneSeparationM
        << " max_sep_err=" << m.maxLaneSeparationErrorM
        << " near_overlap_pairs=" << m.oppositePairNearOverlap << "\n";
    out << "turns: geometric=" << m.geometricTurnPairs
        << " allowed=" << m.allowedTurnPairs
        << " same_seg_geom_uturn=" << m.uTurnSameSegment << "\n";
    out << "trajectory: ran=" << (m.trajectoryProbeRan ? 1 : 0)
        << " max_step_jump_m=" << m.maxStepJumpM
        << " overrun_ticks=" << m.positionOverrunTicks
        << " completed=" << m.completedProbeVehicles << "\n";
    out << "render: roads=" << m.renderRoads
        << " empty_centerlines=" << m.emptyCenterlines
        << " max_render_motion_delta_m=" << m.maxRenderMotionDeltaM << "\n";
    appendDefects(out, "OSM topology", m.defects.osmTopology);
    appendDefects(out, "physical geometry", m.defects.physicalGeometry);
    appendDefects(out, "lane paths", m.defects.lanePaths);
    appendDefects(out, "turn connections", m.defects.turnConnections);
    appendDefects(out, "vehicle trajectories", m.defects.vehicleTrajectories);
    appendDefects(out, "render/LOD", m.defects.renderLod);
}

void softExpectMeasured(const GeometryMetrics& m, const string& tag) {
    expect(m.convertOk, tag + "_convert_ok");
    expect(m.cityOk, tag + "_city_ok");
    expect(m.roads > 0, tag + "_has_roads");
}

} // namespace

int runGeometryControlTests() {
    g_failures = 0;
    g_passed = 0;
    ensureOutputDir();
    cout << "\n=== Geometry control cases (Etap 1) ===\n";

    vector<GeometryMetrics> all;

    auto addSynth = [&](const MapDescription& map) {
        all.push_back(measureMap(map.name, map, "synthetic://MapDescription"));
        softExpectMeasured(all.back(), map.name);
    };

    addSynth(makeSyntheticStraight1Plus1());
    addSynth(makeSyntheticGentleCurve());
    addSynth(makeSyntheticSharpTurn());
    addSynth(makeSyntheticVeryShort());
    addSynth(makeSyntheticTJunction());
    addSynth(makeSyntheticFourWay());
    addSynth(makeSyntheticForkMerge());
    addSynth(makeSyntheticLaneChange2to1());
    addSynth(makeSyntheticLanes(1, 1, "synth_layout_1_1"));
    addSynth(makeSyntheticLanes(2, 1, "synth_layout_2_1"));
    addSynth(makeSyntheticLanes(2, 2, "synth_layout_2_2"));
    addSynth(makeSyntheticDualCarriage());
    addSynth(makeSyntheticCrossingIsland());

    const vector<pair<string, string>> osmCases = {
        {"osm_straight_oneway", fixturePath("straight_oneway.overpass.json")},
        {"osm_gentle_curve", fixturePath("gentle_curve.overpass.json")},
        {"osm_sharp_turn", fixturePath("sharp_turn.overpass.json")},
        {"osm_very_short", fixturePath("very_short_segment.overpass.json")},
        {"osm_t_junction", fixturePath("t_junction.overpass.json")},
        {"osm_four_way", fixturePath("four_way.overpass.json")},
        {"osm_fork_merge", fixturePath("fork_merge.overpass.json")},
        {"osm_lanes_2_1", fixturePath("lanes_2_1.overpass.json")},
        {"osm_lanes_2_2", fixturePath("lanes_2_2.overpass.json")},
        {"osm_dual_carriageway", fixturePath("dual_carriageway.overpass.json")},
        {"osm_crossing_island", fixturePath("crossing_island.overpass.json")},
        {"osm_existing_curved", existingOsmFixture("curved_polyline.overpass.json")},
        {"osm_existing_shared_junction", existingOsmFixture("shared_junction.overpass.json")},
        {"osm_existing_directional_lanes", existingOsmFixture("directional_lanes.overpass.json")},
        {"osm_existing_roundabout", existingOsmFixture("roundabout.overpass.json")},
        {"osm_warsaw_sample", "data/osm/warsaw_sample.overpass.json"},
        {"osm_warsaw_1km", "data/osm/warsaw_1km.overpass.json"}
    };

    for (const auto& c : osmCases) {
        if (!fs::exists(c.second)) {
            cout << "SKIP  " << c.first << " missing " << c.second << "\n";
            continue;
        }
        all.push_back(measureOsmFile(c.first, c.second));
        softExpectMeasured(all.back(), c.first);
    }

    // Lane layout expectations on synthetic 2+2.
    for (const auto& m : all) {
        if (m.caseName == "synth_layout_2_2" && m.cityOk) {
            expect(m.multiLaneRoadsSampled == 2, "synth_2_2_multi_lane_count");
            expect(fabs(m.minLaneSeparationM - SimConstants::LANE_WIDTH) < 0.05, "synth_2_2_lane_sep");
        }
        if (m.caseName == "synth_very_short" && m.cityOk) {
            expect(m.veryShortRoadsLt5m >= 2, "synth_very_short_flagged");
        }
        if (m.caseName == "synth_crossing_island" && m.cityOk) {
            expect(m.crossings == 2, "synth_island_two_crossings");
        }
        if (m.caseName == "osm_lanes_2_1" && m.cityOk) {
            // Expect forward 2 + backward 1 directed edges.
            expect(m.roads == 2, "osm_2_1_two_directed");
        }
    }

    ofstream report("tests/output/etap1_geometry_baseline.txt");
    report << "Etap 1 geometry control metrics\n";
    report << "cases=" << all.size() << "\n";
    for (const auto& m : all) {
        writeMetrics(report, m);
        cout << "GEO  " << m.caseName
             << " roads=" << m.roads
             << " short=" << m.veryShortRoadsLt5m
             << " |len-chord|_max=" << m.maxAbsLengthMinusChordM
             << " jump_max=" << m.maxStepJumpM
             << " defects_phys=" << m.defects.physicalGeometry.size()
             << " defects_lane=" << m.defects.lanePaths.size()
             << "\n";
    }
    report.close();

    // Aggregate defect rollup for store handoff.
    ofstream defects("tests/output/etap1_geometry_defects.txt");
    defects << "Etap 1 geometry defects by layer\n";
    for (const auto& m : all) {
        const size_t n = m.defects.osmTopology.size() + m.defects.physicalGeometry.size() +
            m.defects.lanePaths.size() + m.defects.turnConnections.size() +
            m.defects.vehicleTrajectories.size() + m.defects.renderLod.size();
        if (n == 0) {
            continue;
        }
        writeMetrics(defects, m);
    }
    defects.close();

    return g_failures;
}

int geometryControlPassedCount() {
    return g_passed;
}
