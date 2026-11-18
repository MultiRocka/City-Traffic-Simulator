#include "osm/MapGeometryContinuity.h"

#include "osm/OsmToMapDescription.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

using namespace std;

namespace {

void pushUnique(vector<string>& list, const string& note) {
    if (note.empty()) {
        return;
    }
    if (find(list.begin(), list.end(), note) == list.end()) {
        list.push_back(note);
    }
}

int priorityRank(PriorityMode mode) {
    switch (mode) {
    case PriorityMode::Signalized:
        return 3;
    case PriorityMode::PriorityRoad:
        return 2;
    case PriorityMode::YieldToRight:
        return 1;
    case PriorityMode::None:
    default:
        return 0;
    }
}

void absorbPriority(MapIntersectionDesc& survivor, const MapIntersectionDesc& absorbed) {
    if (priorityRank(absorbed.priorityMode) > priorityRank(survivor.priorityMode)) {
        survivor.priorityMode = absorbed.priorityMode;
        survivor.priorityRoadIds = absorbed.priorityRoadIds;
    } else if (absorbed.priorityMode == survivor.priorityMode &&
               survivor.priorityMode == PriorityMode::PriorityRoad) {
        for (int id : absorbed.priorityRoadIds) {
            if (find(survivor.priorityRoadIds.begin(), survivor.priorityRoadIds.end(), id) ==
                survivor.priorityRoadIds.end()) {
                survivor.priorityRoadIds.push_back(id);
            }
        }
    }
}

MapIntersectionDesc* findIntersection(MapDescription& map, int id) {
    for (auto& node : map.intersections) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

const MapIntersectionDesc* findIntersectionConst(const MapDescription& map, int id) {
    for (const auto& node : map.intersections) {
        if (node.id == id) {
            return &node;
        }
    }
    return nullptr;
}

MapRoadDesc* findRoad(MapDescription& map, int id) {
    for (auto& road : map.roads) {
        if (road.id == id) {
            return &road;
        }
    }
    return nullptr;
}

int nextRoadId(const MapDescription& map) {
    int maxId = 0;
    for (const auto& road : map.roads) {
        maxId = max(maxId, road.id);
    }
    return maxId + 1;
}

unordered_map<int, int> undirectedDegree(const MapDescription& map) {
    unordered_map<int, set<int>> neighbors;
    for (const auto& road : map.roads) {
        neighbors[road.fromIntersectionId].insert(road.toIntersectionId);
        neighbors[road.toIntersectionId].insert(road.fromIntersectionId);
    }
    unordered_map<int, int> degree;
    for (const auto& node : map.intersections) {
        const auto it = neighbors.find(node.id);
        degree[node.id] = it == neighbors.end() ? 0 : static_cast<int>(it->second.size());
    }
    return degree;
}

vector<Vector2D> directedPolyline(
    const MapDescription& map,
    const MapRoadDesc& road
) {
    vector<Vector2D> poly;
    const auto* from = findIntersectionConst(map, road.fromIntersectionId);
    const auto* to = findIntersectionConst(map, road.toIntersectionId);
    if (from == nullptr || to == nullptr) {
        return poly;
    }
    poly.push_back(from->position);
    poly.insert(poly.end(), road.intermediatePoints.begin(), road.intermediatePoints.end());
    poly.push_back(to->position);
    return poly;
}

MapRoadDesc mergeDirectedPair(
    const MapDescription& map,
    const MapRoadDesc& first,
    const MapRoadDesc& second,
    int newId
) {
    MapRoadDesc out;
    out.id = newId;
    out.fromIntersectionId = first.fromIntersectionId;
    out.toIntersectionId = second.toIntersectionId;
    out.length = first.length + second.length;
    // Keep continuity attributes from the longer piece when they disagree.
    if (second.length >= first.length) {
        out.speedLimit = second.speedLimit;
        out.laneCount = second.laneCount;
    } else {
        out.speedLimit = first.speedLimit;
        out.laneCount = first.laneCount;
    }
    out.allowAutoDemand = first.allowAutoDemand && second.allowAutoDemand;
    out.sourceRef = first.sourceRef + "+" + second.sourceRef;

    const auto* mid = findIntersectionConst(map, first.toIntersectionId);
    out.intermediatePoints = first.intermediatePoints;
    if (mid != nullptr) {
        out.intermediatePoints.push_back(mid->position);
    }
    out.intermediatePoints.insert(
        out.intermediatePoints.end(),
        second.intermediatePoints.begin(),
        second.intermediatePoints.end()
    );

    // Authoritative length must match polyline when intermediates exist.
    vector<Vector2D> poly = directedPolyline(map, first);
    // Rebuild from merged geometry endpoints for length authority check.
    const auto* from = findIntersectionConst(map, out.fromIntersectionId);
    const auto* to = findIntersectionConst(map, out.toIntersectionId);
    if (from != nullptr && to != nullptr) {
        vector<Vector2D> full;
        full.push_back(from->position);
        full.insert(full.end(), out.intermediatePoints.begin(), out.intermediatePoints.end());
        full.push_back(to->position);
        out.length = OsmToMapDescription::polylineLengthMeters(full);
    }
    (void)poly;
    return out;
}

void remapRoadId(MapDescription& map, int oldId, int newId) {
    if (oldId == newId) {
        return;
    }
    for (auto& crossing : map.crossings) {
        if (crossing.roadSegmentId == oldId) {
            crossing.roadSegmentId = newId;
        }
    }
    for (auto& conn : map.connections) {
        if (conn.fromRoadId == oldId) {
            conn.fromRoadId = newId;
        }
        if (conn.toRoadId == oldId) {
            conn.toRoadId = newId;
        }
    }
    for (auto& node : map.intersections) {
        for (int& pid : node.priorityRoadIds) {
            if (pid == oldId) {
                pid = newId;
            }
        }
    }
}

void eraseRoads(MapDescription& map, const unordered_set<int>& ids) {
    map.roads.erase(
        remove_if(
            map.roads.begin(),
            map.roads.end(),
            [&](const MapRoadDesc& road) { return ids.count(road.id) > 0; }
        ),
        map.roads.end()
    );
}

void eraseIntersection(MapDescription& map, int id) {
    map.intersections.erase(
        remove_if(
            map.intersections.begin(),
            map.intersections.end(),
            [&](const MapIntersectionDesc& node) { return node.id == id; }
        ),
        map.intersections.end()
    );
}

void shiftCrossingsOntoMerged(
    MapDescription& map,
    int oldFirstId,
    double firstLength,
    int oldSecondId,
    int newId
) {
    for (auto& crossing : map.crossings) {
        if (crossing.roadSegmentId == oldFirstId) {
            crossing.roadSegmentId = newId;
            // position already in [0, firstLength]
        } else if (crossing.roadSegmentId == oldSecondId) {
            crossing.roadSegmentId = newId;
            crossing.positionOnRoad = firstLength + crossing.positionOnRoad;
        }
    }
}

bool tryMergeOneStub(MapDescription& map, double minKeepLengthM, MapGeometryContinuityStats& stats) {
    const auto degree = undirectedDegree(map);

    for (const auto& road : map.roads) {
        if (road.length >= minKeepLengthM) {
            continue;
        }

        const int fromDeg = degree.count(road.fromIntersectionId) ? degree.at(road.fromIntersectionId) : 0;
        const int toDeg = degree.count(road.toIntersectionId) ? degree.at(road.toIntersectionId) : 0;

        int midId = -1;
        int farId = -1;
        if (toDeg == 2) {
            midId = road.toIntersectionId;
            farId = road.fromIntersectionId;
        } else if (fromDeg == 2) {
            midId = road.fromIntersectionId;
            farId = road.toIntersectionId;
        } else {
            continue; // both ends are real forks — keep the short link honest
        }

        // Neighbor of mid other than far.
        set<int> midNeighbors;
        for (const auto& other : map.roads) {
            if (other.fromIntersectionId == midId) {
                midNeighbors.insert(other.toIntersectionId);
            }
            if (other.toIntersectionId == midId) {
                midNeighbors.insert(other.fromIntersectionId);
            }
        }
        midNeighbors.erase(farId);
        if (midNeighbors.size() != 1) {
            continue;
        }
        const int otherId = *midNeighbors.begin();

        // Collect directed roads on undirected edges {far,mid} and {mid,other}.
        vector<int> edgeFarMid;
        vector<int> edgeMidOther;
        for (const auto& other : map.roads) {
            const bool onFarMid =
                (other.fromIntersectionId == farId && other.toIntersectionId == midId) ||
                (other.fromIntersectionId == midId && other.toIntersectionId == farId);
            const bool onMidOther =
                (other.fromIntersectionId == midId && other.toIntersectionId == otherId) ||
                (other.fromIntersectionId == otherId && other.toIntersectionId == midId);
            if (onFarMid) {
                edgeFarMid.push_back(other.id);
            }
            if (onMidOther) {
                edgeMidOther.push_back(other.id);
            }
        }
        if (edgeFarMid.empty() || edgeMidOther.empty()) {
            continue;
        }

        // Require at least one of the undirected edges to be short (already true for `road`).
        auto roadById = [&](int id) -> const MapRoadDesc* {
            for (const auto& r : map.roads) {
                if (r.id == id) {
                    return &r;
                }
            }
            return nullptr;
        };

        double farMidLen = 0.0;
        for (int id : edgeFarMid) {
            if (const auto* r = roadById(id)) {
                farMidLen = max(farMidLen, r->length);
            }
        }
        double midOtherLen = 0.0;
        for (int id : edgeMidOther) {
            if (const auto* r = roadById(id)) {
                midOtherLen = max(midOtherLen, r->length);
            }
        }
        if (farMidLen >= minKeepLengthM && midOtherLen >= minKeepLengthM) {
            continue;
        }

        // Build merged directed roads where a through path exists.
        struct PendingMerge {
            int firstId = -1;
            int secondId = -1;
            MapRoadDesc merged;
            double firstLength = 0.0;
        };
        vector<PendingMerge> pending;
        unordered_set<int> consumed;

        auto tryPair = [&](int aId, int bId) {
            const MapRoadDesc* a = roadById(aId);
            const MapRoadDesc* b = roadById(bId);
            if (a == nullptr || b == nullptr) {
                return;
            }
            if (a->toIntersectionId != b->fromIntersectionId) {
                return;
            }
            if (a->toIntersectionId != midId) {
                return;
            }
            if (consumed.count(aId) || consumed.count(bId)) {
                return;
            }
            PendingMerge pm;
            pm.firstId = aId;
            pm.secondId = bId;
            pm.firstLength = a->length;
            pm.merged = mergeDirectedPair(map, *a, *b, -1);
            pending.push_back(pm);
            consumed.insert(aId);
            consumed.insert(bId);
        };

        for (int aId : edgeFarMid) {
            for (int bId : edgeMidOther) {
                tryPair(aId, bId);
            }
        }
        for (int aId : edgeMidOther) {
            for (int bId : edgeFarMid) {
                tryPair(aId, bId);
            }
        }

        if (pending.empty()) {
            continue;
        }

        // Assign stable new ids now that pending size is known.
        int idCursor = nextRoadId(map);
        for (auto& pm : pending) {
            pm.merged.id = idCursor++;
        }

        MapIntersectionDesc* midNode = findIntersection(map, midId);
        MapIntersectionDesc* farNode = findIntersection(map, farId);
        MapIntersectionDesc* otherNode = findIntersection(map, otherId);
        if (midNode == nullptr || farNode == nullptr || otherNode == nullptr) {
            continue;
        }

        // Prefer transferring soft-junction priority onto the higher-degree survivor
        // (usually the real fork next to a signal stub).
        MapIntersectionDesc* survivor =
            (degree.at(farId) >= degree.at(otherId)) ? farNode : otherNode;
        absorbPriority(*survivor, *midNode);

        for (auto& pm : pending) {
            shiftCrossingsOntoMerged(map, pm.firstId, pm.firstLength, pm.secondId, pm.merged.id);
            remapRoadId(map, pm.firstId, pm.merged.id);
            remapRoadId(map, pm.secondId, pm.merged.id);
            map.roads.push_back(pm.merged);
            ++stats.mergedDirectedRoads;
        }

        eraseRoads(map, consumed);
        eraseIntersection(map, midId);
        ++stats.removedIntersections;

        // Drop degenerate self-connections created by remaps.
        map.connections.erase(
            remove_if(
                map.connections.begin(),
                map.connections.end(),
                [](const MapConnectionDesc& c) {
                    return c.fromRoadId == c.toRoadId || c.fromRoadId < 0 || c.toRoadId < 0;
                }
            ),
            map.connections.end()
        );

        // Clamp crossing positions onto their roads.
        for (auto& crossing : map.crossings) {
            if (const MapRoadDesc* r = findRoad(map, crossing.roadSegmentId)) {
                crossing.positionOnRoad = min(r->length, max(0.0, crossing.positionOnRoad));
            }
        }

        return true;
    }
    return false;
}

} // namespace

MapGeometryContinuityStats MapGeometryContinuity::mergeShortDegree2Stubs(
    MapDescription& map,
    ImportReport* report,
    double minKeepLengthM
) {
    MapGeometryContinuityStats stats;
    const int beforeShort = [&]() {
        int n = 0;
        for (const auto& road : map.roads) {
            if (road.length < minKeepLengthM) {
                ++n;
            }
        }
        return n;
    }();

    // Bound iterations: each success removes at least one intersection.
    const size_t maxPasses = map.intersections.size() + 8;
    for (size_t pass = 0; pass < maxPasses; ++pass) {
        if (!tryMergeOneStub(map, minKeepLengthM, stats)) {
            break;
        }
    }

    for (const auto& road : map.roads) {
        if (road.length < minKeepLengthM) {
            ++stats.remainingVeryShortLt5m;
        }
    }

    if (report != nullptr && (stats.mergedDirectedRoads > 0 || stats.removedIntersections > 0)) {
        ostringstream note;
        note << "Etap2 continuity: merged short degree-2 stubs; removed_intersections="
             << stats.removedIntersections
             << " short_before=" << beforeShort
             << " short_after=" << stats.remainingVeryShortLt5m
             << " (no dual-carriage proximity joins)";
        pushUnique(report->inferred, note.str());
        report->warnings.push_back(note.str());
    }

    return stats;
}
