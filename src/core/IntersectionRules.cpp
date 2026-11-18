#include "core/IntersectionRules.h"

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;

namespace {

double vectorLength(const Vector2D& v) {
    return sqrt(v.x * v.x + v.y * v.y);
}

Vector2D normalizeOrZero(const Vector2D& v) {
    const double len = vectorLength(v);
    if (len < 1e-9) {
        return Vector2D(0.0, 0.0);
    }
    return Vector2D(v.x / len, v.y / len);
}

double dot2(const Vector2D& a, const Vector2D& b) {
    return a.x * b.x + a.y * b.y;
}

double cross2(const Vector2D& a, const Vector2D& b) {
    return a.x * b.y - a.y * b.x;
}

// Rotate 90° counter-clockwise (inbound that comes from ego's right under RHT).
Vector2D rotate90Ccw(const Vector2D& v) {
    return Vector2D(-v.y, v.x);
}

Vector2D rotate90Cw(const Vector2D& v) {
    return Vector2D(v.y, -v.x);
}

double approachBearing(const Vector2D& inbound) {
    return atan2(inbound.y, inbound.x);
}

} // namespace

void IntersectionRules::clear() {
    reservations.clear();
    waitStartedAt.clear();
}

void IntersectionRules::clearStaleReservations(const City& city, const vector<Vehicle>& vehicles) {
    for (auto& entry : reservations) {
        vector<IntersectionReservation> kept;
        for (const auto& reservation : entry.second) {
            if (!reservation.isActive()) {
                continue;
            }

            const Vehicle* vehicle = findVehicle(vehicles, reservation.vehicleId);
            if (vehicle == nullptr || vehicle->isFinished()) {
                continue;
            }

            const int roadId = vehicle->getCurrentRoadSegmentId();
            if (roadId == reservation.toRoadId) {
                if (vehicle->getPositionOnRoad() >= TrafficPolicy::INTERSECTION_CLEARANCE_M) {
                    continue; // cleared beyond junction
                }
                kept.push_back(reservation);
                continue;
            }

            // Still on approach (should be rare) or left entirely → drop if not on exit.
            if (roadId == reservation.fromRoadId) {
                kept.push_back(reservation);
            }
        }
        entry.second = kept;
    }

    // Drop empty intersection buckets.
    for (auto it = reservations.begin(); it != reservations.end(); ) {
        if (it->second.empty()) {
            it = reservations.erase(it);
        } else {
            ++it;
        }
    }

    vector<int> staleWaits;
    for (const auto& entry : waitStartedAt) {
        if (findVehicle(vehicles, entry.first) == nullptr) {
            staleWaits.push_back(entry.first);
        }
    }
    for (int vehicleId : staleWaits) {
        waitStartedAt.erase(vehicleId);
    }

    (void)city;
}

int IntersectionRules::suggestEntryLane(
    int fromLaneIndex,
    int fromLaneCount,
    int toLaneCount,
    TurnType turn
) {
    fromLaneCount = max(1, fromLaneCount);
    toLaneCount = max(1, toLaneCount);
    fromLaneIndex = max(0, min(fromLaneIndex, fromLaneCount - 1));
    if (toLaneCount == 1) {
        return 0;
    }

    switch (turn) {
    case TurnType::Right:
        return 0;
    case TurnType::Left:
        return toLaneCount - 1;
    case TurnType::Straight:
    default:
        if (fromLaneCount == 1) {
            return (toLaneCount - 1) / 2;
        }
        return (fromLaneIndex * (toLaneCount - 1) + (fromLaneCount - 1) / 2) / (fromLaneCount - 1);
    }
}

TurnType IntersectionRules::classifyTurn(const City& city, int fromRoadId, int toRoadId) const {
    const Vector2D inbound = roadInboundDirection(city, fromRoadId);
    const Vector2D outbound = roadOutboundDirection(city, toRoadId);
    const Vector2D inDir = normalizeOrZero(inbound);
    const Vector2D outDir = normalizeOrZero(outbound);

    if (vectorLength(inDir) < 1e-9 || vectorLength(outDir) < 1e-9) {
        return TurnType::Straight;
    }

    const double cosAngle = dot2(inDir, outDir);
    if (cosAngle >= TrafficPolicy::STRAIGHT_COS_THRESHOLD) {
        return TurnType::Straight;
    }

    const double sinAngle = cross2(inDir, outDir);
    // Standard plane: positive cross = CCW = left turn when driving forward.
    if (TrafficPolicy::RIGHT_HAND_TRAFFIC) {
        return sinAngle > 0.0 ? TurnType::Left : TurnType::Right;
    }
    return sinAngle > 0.0 ? TurnType::Right : TurnType::Left;
}

MovementKey IntersectionRules::makeMovement(const City& city, const Vehicle& vehicle) const {
    MovementKey key;
    const int fromRoadId = vehicle.getCurrentRoadSegmentId();
    const int toRoadId = vehicle.getNextRoadSegmentId();
    const RoadSegment* fromRoad = city.findRoadSegmentById(fromRoadId);
    if (fromRoad == nullptr) {
        return key;
    }

    key.intersectionId = fromRoad->getToIntersectionId();
    key.fromRoadId = fromRoadId;
    key.toRoadId = toRoadId;
    if (toRoadId != -1) {
        key.turn = classifyTurn(city, fromRoadId, toRoadId);
    }
    return key;
}

PriorityMode IntersectionRules::effectivePriorityMode(const Intersection& intersection) const {
    // Requirement: unsignalized None → YieldToRight.
    if (intersection.getPriorityMode() == PriorityMode::None) {
        return PriorityMode::YieldToRight;
    }
    return intersection.getPriorityMode();
}

bool IntersectionRules::isPriorityApproach(const Intersection& intersection, int fromRoadId) const {
    const vector<int>& priorityIds = intersection.getPriorityRoadIds();
    return find(priorityIds.begin(), priorityIds.end(), fromRoadId) != priorityIds.end();
}

bool IntersectionRules::movementsConflict(const City& city, const MovementKey& a, const MovementKey& b) const {
    if (a.intersectionId < 0 || a.intersectionId != b.intersectionId) {
        return false;
    }
    if (a.fromRoadId == b.fromRoadId) {
        return false; // same approach — car-following, not junction conflict
    }
    if (a.toRoadId < 0 || b.toRoadId < 0) {
        return true;
    }

    const Vector2D inA = normalizeOrZero(roadInboundDirection(city, a.fromRoadId));
    const Vector2D inB = normalizeOrZero(roadInboundDirection(city, b.fromRoadId));
    const double inboundDot = dot2(inA, inB);

    // Opposite straights share the corridor without crossing paths.
    if (a.turn == TurnType::Straight &&
        b.turn == TurnType::Straight &&
        inboundDot <= TrafficPolicy::OPPOSITE_INBOUND_COS_THRESHOLD) {
        return false;
    }

    // Merge into the same exit from different approaches always conflicts.
    if (a.toRoadId == b.toRoadId) {
        return true;
    }

    const Intersection* intersection = city.findIntersectionById(a.intersectionId);
    if (intersection == nullptr) {
        return true;
    }

    // Movement chords through the junction center (short stubs along inbound/outbound).
    const Vector2D center = intersection->getPosition();
    const Vector2D outA = normalizeOrZero(roadOutboundDirection(city, a.toRoadId));
    const Vector2D outB = normalizeOrZero(roadOutboundDirection(city, b.toRoadId));
    constexpr double stub = 5.0;
    const Vector2D a1(center.x - inA.x * stub, center.y - inA.y * stub);
    const Vector2D a2(center.x + outA.x * stub, center.y + outA.y * stub);
    const Vector2D b1(center.x - inB.x * stub, center.y - inB.y * stub);
    const Vector2D b2(center.x + outB.x * stub, center.y + outB.y * stub);

    if (segmentIntersect(a1, a2, b1, b2)) {
        return true;
    }

    // Left vs opposing straight / left often fails proper-segment intersect at the shared center;
    // treat left turns as conflicting with any non-compatible opposing movement.
    if (a.turn == TurnType::Left || b.turn == TurnType::Left) {
        // Right turns that stay outside a left stock path: if chords don't meet and
        // inbounds are nearly perpendicular with the right-turner on the inside, allow.
        if (a.turn == TurnType::Right || b.turn == TurnType::Right) {
            // Left vs right from adjacent approaches: conflict if paths cross geometrically
            // (already handled) or share a merge; otherwise often compatible.
            return false;
        }
        return true;
    }

    return false;
}

bool IntersectionRules::isFromRight(const City& city, const MovementKey& ego, const MovementKey& other) const {
    const Vector2D egoIn = normalizeOrZero(roadInboundDirection(city, ego.fromRoadId));
    const Vector2D otherIn = normalizeOrZero(roadInboundDirection(city, other.fromRoadId));
    if (vectorLength(egoIn) < 1e-9 || vectorLength(otherIn) < 1e-9) {
        return false;
    }

    // Under RHT, traffic from the right arrives with inbound ≈ rotate90CCW(ego inbound).
    // Under LHT, mirror with CW.
    const Vector2D expected = TrafficPolicy::RIGHT_HAND_TRAFFIC
        ? normalizeOrZero(rotate90Ccw(egoIn))
        : normalizeOrZero(rotate90Cw(egoIn));

    return dot2(otherIn, expected) >= TrafficPolicy::FROM_RIGHT_COS_THRESHOLD;
}

bool IntersectionRules::hasReservation(int intersectionId) const {
    auto it = reservations.find(intersectionId);
    return it != reservations.end() && !it->second.empty();
}

const IntersectionReservation* IntersectionRules::getReservation(int intersectionId) const {
    auto it = reservations.find(intersectionId);
    if (it == reservations.end() || it->second.empty()) {
        return nullptr;
    }
    return &it->second.front();
}

const vector<IntersectionReservation>* IntersectionRules::getReservationsFor(int intersectionId) const {
    auto it = reservations.find(intersectionId);
    if (it == reservations.end()) {
        return nullptr;
    }
    return &it->second;
}

const map<int, vector<IntersectionReservation>>& IntersectionRules::getReservations() const {
    return reservations;
}

void IntersectionRules::reserve(int intersectionId, const Vehicle& vehicle, const MovementKey& movement) {
    // Replace any existing reservation for this vehicle at this junction.
    auto& list = reservations[intersectionId];
    list.erase(
        remove_if(list.begin(), list.end(), [&](const IntersectionReservation& reservation) {
            return reservation.vehicleId == vehicle.getId();
        }),
        list.end()
    );

    IntersectionReservation reservation;
    reservation.vehicleId = vehicle.getId();
    reservation.fromRoadId = movement.fromRoadId;
    reservation.toRoadId = movement.toRoadId;
    reservation.turn = movement.turn;
    list.push_back(reservation);
}

void IntersectionRules::release(int intersectionId) {
    reservations.erase(intersectionId);
}

void IntersectionRules::releaseVehicle(int vehicleId) {
    for (auto& entry : reservations) {
        auto& list = entry.second;
        list.erase(
            remove_if(list.begin(), list.end(), [&](const IntersectionReservation& reservation) {
                return reservation.vehicleId == vehicleId;
            }),
            list.end()
        );
    }
    for (auto it = reservations.begin(); it != reservations.end(); ) {
        if (it->second.empty()) {
            it = reservations.erase(it);
        } else {
            ++it;
        }
    }
    waitStartedAt.erase(vehicleId);
}

void IntersectionRules::noteWaiting(int vehicleId, double currentTime) {
    if (waitStartedAt.find(vehicleId) == waitStartedAt.end()) {
        waitStartedAt[vehicleId] = currentTime;
    }
}

void IntersectionRules::clearWaiting(int vehicleId) {
    waitStartedAt.erase(vehicleId);
}

double IntersectionRules::waitStartTime(int vehicleId) const {
    auto it = waitStartedAt.find(vehicleId);
    if (it == waitStartedAt.end()) {
        return numeric_limits<double>::infinity();
    }
    return it->second;
}

JunctionBlockReason IntersectionRules::evaluateEntry(
    const Vehicle& vehicle,
    const City& city,
    const vector<TrafficLight>& trafficLights,
    const vector<Vehicle>& vehicles,
    bool nextRoadHasSpace
) const {
    const int fromRoadId = vehicle.getCurrentRoadSegmentId();
    const int toRoadId = vehicle.getNextRoadSegmentId();
    if (toRoadId == -1) {
        return JunctionBlockReason::DestinationReached;
    }

    if (!city.areRoadsConnected(fromRoadId, toRoadId)) {
        return JunctionBlockReason::RouteInvalid;
    }

    auto isGreenForRoad = [&](int roadId) {
        for (const auto& light : trafficLights) {
            if (light.getRoadSegmentId() == roadId) {
                return light.isGreen();
            }
        }
        return true; // no light → unsignalized approach
    };

    if (!isGreenForRoad(fromRoadId)) {
        return JunctionBlockReason::RedLight;
    }

    if (!nextRoadHasSpace) {
        return JunctionBlockReason::BlockedNextRoad;
    }

    const RoadSegment* fromRoad = city.findRoadSegmentById(fromRoadId);
    if (fromRoad == nullptr) {
        return JunctionBlockReason::RouteInvalid;
    }

    const int intersectionId = fromRoad->getToIntersectionId();
    const Intersection* intersection = city.findIntersectionById(intersectionId);
    if (intersection == nullptr) {
        return JunctionBlockReason::None;
    }

    const MovementKey ego = makeMovement(city, vehicle);
    const PriorityMode mode = effectivePriorityMode(*intersection);

    auto reservationMovement = [&](const IntersectionReservation& reservation) {
        MovementKey reserved;
        reserved.intersectionId = intersectionId;
        reserved.fromRoadId = reservation.fromRoadId;
        reserved.toRoadId = reservation.toRoadId;
        reserved.turn = reservation.turn;
        return reserved;
    };

    // Active reservations by others with conflicting movements.
    if (const vector<IntersectionReservation>* occupied = getReservationsFor(intersectionId)) {
        for (const auto& reservation : *occupied) {
            if (reservation.vehicleId == vehicle.getId()) {
                continue;
            }
            if (movementsConflict(city, ego, reservationMovement(reservation))) {
                return JunctionBlockReason::IntersectionConflict;
            }
        }
    }

    // Yield / priority against relevant participants (at/near stop or occupying).
    for (const auto& other : vehicles) {
        if (other.getId() == vehicle.getId() || other.isFinished()) {
            continue;
        }

        if (!isRelevantParticipant(other, city, intersectionId)) {
            continue;
        }

        MovementKey otherMovement;
        if (other.getCurrentRoadSegmentId() == ego.fromRoadId) {
            continue;
        }

        const RoadSegment* otherRoad = city.findRoadSegmentById(other.getCurrentRoadSegmentId());
        if (otherRoad != nullptr && otherRoad->getToIntersectionId() == intersectionId) {
            otherMovement = makeMovement(city, other);
        } else {
            bool found = false;
            if (const vector<IntersectionReservation>* occupied = getReservationsFor(intersectionId)) {
                for (const auto& reservation : *occupied) {
                    if (reservation.vehicleId == other.getId()) {
                        otherMovement = reservationMovement(reservation);
                        found = true;
                        break;
                    }
                }
            }
            if (!found) {
                continue;
            }
        }

        if (!movementsConflict(city, ego, otherMovement)) {
            continue;
        }

        if (mustYieldToOther(ego, otherMovement, mode, *intersection, city)) {
            return JunctionBlockReason::YieldPriority;
        }
    }

    return JunctionBlockReason::None;
}

string IntersectionRules::blockReasonToStopReason(JunctionBlockReason reason) {
    switch (reason) {
    case JunctionBlockReason::RedLight:
        return "red_light";
    case JunctionBlockReason::BlockedNextRoad:
        return "blocked_next_road";
    case JunctionBlockReason::YieldPriority:
        return "yield_priority";
    case JunctionBlockReason::IntersectionConflict:
        return "intersection_conflict";
    case JunctionBlockReason::RouteInvalid:
        return "route_invalid";
    case JunctionBlockReason::DestinationReached:
        return "destination_reached";
    case JunctionBlockReason::None:
    default:
        return "blocked_next_road";
    }
}

vector<int> IntersectionRules::resolveGrants(
    int intersectionId,
    const vector<int>& candidateVehicleIds,
    const City& city,
    const vector<TrafficLight>& trafficLights,
    const vector<Vehicle>& vehicles,
    double currentTime
) const {
    (void)currentTime;
    (void)trafficLights;

    struct Candidate {
        int vehicleId = -1;
        MovementKey movement;
        double waitStart = 0.0;
        double bearing = 0.0;
    };

    vector<Candidate> candidates;
    for (int vehicleId : candidateVehicleIds) {
        const Vehicle* vehicle = findVehicle(vehicles, vehicleId);
        if (vehicle == nullptr || vehicle->isFinished()) {
            continue;
        }
        Candidate candidate;
        candidate.vehicleId = vehicleId;
        candidate.movement = makeMovement(city, *vehicle);
        if (candidate.movement.intersectionId != intersectionId) {
            continue;
        }
        candidate.waitStart = waitStartTime(vehicleId);
        if (!isfinite(candidate.waitStart)) {
            candidate.waitStart = currentTime;
        }
        candidate.bearing = approachBearing(
            normalizeOrZero(roadInboundDirection(city, candidate.movement.fromRoadId))
        );
        candidates.push_back(candidate);
    }

    if (candidates.empty()) {
        return {};
    }

    const Intersection* intersection = city.findIntersectionById(intersectionId);
    if (intersection == nullptr) {
        return {};
    }
    const PriorityMode mode = effectivePriorityMode(*intersection);

    auto conflictsExistingReservations = [&](const MovementKey& movement, int vehicleId) -> bool {
        const vector<IntersectionReservation>* occupied = getReservationsFor(intersectionId);
        if (occupied == nullptr) {
            return false;
        }
        for (const auto& reservation : *occupied) {
            if (reservation.vehicleId == vehicleId) {
                continue;
            }
            MovementKey reserved;
            reserved.intersectionId = intersectionId;
            reserved.fromRoadId = reservation.fromRoadId;
            reserved.toRoadId = reservation.toRoadId;
            reserved.turn = reservation.turn;
            if (movementsConflict(city, movement, reserved)) {
                return true;
            }
        }
        return false;
    };

    auto movementForNonCandidate = [&](const Vehicle& otherVehicle, MovementKey& out) -> bool {
        const RoadSegment* otherRoad = city.findRoadSegmentById(otherVehicle.getCurrentRoadSegmentId());
        if (otherRoad != nullptr && otherRoad->getToIntersectionId() == intersectionId) {
            out = makeMovement(city, otherVehicle);
            return out.toRoadId >= 0;
        }
        if (const vector<IntersectionReservation>* occupied = getReservationsFor(intersectionId)) {
            for (const auto& reservation : *occupied) {
                if (reservation.vehicleId == otherVehicle.getId()) {
                    out.intersectionId = intersectionId;
                    out.fromRoadId = reservation.fromRoadId;
                    out.toRoadId = reservation.toRoadId;
                    out.turn = reservation.turn;
                    return true;
                }
            }
        }
        return false;
    };

    // Hard blocks: reservation or yield to someone not in this candidate set (approaching priority).
    // Deadlock break must NEVER override these.
    auto hardBlocked = [&](const Candidate& candidate) -> bool {
        if (conflictsExistingReservations(candidate.movement, candidate.vehicleId)) {
            return true;
        }

        for (const auto& otherVehicle : vehicles) {
            if (otherVehicle.getId() == candidate.vehicleId || otherVehicle.isFinished()) {
                continue;
            }
            if (!isRelevantParticipant(otherVehicle, city, intersectionId)) {
                continue;
            }

            bool isCandidate = false;
            for (const auto& otherCand : candidates) {
                if (otherCand.vehicleId == otherVehicle.getId()) {
                    isCandidate = true;
                    break;
                }
            }
            if (isCandidate) {
                continue;
            }

            MovementKey otherMovement;
            if (!movementForNonCandidate(otherVehicle, otherMovement)) {
                continue;
            }
            if (!movementsConflict(city, candidate.movement, otherMovement)) {
                continue;
            }
            if (mustYieldToOther(candidate.movement, otherMovement, mode, *intersection, city)) {
                return true;
            }
        }

        return false;
    };

    auto peerYieldBlocked = [&](const Candidate& candidate) -> bool {
        for (const auto& other : candidates) {
            if (other.vehicleId == candidate.vehicleId) {
                continue;
            }
            if (!movementsConflict(city, candidate.movement, other.movement)) {
                continue;
            }
            if (mustYieldToOther(candidate.movement, other.movement, mode, *intersection, city)) {
                return true;
            }
        }
        return false;
    };

    vector<Candidate> eligible;
    for (const auto& candidate : candidates) {
        if (!hardBlocked(candidate) && !peerYieldBlocked(candidate)) {
            eligible.push_back(candidate);
        }
    }

    // Circular yield among peers only: earliest wait, then bearing — never vehicleId alone.
    // Do not override hard yield to approaching priority / active occupation.
    if (eligible.empty()) {
        for (const auto& candidate : candidates) {
            if (!hardBlocked(candidate)) {
                eligible.push_back(candidate);
            }
        }
    }

    sort(eligible.begin(), eligible.end(), [](const Candidate& a, const Candidate& b) {
        if (a.waitStart != b.waitStart) {
            return a.waitStart < b.waitStart; // earlier arrival / longer wait first
        }
        if (a.bearing != b.bearing) {
            return a.bearing < b.bearing;
        }
        // Stable geographic tiebreak — not vehicleId as sole priority rule.
        return a.movement.fromRoadId < b.movement.fromRoadId;
    });

    vector<int> grants;
    if (eligible.empty()) {
        return grants;
    }

    // Grant the first; then any further whose movements do not conflict with already granted.
    vector<MovementKey> grantedMovements;
    for (const auto& candidate : eligible) {
        bool conflictsGranted = false;
        for (const auto& granted : grantedMovements) {
            if (movementsConflict(city, candidate.movement, granted)) {
                conflictsGranted = true;
                break;
            }
        }
        if (conflictsGranted) {
            continue;
        }
        grants.push_back(candidate.vehicleId);
        grantedMovements.push_back(candidate.movement);
    }

    return grants;
}

Vector2D IntersectionRules::roadInboundDirection(const City& city, int roadId) const {
    const RoadSegment* road = city.findRoadSegmentById(roadId);
    if (road == nullptr) {
        return Vector2D(0.0, 0.0);
    }
    const Intersection* from = city.findIntersectionById(road->getFromIntersectionId());
    const Intersection* to = city.findIntersectionById(road->getToIntersectionId());
    if (from == nullptr || to == nullptr) {
        return Vector2D(0.0, 0.0);
    }
    // Use the final polyline leg into the junction so curved approaches classify correctly.
    const vector<Vector2D>& mids = road->getIntermediatePoints();
    if (!mids.empty()) {
        const Vector2D& prev = mids.back();
        return Vector2D(to->getPosition().x - prev.x, to->getPosition().y - prev.y);
    }
    return Vector2D(
        to->getPosition().x - from->getPosition().x,
        to->getPosition().y - from->getPosition().y
    );
}

Vector2D IntersectionRules::roadOutboundDirection(const City& city, int roadId) const {
    const RoadSegment* road = city.findRoadSegmentById(roadId);
    if (road == nullptr) {
        return Vector2D(0.0, 0.0);
    }
    const Intersection* from = city.findIntersectionById(road->getFromIntersectionId());
    const Intersection* to = city.findIntersectionById(road->getToIntersectionId());
    if (from == nullptr || to == nullptr) {
        return Vector2D(0.0, 0.0);
    }
    // First polyline leg leaving the junction (not the full chord).
    const vector<Vector2D>& mids = road->getIntermediatePoints();
    if (!mids.empty()) {
        const Vector2D& next = mids.front();
        return Vector2D(next.x - from->getPosition().x, next.y - from->getPosition().y);
    }
    return Vector2D(
        to->getPosition().x - from->getPosition().x,
        to->getPosition().y - from->getPosition().y
    );
}

bool IntersectionRules::segmentIntersect(
    const Vector2D& a1,
    const Vector2D& a2,
    const Vector2D& b1,
    const Vector2D& b2
) const {
    const Vector2D r(a2.x - a1.x, a2.y - a1.y);
    const Vector2D s(b2.x - b1.x, b2.y - b1.y);
    const double rxs = cross2(r, s);
    const Vector2D qp(b1.x - a1.x, b1.y - a1.y);

    if (abs(rxs) < 1e-9) {
        return false; // parallel / colinear — treat as non-crossing
    }

    const double t = cross2(qp, s) / rxs;
    const double u = cross2(qp, r) / rxs;
    // Proper intersection in the open segment (exclude shared endpoints at center noise).
    return t > 0.02 && t < 0.98 && u > 0.02 && u < 0.98;
}

bool IntersectionRules::isRelevantParticipant(
    const Vehicle& vehicle,
    const City& city,
    int intersectionId
) const {
    if (vehicle.isFinished()) {
        return false;
    }

    const IntersectionReservation* reservation = getReservation(intersectionId);
    if (reservation != nullptr && reservation->vehicleId == vehicle.getId()) {
        return true;
    }

    const RoadSegment* road = city.findRoadSegmentById(vehicle.getCurrentRoadSegmentId());
    if (road == nullptr) {
        return false;
    }

    if (road->getToIntersectionId() != intersectionId) {
        return false;
    }

    const double distanceToEnd = road->getLength() - vehicle.getPositionOnRoad();
    return distanceToEnd <= TrafficPolicy::APPROACH_RELEVANCE_DISTANCE_M;
}

bool IntersectionRules::mustYieldToOther(
    const MovementKey& ego,
    const MovementKey& other,
    PriorityMode mode,
    const Intersection& intersection,
    const City& city
) const {
    if (!movementsConflict(city, ego, other)) {
        return false;
    }

    if (mode == PriorityMode::PriorityRoad) {
        const bool egoPri = isPriorityApproach(intersection, ego.fromRoadId);
        const bool otherPri = isPriorityApproach(intersection, other.fromRoadId);
        if (!egoPri && otherPri) {
            return true;
        }
        if (egoPri && !otherPri) {
            return false;
        }
        // Same class → fall through to yield-to-right.
    }

    if (mode == PriorityMode::Signalized) {
        // Among greens, only reservation/conflict geometry matters — no YTR duty.
        // (Caller already checked lights.) Still yield if other already holds reservation
        // of a conflicting move — handled separately.
        return false;
    }

    // YieldToRight (and PriorityRoad same-class tiebreak).
    if (isFromRight(city, ego, other)) {
        return true;
    }

    return false;
}

const Vehicle* IntersectionRules::findVehicle(const vector<Vehicle>& vehicles, int vehicleId) const {
    for (const auto& vehicle : vehicles) {
        if (vehicle.getId() == vehicleId) {
            return &vehicle;
        }
    }
    return nullptr;
}
