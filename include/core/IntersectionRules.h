#pragma once

#include <map>
#include <string>
#include <vector>
#include "core/City.h"
#include "core/TrafficLight.h"
#include "core/TrafficPolicy.h"
#include "core/Vehicle.h"

using namespace std;

enum class TurnType {
    Straight,
    Left,
    Right
};

enum class JunctionBlockReason {
    None,
    RedLight,
    BlockedNextRoad,
    YieldPriority,
    IntersectionConflict,
    RouteInvalid,
    DestinationReached
};

struct MovementKey {
    int intersectionId = -1;
    int fromRoadId = -1;
    int toRoadId = -1;
    TurnType turn = TurnType::Straight;
};

struct IntersectionReservation {
    int vehicleId = -1;
    int fromRoadId = -1;
    int toRoadId = -1;
    TurnType turn = TurnType::Straight;

    bool isActive() const {
        return vehicleId >= 0;
    }
};

// Geometry-based turn classification, priority / yield-to-right, and junction occupation.
// Does not use vehicleId as the sole priority rule; simultaneous arrivals use wait time
// then approach bearing (see resolveGrantOrder).
class IntersectionRules {
public:
    void clear();

    // Drop reservations for finished / removed vehicles, or after clearance on the exit road.
    void clearStaleReservations(const City& city, const vector<Vehicle>& vehicles);

    TurnType classifyTurn(const City& city, int fromRoadId, int toRoadId) const;

    // Etap 2: pick an entry lane on the exit road from turn type + lane counts.
    // Lane 0 = rightmost (RHT); laneCount-1 = leftmost. Clamps to [0, toLaneCount-1].
    static int suggestEntryLane(
        int fromLaneIndex,
        int fromLaneCount,
        int toLaneCount,
        TurnType turn
    );

    MovementKey makeMovement(const City& city, const Vehicle& vehicle) const;

    // Unsignalized None → YieldToRight. Signalized / PriorityRoad kept as stored.
    PriorityMode effectivePriorityMode(const Intersection& intersection) const;

    bool isPriorityApproach(const Intersection& intersection, int fromRoadId) const;

    // True when two movements must not occupy the junction at the same time.
    bool movementsConflict(const City& city, const MovementKey& a, const MovementKey& b) const;

    // Other vehicle is approaching from ego's right (RIGHT_HAND_TRAFFIC policy).
    bool isFromRight(const City& city, const MovementKey& ego, const MovementKey& other) const;

    bool hasReservation(int intersectionId) const;
    // First active reservation for the intersection (tests / simple queries).
    const IntersectionReservation* getReservation(int intersectionId) const;
    const vector<IntersectionReservation>* getReservationsFor(int intersectionId) const;
    const map<int, vector<IntersectionReservation>>& getReservations() const;

    void reserve(int intersectionId, const Vehicle& vehicle, const MovementKey& movement);
    void release(int intersectionId);
    void releaseVehicle(int vehicleId);

    // Record when a vehicle first waits at a stop line (starvation avoidance).
    void noteWaiting(int vehicleId, double currentTime);
    void clearWaiting(int vehicleId);
    double waitStartTime(int vehicleId) const;

    // Why ego may not enter yet (priority / conflict / lights / space). Empty block → may enter.
    JunctionBlockReason evaluateEntry(
        const Vehicle& vehicle,
        const City& city,
        const vector<TrafficLight>& trafficLights,
        const vector<Vehicle>& vehicles,
        bool nextRoadHasSpace
    ) const;

    static string blockReasonToStopReason(JunctionBlockReason reason);

    // Among candidates that already passed lights + space, pick who may claim this tick.
    // Returns vehicle ids in grant order (usually 0 or 1 per intersection).
    vector<int> resolveGrants(
        int intersectionId,
        const vector<int>& candidateVehicleIds,
        const City& city,
        const vector<TrafficLight>& trafficLights,
        const vector<Vehicle>& vehicles,
        double currentTime
    ) const;

private:
    map<int, vector<IntersectionReservation>> reservations; // intersectionId → occupations
    map<int, double> waitStartedAt;                         // vehicleId → sim time

    Vector2D roadInboundDirection(const City& city, int roadId) const;
    Vector2D roadOutboundDirection(const City& city, int roadId) const;
    bool segmentIntersect(
        const Vector2D& a1,
        const Vector2D& a2,
        const Vector2D& b1,
        const Vector2D& b2
    ) const;

    bool isRelevantParticipant(
        const Vehicle& vehicle,
        const City& city,
        int intersectionId
    ) const;

    bool mustYieldToOther(
        const MovementKey& ego,
        const MovementKey& other,
        PriorityMode mode,
        const Intersection& intersection,
        const City& city
    ) const;

    const Vehicle* findVehicle(const vector<Vehicle>& vehicles, int vehicleId) const;
};
