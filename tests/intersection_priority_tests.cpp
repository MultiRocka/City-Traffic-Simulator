#include "core/City.h"
#include "core/CityBuilder.h"
#include "core/CollisionDetector.h"
#include "core/Intersection.h"
#include "core/IntersectionRules.h"
#include "core/MapDescription.h"
#include "core/RoadSegment.h"
#include "core/Scenario.h"
#include "core/Simulation.h"
#include "core/TrafficLight.h"
#include "core/TrafficPolicy.h"
#include "core/Vehicle.h"
#include "core/Vector2D.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

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

void step(Simulation& sim, const City& city, double dt, int steps) {
    for (int i = 0; i < steps; ++i) {
        sim.update(dt, city);
    }
}

Vehicle makeVehicle(int id, double maxSpeed, const vector<int>& route) {
    Vehicle vehicle(id, maxSpeed);
    vehicle.setRoute(route);
    vehicle.setLength(4.0);
    return vehicle;
}

const Vehicle* findVehicle(const Simulation& sim, int id) {
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == id) {
            return &vehicle;
        }
    }
    return nullptr;
}

// Crossroads: center 5, arms N=1, E=2, S=3, W=4.
// Roads into center: S→C (10), W→C (11), N→C (12), E→C (13)
// Roads out of center: C→N (20), C→E (21), C→S (22), C→W (23)
bool buildCrossroads(City& city, PriorityMode mode, const vector<int>& priorityRoadIds) {
    MapDescription map;
    map.name = "crossroads";

    auto addNode = [&](int id, double x, double y) {
        MapIntersectionDesc node;
        node.id = id;
        node.position = Vector2D(x, y);
        if (id == 5) {
            node.priorityMode = mode;
            node.priorityRoadIds = priorityRoadIds;
        }
        map.intersections.push_back(node);
    };

    addNode(1, 0.0, 40.0);   // N
    addNode(2, 40.0, 0.0);   // E
    addNode(3, 0.0, -40.0);  // S
    addNode(4, -40.0, 0.0);  // W
    addNode(5, 0.0, 0.0);    // center

    auto addRoad = [&](int id, int from, int to, double length) {
        MapRoadDesc road;
        road.id = id;
        road.fromIntersectionId = from;
        road.toIntersectionId = to;
        road.length = length;
        road.speedLimit = 8.0;
        map.roads.push_back(road);
    };

    addRoad(10, 3, 5, 40.0); // S → C
    addRoad(11, 4, 5, 40.0); // W → C
    addRoad(12, 1, 5, 40.0); // N → C
    addRoad(13, 2, 5, 40.0); // E → C
    addRoad(20, 5, 1, 40.0); // C → N
    addRoad(21, 5, 2, 40.0); // C → E
    addRoad(22, 5, 3, 40.0); // C → S
    addRoad(23, 5, 4, 40.0); // C → W

    string error;
    return CityBuilder::build(map, city, error);
}

void testLightsStillStopAndResume() {
    City city;
    expect(buildCrossroads(city, PriorityMode::Signalized, {}), "prio_lights_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);
    // Long red then green on approach 10 (S→C), route continues north.
    sim.addTrafficLight(TrafficLight(1, 10, 12.0, 6.0, false));

    Vehicle vehicle = makeVehicle(1, 8.0, {10, 20});
    vehicle.setPositionOnRoad(30.0);
    sim.addVehicle(vehicle);
    sim.start();

    step(sim, city, 0.1, 50);
    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr, "prio_lights_present_on_red");
    if (v != nullptr) {
        expect(v->getCurrentRoadSegmentId() == 10, "prio_lights_held_on_approach");
        expect(v->isStopped(), "prio_lights_stopped_on_red");
        expect(
            v->getPositionOnRoad() <= 40.0 - SimConstants::RED_LIGHT_STOP_DISTANCE + 0.3,
            "prio_lights_before_stop_line"
        );
    }

    step(sim, city, 0.1, 120);
    bool advanced = false;
    for (const auto& vehicleState : sim.getVehicles()) {
        if (vehicleState.getId() != 1) {
            continue;
        }
        advanced = vehicleState.getCurrentRoadSegmentId() == 20 || vehicleState.isFinished();
    }
    if (sim.getVehicles().empty()) {
        advanced = true;
    }
    expect(advanced, "prio_lights_resumed_after_green");
}

void testPriorityRoadGoesWhileSubordinateWaits() {
    // E–W (11 → 21) is priority; S→N (10 → 20) is subordinate.
    City city;
    expect(buildCrossroads(city, PriorityMode::PriorityRoad, {11}), "prio_road_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle priority = makeVehicle(1, 6.0, {11, 21});
    priority.setPositionOnRoad(36.0);
    Vehicle subordinate = makeVehicle(2, 6.0, {10, 20});
    subordinate.setPositionOnRoad(39.5);
    subordinate.setSpeed(0.0);
    sim.addVehicle(priority);
    sim.addVehicle(subordinate);
    sim.start();

    IntersectionRules probe;
    expect(
        probe.movementsConflict(
            city,
            probe.makeMovement(city, priority),
            probe.makeMovement(city, subordinate)
        ),
        "prio_road_ew_ns_conflict"
    );

    // Subordinate is at the stop line; priority is approaching.
    step(sim, city, 0.05, 4);
    const Vehicle* sub = findVehicle(sim, 2);
    const Vehicle* pri = findVehicle(sim, 1);
    expect(sub != nullptr && sub->getCurrentRoadSegmentId() == 10, "prio_road_sub_still_waiting");
    expect(pri != nullptr, "prio_road_priority_present");

    // Priority should enter the exit before subordinate; sub held while reservation active.
    bool priorityFirst = false;
    bool subordinateWaitedDuringPriority = false;
    bool subordinateLater = false;
    for (int i = 0; i < 200; ++i) {
        sim.update(0.05, city);
        const Vehicle* p = findVehicle(sim, 1);
        const Vehicle* s = findVehicle(sim, 2);
        const bool priorityOccupying =
            sim.getIntersectionRules().hasReservation(5) &&
            sim.getIntersectionRules().getReservation(5) != nullptr &&
            sim.getIntersectionRules().getReservation(5)->vehicleId == 1;

        if (p != nullptr && p->getCurrentRoadSegmentId() == 21) {
            if (!priorityFirst) {
                priorityFirst = true;
            }
            if (s != nullptr && s->getCurrentRoadSegmentId() == 10) {
                subordinateWaitedDuringPriority = true;
            }
        }
        if (priorityOccupying && s != nullptr && s->getCurrentRoadSegmentId() == 10) {
            subordinateWaitedDuringPriority = true;
        }
        if (priorityFirst && s != nullptr && s->getCurrentRoadSegmentId() == 20) {
            subordinateLater = true;
            break;
        }
        if (priorityFirst && s == nullptr) {
            subordinateLater = true;
            break;
        }
    }

    expect(priorityFirst, "prio_road_priority_enters_first");
    expect(subordinateWaitedDuringPriority, "prio_road_sub_waits_while_priority_crosses");
    expect(subordinateLater, "prio_road_subordinate_proceeds_after");
}

void testYieldToRight() {
    // Ego from west (11→21) going east; other from south (10→20) going north.
    // For RHT, south approach is to the right of westbound? 
    // West inbound = (1,0). From-right inbound = rotate90CCW = (0,1) = northbound.
    // So traffic FROM the north (12) is from the right of west→east traveler.
    // Use: vehicle A from west, vehicle B from north — A yields to B.
    City city;
    expect(buildCrossroads(city, PriorityMode::YieldToRight, {}), "prio_ytr_map_builds");
    expect(TrafficPolicy::RIGHT_HAND_TRAFFIC, "prio_ytr_rht_policy");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle fromWest = makeVehicle(1, 6.0, {11, 21}); // W→E
    fromWest.setPositionOnRoad(39.5);
    fromWest.setSpeed(0.0);
    // South approach inbound (0,1) is from the right of westbound under RHT.
    Vehicle fromSouth = makeVehicle(2, 6.0, {10, 20}); // S→N
    fromSouth.setPositionOnRoad(39.5);
    fromSouth.setSpeed(0.0);

    sim.addVehicle(fromWest);
    sim.addVehicle(fromSouth);
    sim.start();

    IntersectionRules rules;
    MovementKey west = rules.makeMovement(city, fromWest);
    MovementKey south = rules.makeMovement(city, fromSouth);
    expect(rules.isFromRight(city, west, south), "prio_ytr_south_is_from_right_of_west");
    expect(rules.movementsConflict(city, west, south), "prio_ytr_movements_conflict");

    bool southFirst = false;
    bool westAfter = false;
    for (int i = 0; i < 250; ++i) {
        sim.update(0.05, city);
        const Vehicle* w = findVehicle(sim, 1);
        const Vehicle* s = findVehicle(sim, 2);
        if (s != nullptr && s->getCurrentRoadSegmentId() == 20 && !southFirst) {
            southFirst = true;
            if (w != nullptr) {
                expect(w->getCurrentRoadSegmentId() == 11, "prio_ytr_west_waits_for_right");
            }
        }
        if (southFirst && w != nullptr && w->getCurrentRoadSegmentId() == 21) {
            westAfter = true;
            break;
        }
        if (southFirst && w == nullptr) {
            westAfter = true;
            break;
        }
    }

    expect(southFirst, "prio_ytr_right_approach_goes_first");
    expect(westAfter, "prio_ytr_left_approach_then_proceeds");
}

void testLeftRightConflict() {
    City city;
    expect(buildCrossroads(city, PriorityMode::YieldToRight, {}), "prio_lr_map_builds");

    IntersectionRules rules;
    // Straight from south (10→20) vs left from east (13→20) merging / crossing.
    Vehicle straight = makeVehicle(1, 6.0, {10, 20});
    Vehicle leftTurn = makeVehicle(2, 6.0, {13, 20});
    MovementKey a = rules.makeMovement(city, straight);
    MovementKey b = rules.makeMovement(city, leftTurn);
    expect(a.turn == TurnType::Straight, "prio_lr_south_classified_straight");
    expect(b.turn == TurnType::Left || b.turn == TurnType::Right, "prio_lr_east_classified_turn");
    // East→North is left under RHT (inbound (-1,0), outbound (0,1), cross = -1*1 - 0*0 = -1 → right?
    // inbound from E: from (40,0) to (0,0) = (-40,0) → (-1,0)
    // outbound C→N: (0,0) to (0,40) = (0,40) → (0,1)
    // cross = (-1)(1) - (0)(0) = -1 → clockwise → Right under RHT.
    // Use west left to north: W→C inbound (1,0), C→N outbound (0,1), cross = 1 > 0 → Left.
    Vehicle leftFromWest = makeVehicle(3, 6.0, {11, 20});
    MovementKey left = rules.makeMovement(city, leftFromWest);
    expect(left.turn == TurnType::Left, "prio_lr_west_to_north_is_left");
    expect(rules.movementsConflict(city, a, left), "prio_lr_left_conflicts_opposing_straight");

    Simulation sim;
    sim.setMinVehicleGap(2.5);
    straight.setPositionOnRoad(39.5);
    straight.setSpeed(0.0);
    leftFromWest.setPositionOnRoad(39.5);
    leftFromWest.setSpeed(0.0);
    sim.addVehicle(straight);
    sim.addVehicle(leftFromWest);
    sim.start();

    bool straightFirst = false;
    for (int i = 0; i < 200; ++i) {
        sim.update(0.05, city);
        const Vehicle* s = findVehicle(sim, 1);
        const Vehicle* l = findVehicle(sim, 3);
        if (s != nullptr && s->getCurrentRoadSegmentId() == 20 && !straightFirst) {
            straightFirst = true;
            if (l != nullptr) {
                expect(l->getCurrentRoadSegmentId() == 11, "prio_lr_left_waits_for_straight");
            }
        }
        if (straightFirst && l != nullptr && l->getCurrentRoadSegmentId() == 20) {
            break;
        }
    }
    expect(straightFirst, "prio_lr_straight_clears_before_left");
}

void testConflictReservationPreventsSimultaneousOccupy() {
    City city;
    expect(buildCrossroads(city, PriorityMode::YieldToRight, {}), "prio_res_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle a = makeVehicle(1, 6.0, {10, 20}); // S→N
    a.setPositionOnRoad(39.8);
    a.setSpeed(0.0);
    Vehicle b = makeVehicle(2, 6.0, {11, 21}); // W→E (conflicts with S→N)
    b.setPositionOnRoad(39.8);
    b.setSpeed(0.0);
    sim.addVehicle(a);
    sim.addVehicle(b);
    sim.start();

    int maxOccupants = 0;
    for (int i = 0; i < 80; ++i) {
        sim.update(0.05, city);
        const auto* list = sim.getIntersectionRules().getReservationsFor(5);
        const int count = list != nullptr ? static_cast<int>(list->size()) : 0;
        if (count > maxOccupants) {
            maxOccupants = count;
        }
        if (count >= 2) {
            // If two reservations exist they must be compatible — S→N and W→E conflict,
            // so this should never happen.
            IntersectionRules probe;
            MovementKey ma;
            ma.intersectionId = 5;
            ma.fromRoadId = (*list)[0].fromRoadId;
            ma.toRoadId = (*list)[0].toRoadId;
            ma.turn = (*list)[0].turn;
            MovementKey mb;
            mb.intersectionId = 5;
            mb.fromRoadId = (*list)[1].fromRoadId;
            mb.toRoadId = (*list)[1].toRoadId;
            mb.turn = (*list)[1].turn;
            expect(!probe.movementsConflict(city, ma, mb), "prio_res_no_conflicting_double_occupy");
        }
    }

    expect(maxOccupants <= 1, "prio_res_at_most_one_conflicting_occupant");
}

void testResumeAfterClear() {
    City city;
    expect(buildCrossroads(city, PriorityMode::PriorityRoad, {11}), "prio_resume_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle priority = makeVehicle(1, 8.0, {11, 21});
    priority.setPositionOnRoad(38.0);
    Vehicle subordinate = makeVehicle(2, 8.0, {10, 20});
    subordinate.setPositionOnRoad(39.5);
    subordinate.setSpeed(0.0);
    sim.addVehicle(priority);
    sim.addVehicle(subordinate);
    sim.start();

    bool subEntered = false;
    for (int i = 0; i < 300; ++i) {
        sim.update(0.05, city);
        const Vehicle* s = findVehicle(sim, 2);
        if (s != nullptr && s->getCurrentRoadSegmentId() == 20) {
            subEntered = true;
            break;
        }
        if (s == nullptr && i > 50) {
            // Finished after entering — OK
            subEntered = true;
            break;
        }
    }
    expect(subEntered, "prio_resume_subordinate_enters_after_clear");
}

void testNoPermanentReservationAfterFinish() {
    City city;
    expect(buildCrossroads(city, PriorityMode::None, {}), "prio_clear_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle vehicle = makeVehicle(1, 10.0, {10, 20});
    vehicle.setPositionOnRoad(35.0);
    sim.addVehicle(vehicle);
    sim.start();

    step(sim, city, 0.1, 200);

    expect(sim.getVehicles().empty() || findVehicle(sim, 1) == nullptr ||
               (findVehicle(sim, 1) != nullptr && findVehicle(sim, 1)->getCurrentRoadSegmentId() == 20),
           "prio_clear_vehicle_progressed_or_finished");

    // Force finish path: short route end.
    Simulation sim2;
    sim2.setMinVehicleGap(2.5);
    Vehicle shortRoute = makeVehicle(7, 12.0, {10});
    shortRoute.setPositionOnRoad(39.5);
    sim2.addVehicle(shortRoute);
    sim2.start();
    step(sim2, city, 0.1, 30);

    expect(!sim2.getIntersectionRules().hasReservation(5), "prio_clear_no_reservation_after_finish");
    expect(sim2.getIntersectionRules().getReservations().empty(), "prio_clear_reservation_map_empty");
}

void testGreenStillNeedsSpaceAndConflict() {
    City city;
    expect(buildCrossroads(city, PriorityMode::Signalized, {}), "prio_green_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(3.0);
    // Timed light stuck on a long green (green does not skip entrance space).
    sim.addTrafficLight(TrafficLight(1, 10, 60.0, 1.0, true));

    // Block exit entrance.
    Vehicle blocker = makeVehicle(9, 0.1, {20});
    blocker.setPositionOnRoad(1.0);
    blocker.setSpeed(0.0);
    Vehicle waiter = makeVehicle(1, 6.0, {10, 20});
    waiter.setPositionOnRoad(39.5);
    waiter.setSpeed(0.0);
    sim.addVehicle(blocker);
    sim.addVehicle(waiter);
    sim.start();

    step(sim, city, 0.05, 20);
    const Vehicle* w = findVehicle(sim, 1);
    expect(w != nullptr && w->getCurrentRoadSegmentId() == 10, "prio_green_still_blocked_by_space");

    string reason = sim.getCollisionDetector().determineStopReason(
        *w,
        city,
        sim.getTrafficLights(),
        sim.getPedestrianCrossings(),
        sim.getVehicles(),
        &sim.getIntersectionRules()
    );
    expect(reason == "blocked_next_road", "prio_green_stop_reason_blocked_next");
}

void testScenarioPriorityModesWired() {
    City city;
    Simulation sim;
    Scenario::setupIntersectionTest(city, sim);

    const Intersection* signalized = city.findIntersectionById(6);
    const Intersection* priority = city.findIntersectionById(2);
    const Intersection* ytr = city.findIntersectionById(5);

    expect(signalized != nullptr && signalized->getPriorityMode() == PriorityMode::Signalized,
           "prio_scenario_signalized_wired");
    expect(priority != nullptr && priority->getPriorityMode() == PriorityMode::PriorityRoad,
           "prio_scenario_priority_road_wired");
    expect(priority != nullptr && !priority->getPriorityRoadIds().empty(),
           "prio_scenario_priority_road_ids");
    expect(ytr != nullptr && ytr->getPriorityMode() == PriorityMode::YieldToRight,
           "prio_scenario_yield_to_right_wired");
}

void testStopReasonYieldAndConflict() {
    City city;
    expect(buildCrossroads(city, PriorityMode::PriorityRoad, {11}), "prio_reason_map_builds");

    Simulation sim;
    sim.setMinVehicleGap(2.5);

    Vehicle priority = makeVehicle(1, 6.0, {11, 21});
    priority.setPositionOnRoad(39.5);
    priority.setSpeed(0.0);
    Vehicle subordinate = makeVehicle(2, 6.0, {10, 20});
    subordinate.setPositionOnRoad(39.5);
    subordinate.setSpeed(0.0);
    sim.addVehicle(priority);
    sim.addVehicle(subordinate);
    sim.start();

    // Hold priority on approach without letting it leave yet: one update to register wait.
    sim.update(0.05, city);

    const Vehicle* sub = findVehicle(sim, 2);
    expect(sub != nullptr, "prio_reason_sub_present");
    if (sub != nullptr) {
        string reason = sim.getCollisionDetector().determineStopReason(
            *sub,
            city,
            sim.getTrafficLights(),
            sim.getPedestrianCrossings(),
            sim.getVehicles(),
            &sim.getIntersectionRules()
        );
        expect(reason == "yield_priority" || reason == "intersection_conflict" || sub->getCurrentRoadSegmentId() != 10,
               "prio_reason_yield_or_conflict_or_moved");
    }
}

} // namespace

int runIntersectionPriorityTests() {
    cout << "\nIntersection priority tests (Stage 3)\n";

    testLightsStillStopAndResume();
    testPriorityRoadGoesWhileSubordinateWaits();
    testYieldToRight();
    testLeftRightConflict();
    testConflictReservationPreventsSimultaneousOccupy();
    testResumeAfterClear();
    testNoPermanentReservationAfterFinish();
    testGreenStillNeedsSpaceAndConflict();
    testScenarioPriorityModesWired();
    testStopReasonYieldAndConflict();

    return g_failures;
}

int intersectionPriorityPassedCount() {
    return g_passed;
}
