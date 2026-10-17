#include "core/City.h"
#include "core/CollisionDetector.h"
#include "core/Intersection.h"
#include "core/LaneBlockage.h"
#include "core/RoadSegment.h"
#include "core/Simulation.h"
#include "core/Vehicle.h"
#include "core/Vector2D.h"
#include "ui/RenderComposer.h"

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

City makeDirectedPair(double length, double speedLimit, int laneCount) {
    // Road 1: A→B, Road 2: B→A (opposite directed segment).
    City city("lane_block_city");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(length, 0.0)));
    RoadSegment forward(1, 1, 2, length, speedLimit);
    forward.setLaneCount(laneCount);
    RoadSegment opposite(2, 2, 1, length, speedLimit);
    opposite.setLaneCount(laneCount);
    city.addRoadSegment(forward);
    city.addRoadSegment(opposite);
    return city;
}

City makeCorridor(double length, double speedLimit, int laneCount) {
    City city("lane_block_corridor");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(length, 0.0)));
    city.addIntersection(Intersection(3, Vector2D(length + 40.0, 0.0)));
    RoadSegment road1(1, 1, 2, length, speedLimit);
    road1.setLaneCount(laneCount);
    city.addRoadSegment(road1);
    city.addRoadSegment(RoadSegment(2, 2, 3, 40.0, speedLimit));
    return city;
}

Vehicle makeVehicle(int id, double maxSpeed, const vector<int>& route, int lane = 0) {
    Vehicle vehicle(id, maxSpeed);
    vehicle.setRoute(route);
    vehicle.setLaneIndex(lane);
    vehicle.setLength(4.5);
    return vehicle;
}

const Vehicle* findVehicle(const Simulation& sim, int id) {
    for (const auto& v : sim.getVehicles()) {
        if (v.getId() == id) {
            return &v;
        }
    }
    return nullptr;
}

bool hasEventType(const Simulation& sim, const string& type) {
    for (const auto& event : sim.getEventLog().getEvents()) {
        if (event.type == type) {
            return true;
        }
    }
    return false;
}

void testOneLaneStopAndResume() {
    City city = makeCorridor(80.0, 8.0, 1);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    Vehicle car = makeVehicle(1, 8.0, {1, 2}, 0);
    car.setPositionOnRoad(10.0);
    car.setSpeed(6.0);
    sim.addVehicle(car);

    const int blockId = sim.addLaneBlockage(1, 0, 40.0, 8.0, 0.0, 5.0, city);
    expect(blockId > 0, "one_lane_block_added");

    sim.start();
    step(sim, city, 0.1, 40);

    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr && v->getPositionOnRoad() < 40.0, "one_lane_stops_before_block");
    expect(v != nullptr && v->isStopped(), "one_lane_stopped_flag");
    expect(hasEventType(sim, "lane_blockage_started"), "one_lane_start_event");

    // Block ends at t=5; resume afterward.
    step(sim, city, 0.1, 40);
    v = findVehicle(sim, 1);
    expect(hasEventType(sim, "lane_blockage_ended"), "one_lane_end_event");
    expect(v != nullptr && v->getPositionOnRoad() > 45.0, "one_lane_resumes_after_clear");
}

void testAllLanesBlockedQueue() {
    City city = makeCorridor(90.0, 8.0, 2);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    Vehicle a = makeVehicle(1, 8.0, {1, 2}, 0);
    a.setPositionOnRoad(15.0);
    a.setSpeed(5.0);
    Vehicle b = makeVehicle(2, 8.0, {1, 2}, 1);
    b.setPositionOnRoad(18.0);
    b.setSpeed(5.0);
    sim.addVehicle(a);
    sim.addVehicle(b);

    expect(sim.addLaneBlockage(1, 0, 50.0, 10.0, 0.0, 20.0, city) > 0, "all_lanes_block0");
    expect(sim.addLaneBlockage(1, 1, 50.0, 10.0, 0.0, 20.0, city) > 0, "all_lanes_block1");

    sim.start();
    step(sim, city, 0.1, 60);

    const Vehicle* v0 = findVehicle(sim, 1);
    const Vehicle* v1 = findVehicle(sim, 2);
    expect(v0 != nullptr && v0->getPositionOnRoad() < 50.0, "all_lanes_v0_queued");
    expect(v1 != nullptr && v1->getPositionOnRoad() < 50.0, "all_lanes_v1_queued");
    expect(v0 != nullptr && v0->getLaneIndex() == 0, "all_lanes_no_detour_v0");
    expect(v1 != nullptr && v1->getLaneIndex() == 1, "all_lanes_no_detour_v1");
}

void testOppositeDirectionUnaffected() {
    City city = makeDirectedPair(70.0, 8.0, 1);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    Vehicle forward = makeVehicle(1, 8.0, {1}, 0);
    forward.setPositionOnRoad(10.0);
    forward.setSpeed(6.0);
    Vehicle opposite = makeVehicle(2, 8.0, {2}, 0);
    opposite.setPositionOnRoad(10.0);
    opposite.setSpeed(6.0);
    sim.addVehicle(forward);
    sim.addVehicle(opposite);

    expect(sim.addLaneBlockage(1, 0, 35.0, 8.0, 0.0, 30.0, city) > 0, "opp_block_on_forward");

    sim.start();
    step(sim, city, 0.1, 50);

    const Vehicle* vf = findVehicle(sim, 1);
    const Vehicle* vo = findVehicle(sim, 2);
    expect(vf != nullptr && vf->getPositionOnRoad() < 35.0, "opp_forward_stopped");
    expect(vo != nullptr && vo->getPositionOnRoad() > 40.0, "opp_opposite_unaffected");
}

void testManualClearEndsBlock() {
    City city = makeCorridor(70.0, 8.0, 1);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    Vehicle car = makeVehicle(1, 8.0, {1, 2}, 0);
    car.setPositionOnRoad(5.0);
    car.setSpeed(6.0);
    sim.addVehicle(car);

    const int id = sim.addLaneBlockage(1, 0, 30.0, 6.0, 0.0, 100.0, city);
    expect(id > 0, "clear_block_added");

    sim.start();
    step(sim, city, 0.1, 30);
    expect(findVehicle(sim, 1)->getPositionOnRoad() < 30.0, "clear_stopped_before");

    expect(sim.clearLaneBlockage(id), "clear_api_removes");
    expect(sim.getLaneBlockages().empty(), "clear_list_empty");

    step(sim, city, 0.1, 40);
    expect(findVehicle(sim, 1)->getPositionOnRoad() > 35.0, "clear_resume_after_manual");
}

void testNoSpawnInsideVehicle() {
    City city = makeCorridor(60.0, 6.0, 1);
    Simulation sim;

    Vehicle car = makeVehicle(1, 6.0, {1, 2}, 0);
    car.setPositionOnRoad(25.0);
    car.setSpeed(0.0);
    car.setDesiredSpeed(0.0);
    car.setMaxSpeed(0.0);
    sim.addVehicle(car);

    // Span overlaps vehicle body [20.5, 25.0].
    const int rejected = sim.addLaneBlockage(1, 0, 22.0, 6.0, 0.0, 10.0, city);
    expect(rejected < 0, "spawn_inside_vehicle_rejected");

    const int ok = sim.addLaneBlockage(1, 0, 40.0, 5.0, 0.0, 10.0, city);
    expect(ok > 0, "spawn_clear_of_vehicle_accepted");
}

void testZipperFairnessAndEmptyQueue() {
    City city = makeCorridor(100.0, 6.0, 2);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    // Closed lane queue (lane 0) and open lane queue (lane 1) near merge at 55.
    Vehicle c1 = makeVehicle(1, 6.0, {1, 2}, 0);
    c1.setPositionOnRoad(42.0);
    c1.setSpeed(0.5);
    Vehicle c2 = makeVehicle(2, 6.0, {1, 2}, 0);
    c2.setPositionOnRoad(35.0);
    c2.setSpeed(0.5);

    Vehicle o1 = makeVehicle(3, 6.0, {1, 2}, 1);
    o1.setPositionOnRoad(44.0);
    o1.setSpeed(0.5);
    Vehicle o2 = makeVehicle(4, 6.0, {1, 2}, 1);
    o2.setPositionOnRoad(37.0);
    o2.setSpeed(0.5);

    sim.addVehicle(c1);
    sim.addVehicle(c2);
    sim.addVehicle(o1);
    sim.addVehicle(o2);

    expect(sim.addLaneBlockage(1, 0, 55.0, 10.0, 0.0, 40.0, city) > 0, "zipper_block_added");

    sim.start();

    vector<int> mergeOrder;
    int lastLaneSeen = -1;
    int alternations = 0;

    for (int i = 0; i < 250; ++i) {
        sim.update(0.1, city);

        for (const auto& v : sim.getVehicles()) {
            if (v.isFinished()) {
                continue;
            }
            // Count a "merge passage" when a former closed-lane car is now on lane 1 past 50,
            // or open-lane car passes 55.
            if (v.getCurrentRoadSegmentId() != 1) {
                continue;
            }
            if (v.getPositionOnRoad() < 52.0) {
                continue;
            }

            bool already = false;
            for (int id : mergeOrder) {
                if (id == v.getId()) {
                    already = true;
                    break;
                }
            }
            if (already) {
                continue;
            }

            // Attribute lane by original: 1,2 closed; 3,4 open — after LC closed cars are lane 1.
            const int sourceLane = (v.getId() <= 2) ? 0 : 1;
            mergeOrder.push_back(v.getId());
            if (lastLaneSeen >= 0 && lastLaneSeen != sourceLane) {
                ++alternations;
            }
            lastLaneSeen = sourceLane;
        }
    }

    expect(mergeOrder.size() >= 2, "zipper_at_least_two_passed");
    expect(alternations >= 1, "zipper_alternates_when_both_queues");

    // Empty closed queue: open must not stall for a ghost car.
    Simulation simEmpty;
    simEmpty.setMinVehicleGap(3.0);
    Vehicle onlyOpen = makeVehicle(10, 8.0, {1, 2}, 1);
    onlyOpen.setPositionOnRoad(40.0);
    onlyOpen.setSpeed(6.0);
    simEmpty.addVehicle(onlyOpen);
    expect(simEmpty.addLaneBlockage(1, 0, 55.0, 10.0, 0.0, 30.0, city) > 0, "zipper_empty_block");
    simEmpty.start();
    step(simEmpty, city, 0.1, 50);
    const Vehicle* openOnly = findVehicle(simEmpty, 10);
    expect(openOnly != nullptr && openOnly->getPositionOnRoad() > 60.0, "zipper_empty_closed_no_stall");
}

void testSnapshotContainsBlocks() {
    City city = makeCorridor(50.0, 6.0, 2);
    Simulation sim;
    expect(sim.addLaneBlockage(1, 1, 20.0, 5.0, 0.0, 15.0, city) > 0, "snap_block_added");
    sim.start();
    step(sim, city, 0.1, 5);

    RenderComposer composer;
    RenderSnapshot snap = composer.buildSnapshot(city, sim);
    expect(!snap.laneBlockages.empty(), "snap_has_lane_blockages");
    if (!snap.laneBlockages.empty()) {
        expect(snap.laneBlockages[0].laneIndex == 1, "snap_lane_index");
        expect(snap.laneBlockages[0].active, "snap_active_flag");
        expect(snap.laneBlockages[0].length > 0.0, "snap_length");
        expect(snap.laneBlockages[0].hasMergeZone, "snap_merge_zone_multi_lane");
    }
}

void testMultiLaneEscapeNotOppositeRoad() {
    City city = makeCorridor(90.0, 7.0, 2);
    Simulation sim;
    sim.setMinVehicleGap(3.0);

    Vehicle car = makeVehicle(1, 7.0, {1, 2}, 0);
    car.setPositionOnRoad(40.0);
    car.setSpeed(4.0);
    sim.addVehicle(car);

    expect(sim.addLaneBlockage(1, 0, 55.0, 8.0, 0.0, 25.0, city) > 0, "escape_block_added");
    sim.start();

    bool sawEscapeLc = false;
    bool sawPastBlockOnOpenLane = false;
    bool neverUsedOppositeRoad = true;

    for (int i = 0; i < 100; ++i) {
        sim.update(0.1, city);

        for (const auto& event : sim.getEventLog().getEvents()) {
            if (event.type == "vehicle_lane_change" && event.reason == "lane_blockage_escape") {
                sawEscapeLc = true;
            }
        }

        for (const auto& v : sim.getVehicles()) {
            if (v.getId() != 1) {
                continue;
            }
            // Opposite directed segment is not in this corridor; only roads 1 and 2 exist.
            if (v.getCurrentRoadSegmentId() != 1 && v.getCurrentRoadSegmentId() != 2) {
                neverUsedOppositeRoad = false;
            }
            if (v.getCurrentRoadSegmentId() == 1 &&
                v.getLaneIndex() == 1 &&
                v.getPositionOnRoad() > 55.0) {
                sawPastBlockOnOpenLane = true;
            }
            if (v.getCurrentRoadSegmentId() == 2) {
                // Reached next road only after clearing the blocked segment via open lane.
                sawPastBlockOnOpenLane = true;
            }
        }
    }

    expect(sawEscapeLc, "escape_lc_to_open_lane");
    expect(sawPastBlockOnOpenLane, "escape_passes_via_open_lane");
    expect(neverUsedOppositeRoad, "escape_stays_same_direction_roads");
}

} // namespace

int laneBlockagePassedCount() {
    return g_passed;
}

int runLaneBlockageTests() {
    g_failures = 0;
    g_passed = 0;

    cout << "\n--- Lane blockage Stage 5 tests ---\n";
    testOneLaneStopAndResume();
    testAllLanesBlockedQueue();
    testOppositeDirectionUnaffected();
    testManualClearEndsBlock();
    testNoSpawnInsideVehicle();
    testZipperFairnessAndEmptyQueue();
    testSnapshotContainsBlocks();
    testMultiLaneEscapeNotOppositeRoad();

    return g_failures;
}
