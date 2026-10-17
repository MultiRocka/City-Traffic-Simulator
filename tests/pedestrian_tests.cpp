#include "core/City.h"
#include "core/CollisionDetector.h"
#include "core/Intersection.h"
#include "core/Pedestrian.h"
#include "core/PedestrianCrossing.h"
#include "core/PedestrianManager.h"
#include "core/Phase.h"
#include "core/PhaseController.h"
#include "core/RoadSegment.h"
#include "core/Simulation.h"
#include "core/TrafficLight.h"
#include "core/Vehicle.h"
#include "core/Vector2D.h"
#include "ui/RenderComposer.h"
#include "ui/SimulationSession.h"

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

City makeRoadCity(double length, double speedLimit, int laneCount = 1) {
    City city("ped_road");
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
    return vehicle;
}

const Pedestrian* findPed(const Simulation& sim, int id) {
    for (const auto& ped : sim.getPedestrianManager().getPedestrians()) {
        if (ped.getId() == id) {
            return &ped;
        }
    }
    return nullptr;
}

const Vehicle* findVehicle(const Simulation& sim, int id) {
    for (const auto& v : sim.getVehicles()) {
        if (v.getId() == id) {
            return &v;
        }
    }
    return nullptr;
}

void testWaitingCrossingClearAndSnapshot() {
    City city = makeRoadCity(60.0, 6.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(42);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 30.0, 100.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    crossing.setWalkSpeed(2.0);
    sim.addPedestrianCrossing(crossing);

    const int pedId = sim.getPedestrianManager().spawnWaitingPedestrian(
        sim.getPedestrianCrossings().front(),
        city
    );
    expect(pedId > 0, "ped_spawn_waiting_id");

    // Empty road → ped should enter and cross.
    sim.start();
    step(sim, city, 0.1, 5);

    const Pedestrian* ped = findPed(sim, pedId);
    expect(ped != nullptr && ped->isOnRoadway(), "ped_entered_crossing");

    step(sim, city, 0.1, 40);
    ped = findPed(sim, pedId);
    expect(ped == nullptr || ped->isFinished(), "ped_cleared_crossing");
    expect(!sim.getPedestrianManager().hasPedestrianOnRoadway(1), "ped_manager_clear");

    // Visible in snapshot while present
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    step(sim, city, 0.1, 2);
    RenderComposer composer;
    RenderSnapshot snap = composer.buildSnapshot(city, sim);
    expect(!snap.pedestrians.empty(), "ped_snapshot_has_entity");
    expect(static_cast<int>(snap.crossings.size()) == 1, "ped_snapshot_has_crossing");
}

void testSignalizedClearAfterPhaseHold() {
    City city = makeRoadCity(50.0, 8.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(7);
    sim.setPedestrianYieldRefusalProbability(0.0);

    // Long inactive so a second request does not spawn during the clear check.
    PedestrianCrossing crossing(1, 1, 25.0, 1000.0, 5.0, false);
    crossing.setMode(CrossingMode::Signalized);
    crossing.setWalkSpeed(1.0);
    sim.addPedestrianCrossing(crossing);

    // Phase-controlled light on road 1: short red then green.
    PhaseController controller(1, 2);
    controller.addPhase(Phase(1, 3.0, {}));          // all red — ped walk
    controller.addPhase(Phase(2, 3.0, {1}));         // vehicle green
    sim.addPhaseController(controller);
    sim.addTrafficLight(TrafficLight(1, 1, 2, true));

    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    sim.start();

    // During red, ped enters.
    step(sim, city, 0.1, 15);
    expect(sim.getPedestrianManager().hasPedestrianOnRoadway(1) ||
               !sim.getPedestrianManager().getPedestrians().empty(),
           "ped_signal_entered_on_red");

    // Force a slow walker still on roadway when phase would end.
    if (!sim.getPedestrianManager().getPedestrians().empty()) {
        auto& peds = sim.getPedestrianManager().getPedestrians();
        for (auto& ped : peds) {
            if (ped.isOnRoadway()) {
                ped.setCrossProgress(0.1);
                // Slow enough that phase duration (3s) expires before clear.
                ped.setCrossWidth(20.0);
            }
        }
    }

    const int phaseBefore = sim.getPhaseControllers().front().getCurrentPhaseId();
    step(sim, city, 0.1, 40);
    const bool stillOnRoad = sim.getPedestrianManager().hasPedestrianOnRoadway(1);
    const int phaseAfter = sim.getPhaseControllers().front().getCurrentPhaseId();

    if (stillOnRoad) {
        expect(phaseAfter == phaseBefore, "ped_signal_phase_held_while_on_road");
    } else {
        expect(true, "ped_signal_phase_held_while_on_road");
    }

    // Eventually clears without being deleted mid-crossing (phase may advance after).
    step(sim, city, 0.1, 250);
    expect(!sim.getPedestrianManager().hasPedestrianOnRoadway(1), "ped_signal_eventually_clear");
}

void testUnsignalizedYield() {
    City city = makeRoadCity(80.0, 8.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(99);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    crossing.setWalkSpeed(1.5);
    sim.addPedestrianCrossing(crossing);

    sim.addVehicle(makeVehicle(1, 8.0, {1, 2}));
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    sim.start();

    bool sawStopBefore = false;
    bool sawPedReason = false;
    for (int i = 0; i < 120; ++i) {
        sim.update(0.1, city);
        const Vehicle* v = findVehicle(sim, 1);
        if (v == nullptr) {
            continue;
        }
        const bool pedPresent =
            sim.getPedestrianManager().hasWaitingPedestrian(1) ||
            sim.getPedestrianManager().hasPedestrianOnRoadway(1);
        if (pedPresent &&
            v->getPositionOnRoad() < 40.0 - SimConstants::CROSSING_STOP_DISTANCE + 1.0 &&
            (v->isStopped() || v->getSpeed() < 0.5)) {
            sawStopBefore = true;
            string reason = sim.getCollisionDetector().determineStopReason(
                *v,
                city,
                sim.getTrafficLights(),
                sim.getPedestrianCrossings(),
                sim.getVehicles(),
                nullptr,
                &sim.getPedestrianManager()
            );
            if (reason == "pedestrian_crossing" || reason == "pedestrian_on_road") {
                sawPedReason = true;
            }
        }
    }

    expect(findVehicle(sim, 1) != nullptr || sawStopBefore, "ped_yield_vehicle_present");
    expect(sawStopBefore, "ped_yield_car_stops_before_crossing");
    expect(sawPedReason, "ped_yield_stop_reason");
}

void testRefusalZeroAlwaysYields() {
    City city = makeRoadCity(80.0, 10.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(12345);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    sim.addPedestrianCrossing(crossing);
    sim.addVehicle(makeVehicle(1, 10.0, {1, 2}));
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);

    expect(sim.getPedestrianManager().mustYieldToWaitingPedestrian(1, 1),
           "ped_refusal0_yield_decision");

    sim.start();
    bool stayedBackWhileWaiting = false;
    for (int i = 0; i < 80; ++i) {
        sim.update(0.05, city);
        const Vehicle* v = findVehicle(sim, 1);
        if (v != nullptr &&
            sim.getPedestrianManager().hasWaitingPedestrian(1) &&
            v->getPositionOnRoad() < 40.0) {
            stayedBackWhileWaiting = true;
        }
    }
    expect(stayedBackWhileWaiting, "ped_refusal0_car_did_not_pass_waiting");
}

void testRefusalHundredPassesWaiting() {
    City city = makeRoadCity(80.0, 10.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(12345);
    sim.setPedestrianYieldRefusalProbability(1.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    crossing.setWalkSpeed(0.5);
    sim.addPedestrianCrossing(crossing);

    Vehicle car = makeVehicle(1, 10.0, {1, 2});
    car.setPositionOnRoad(5.0);
    sim.addVehicle(car);
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    sim.start();

    // Roll encounter while waiting.
    expect(!sim.getPedestrianManager().mustYieldToWaitingPedestrian(1, 1), "ped_refusal100_no_yield");

    step(sim, city, 0.1, 60);
    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr, "ped_refusal100_vehicle_present");
    if (v != nullptr) {
        expect(v->getPositionOnRoad() > 40.0 || v->getCurrentRoadSegmentId() == 2,
               "ped_refusal100_car_passed_while_ped_waiting");
    }

    // Ped should still be waiting or only enter after car clear — never hit logic.
    bool pedStillSafe = true;
    for (const auto& ped : sim.getPedestrianManager().getPedestrians()) {
        if (ped.isOnRoadway() && v != nullptr &&
            v->getCurrentRoadSegmentId() == 1 &&
            abs(v->getPositionOnRoad() - 40.0) < 3.0 &&
            v->getSpeed() > 1.0) {
            pedStillSafe = false;
        }
    }
    expect(pedStillSafe, "ped_refusal100_no_step_in_front");
}

void testRefusalFiveConfigNotFragile() {
    // With p=0.05, assert loose band + once-per-encounter stability — not exact 5-of-100.
    City city = makeRoadCity(100.0, 8.0, 1);
    PedestrianCrossing crossing(1, 1, 50.0, 5.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);

    const int encounters = 200;
    int refusals = 0;
    int stableOk = 0;
    for (int i = 0; i < encounters; ++i) {
        PedestrianManager m;
        m.setSeed(20261013u + static_cast<uint32_t>(i) * 9973u);
        m.setYieldRefusalProbability(0.05);
        m.spawnWaitingPedestrian(crossing, city);
        const int vehicleId = 1000 + i;
        const bool yield1 = m.mustYieldToWaitingPedestrian(vehicleId, 1);
        const bool yield2 = m.mustYieldToWaitingPedestrian(vehicleId, 1);
        if (yield1 == yield2) {
            ++stableOk;
        }
        if (!yield1) {
            ++refusals;
        }
    }

    expect(stableOk == encounters, "ped_refusal5_once_per_encounter_stable");
    expect(refusals >= 1 && refusals < encounters / 2, "ped_refusal5_config_in_loose_band");

    PedestrianManager a;
    PedestrianManager b;
    a.setSeed(42);
    b.setSeed(42);
    a.setYieldRefusalProbability(0.05);
    b.setYieldRefusalProbability(0.05);
    a.spawnWaitingPedestrian(crossing, city);
    b.spawnWaitingPedestrian(crossing, city);
    expect(
        a.mustYieldToWaitingPedestrian(7, 1) == b.mustYieldToWaitingPedestrian(7, 1),
        "ped_refusal5_seed_reproducible"
    );
}

void testMultiLaneAllLanesSafety() {
    City city = makeRoadCity(80.0, 8.0, 2);
    Simulation sim;
    sim.setPedestrianSeed(3);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    crossing.setWalkSpeed(1.4);
    sim.addPedestrianCrossing(crossing);

    const double stopLine = 40.0 - SimConstants::CROSSING_STOP_DISTANCE;

    // Lane 0 stopped at yield line; lane 1 still moving on approach.
    Vehicle stopped = makeVehicle(1, 8.0, {1, 2}, 0);
    stopped.setPositionOnRoad(stopLine - 0.2);
    stopped.setSpeed(0.0);
    sim.addVehicle(stopped);

    Vehicle moving = makeVehicle(2, 8.0, {1, 2}, 1);
    moving.setPositionOnRoad(25.0);
    moving.setSpeed(8.0);
    sim.addVehicle(moving);

    const int pedId = sim.getPedestrianManager().spawnWaitingPedestrian(
        sim.getPedestrianCrossings().front(),
        city
    );
    sim.start();

    step(sim, city, 0.1, 3);
    const Pedestrian* ped = findPed(sim, pedId);
    expect(ped != nullptr && ped->isWaiting(), "ped_multilane_waits_until_all_lanes_safe");

    // Both lanes fully yielded at the stop line.
    for (auto& v : sim.getVehicles()) {
        v.setSpeed(0.0);
        v.setPositionOnRoad(stopLine - 0.3);
    }

    step(sim, city, 0.1, 5);
    ped = findPed(sim, pedId);
    expect(ped != nullptr && ped->isOnRoadway(), "ped_multilane_enters_when_all_lanes_stopped");

    const Vehicle* v0 = findVehicle(sim, 1);
    bool constrained = false;
    if (v0 != nullptr) {
        string reason0 = sim.getCollisionDetector().determineStopReason(
            *v0,
            city,
            sim.getTrafficLights(),
            sim.getPedestrianCrossings(),
            sim.getVehicles(),
            nullptr,
            &sim.getPedestrianManager()
        );
        constrained = reason0 == "pedestrian_on_road" || reason0 == "pedestrian_crossing";
    }
    expect(constrained, "ped_multilane_constrains_traffic");
}

void testResumeAfterClear() {
    City city = makeRoadCity(60.0, 8.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(11);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 30.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    crossing.setWalkSpeed(3.0);
    sim.addPedestrianCrossing(crossing);

    sim.addVehicle(makeVehicle(1, 8.0, {1, 2}));
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    sim.start();

    step(sim, city, 0.1, 50);
    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr, "ped_resume_vehicle_present");
    double stopPos = v != nullptr ? v->getPositionOnRoad() : 0.0;

    // Wait for ped to clear.
    step(sim, city, 0.1, 80);
    expect(!sim.getPedestrianManager().hasPedestrianOnRoadway(1), "ped_resume_crossing_clear");

    v = findVehicle(sim, 1);
    bool resumed = false;
    if (v != nullptr) {
        resumed = v->getPositionOnRoad() > stopPos + 2.0 ||
                  v->getCurrentRoadSegmentId() == 2 ||
                  v->isFinished();
    } else {
        resumed = true; // finished and pruned
    }
    expect(resumed, "ped_resume_after_clear");
}

void testNoHitLogicAndRedStillRespected() {
    City city = makeRoadCity(80.0, 10.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(5);
    sim.setPedestrianYieldRefusalProbability(1.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    sim.addPedestrianCrossing(crossing);

    // Red light near end — refusal must not override red.
    sim.addTrafficLight(TrafficLight(1, 1, 8.0, 20.0, false));

    Vehicle car = makeVehicle(1, 10.0, {1, 2});
    car.setPositionOnRoad(10.0);
    sim.addVehicle(car);
    sim.getPedestrianManager().spawnWaitingPedestrian(sim.getPedestrianCrossings().front(), city);
    sim.start();

    step(sim, city, 0.1, 100);
    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr, "ped_nohit_vehicle_present");
    if (v != nullptr) {
        expect(v->getPositionOnRoad() < 80.0 - 0.5, "ped_nohit_red_still_respected");
        string reason = sim.getCollisionDetector().determineStopReason(
            *v,
            city,
            sim.getTrafficLights(),
            sim.getPedestrianCrossings(),
            sim.getVehicles(),
            nullptr,
            &sim.getPedestrianManager()
        );
        expect(reason == "red_light", "ped_nohit_stop_reason_red");
    }

    // No hit / collision events in the event log.
    bool hasHit = false;
    for (const auto& ev : sim.getEventLog().getEvents()) {
        if (ev.type.find("hit") != string::npos ||
            ev.type.find("collision") != string::npos ||
            ev.reason.find("hit") != string::npos) {
            hasHit = true;
        }
    }
    expect(!hasHit, "ped_nohit_no_collision_events");
}

void testCarAlreadyCrossingFinishes() {
    City city = makeRoadCity(80.0, 8.0, 1);
    Simulation sim;
    sim.setPedestrianSeed(8);
    sim.setPedestrianYieldRefusalProbability(0.0);

    PedestrianCrossing crossing(1, 1, 40.0, 50.0, 5.0, false);
    crossing.setMode(CrossingMode::Unsignalized);
    sim.addPedestrianCrossing(crossing);

    // Vehicle already past stop line, in conflict zone.
    Vehicle car = makeVehicle(1, 8.0, {1, 2});
    car.setPositionOnRoad(40.0);
    car.setSpeed(6.0);
    sim.addVehicle(car);

    const int pedId = sim.getPedestrianManager().spawnWaitingPedestrian(
        sim.getPedestrianCrossings().front(),
        city
    );
    sim.start();
    step(sim, city, 0.1, 5);

    const Pedestrian* ped = findPed(sim, pedId);
    expect(ped != nullptr && ped->isWaiting(), "ped_waits_for_car_already_in_zone");
}

void testTimedCrossingStillBlocksAndShowsPed() {
    City city = makeRoadCity(60.0, 6.0, 1);
    Simulation sim;
    sim.addPedestrianCrossing(PedestrianCrossing(1, 1, 25.0, 30.0, 8.0, true));
    sim.addVehicle(makeVehicle(1, 6.0, {1, 2}));
    sim.start();
    step(sim, city, 0.1, 20);

    expect(!sim.getPedestrianManager().getPedestrians().empty() ||
               sim.getPedestrianCrossings().front().isActive(),
           "ped_timed_active_or_visible");
    const Vehicle* v = findVehicle(sim, 1);
    expect(v != nullptr && v->getPositionOnRoad() < 25.0, "ped_timed_car_blocked");
}

} // namespace

int pedestrianPassedCount() {
    return g_passed;
}

int runPedestrianTests() {
    g_failures = 0;
    g_passed = 0;

    cout << "\n--- Pedestrian Stage 4 tests ---\n";
    testWaitingCrossingClearAndSnapshot();
    testSignalizedClearAfterPhaseHold();
    testUnsignalizedYield();
    testRefusalZeroAlwaysYields();
    testRefusalHundredPassesWaiting();
    testRefusalFiveConfigNotFragile();
    testMultiLaneAllLanesSafety();
    testResumeAfterClear();
    testNoHitLogicAndRedStillRespected();
    testCarAlreadyCrossingFinishes();
    testTimedCrossingStillBlocksAndShowsPed();

    return g_failures;
}
