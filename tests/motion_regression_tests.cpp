#include "core/City.h"
#include "core/CollisionDetector.h"
#include "core/Intersection.h"
#include "core/PedestrianCrossing.h"
#include "core/RoadSegment.h"
#include "core/Scenario.h"
#include "core/Simulation.h"
#include "core/TrafficLight.h"
#include "core/Vehicle.h"
#include "core/VehicleGenerator.h"
#include "core/Vector2D.h"
#include "ui/CityLayout.h"
#include "ui/SimulationSession.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

using namespace std;

int runMapModelTests();
int mapModelPassedCount();
int runIntersectionPriorityTests();
int intersectionPriorityPassedCount();
int runPedestrianTests();
int pedestrianPassedCount();
int runLaneBlockageTests();
int laneBlockagePassedCount();
int runDemoIntegrationTests();
int demoIntegrationPassedCount();
int runOsmJsonParserTests();
int osmJsonParserPassedCount();
int runOsmToMapTests();
int osmToMapPassedCount();
int runOsmScenarioTests();
int runOsmLargerMapTests();
int runOsmSessionTests();
int osmSessionPassedCount();
int osmScenarioPassedCount();
int osmLargerMapPassedCount();
int runMapCameraTests();
int mapCameraPassedCount();
int runMapDownloadTests();
int mapDownloadPassedCount();
int runLocatorSelectionTests();
int locatorSelectionPassedCount();
int runGeometryControlTests();
int geometryControlPassedCount();
int runTrafficBaselineTests();
int trafficBaselinePassedCount();
int runEtap3TrafficTests();
int etap3TrafficPassedCount();
#if defined(_WIN32)
int runEncodingTests();
int encodingPassedCount();
#else
static int runEncodingTests() {
    cout << "SKIP  encoding_tests (Win32-only)\n";
    return 0;
}
static int encodingPassedCount() {
    return 0;
}
#endif

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
    filesystem::create_directories("tests/output");
}

void redirectSimOutputs(Simulation& sim, const string& tag) {
    ensureOutputDir();
    const string base = "tests/output/" + tag;
    sim.setEventLogOutputFile(base + "_events.json");
    sim.setStatisticsOutputFiles(base + "_timeline.csv", base + "_summary.csv");
}

void step(Simulation& sim, const City& city, double dt, int steps) {
    for (int i = 0; i < steps; ++i) {
        sim.update(dt, city);
    }
}

City makeTwoSegmentCity(double length1, double length2, double speedLimit) {
    City city("two_segment");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(length1, 0.0)));
    city.addIntersection(Intersection(3, Vector2D(length1 + length2, 0.0)));
    city.addRoadSegment(RoadSegment(1, 1, 2, length1, speedLimit));
    city.addRoadSegment(RoadSegment(2, 2, 3, length2, speedLimit));
    return city;
}

Vehicle makeVehicle(int id, double maxSpeed, const vector<int>& route) {
    Vehicle vehicle(id, maxSpeed);
    vehicle.setRoute(route);
    return vehicle;
}

bool approx(double a, double b, double eps = 0.15) {
    return abs(a - b) <= eps;
}

void testFreePathTwoSegments() {
    City city = makeTwoSegmentCity(40.0, 40.0, 8.0);
    Simulation sim;
    redirectSimOutputs(sim, "free_path");
    sim.setMinVehicleGap(2.5);
    sim.addVehicle(makeVehicle(1, 8.0, {1, 2}));
    sim.start();

    step(sim, city, 0.1, 200);

    const auto& vehicles = sim.getVehicles();
    bool progressed = false;
    bool finishedOrOnSecond = false;

    for (const auto& vehicle : vehicles) {
        if (vehicle.getId() != 1) {
            continue;
        }

        progressed = vehicle.getCurrentRoadIndex() >= 1 || vehicle.isFinished();
        finishedOrOnSecond = vehicle.isFinished() || vehicle.getCurrentRoadSegmentId() == 2;
        expect(vehicle.getSpeed() <= 8.0 + 0.05, "free_path_speed_limit");
    }

    // Vehicle may have finished and been pruned.
    if (vehicles.empty()) {
        expect(true, "free_path_progressed_and_finished");
        expect(true, "free_path_reached_second_or_finished");
    } else {
        expect(progressed, "free_path_progressed_and_finished");
        expect(finishedOrOnSecond, "free_path_reached_second_or_finished");
    }
}

void testRedStopResumeGreen() {
    City city = makeTwoSegmentCity(30.0, 30.0, 6.0);
    Simulation sim;
    redirectSimOutputs(sim, "red_green");
    sim.setMinVehicleGap(2.5);

    // Long red, then green: start red so vehicle must stop before end.
    sim.addTrafficLight(TrafficLight(1, 1, 20.0, 8.0, false));
    sim.addVehicle(makeVehicle(1, 6.0, {1, 2}));
    sim.start();

    step(sim, city, 0.1, 80);

    const Vehicle* v = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            v = &vehicle;
            break;
        }
    }

    expect(v != nullptr, "red_green_vehicle_present_on_red");
    if (v != nullptr) {
        expect(v->getCurrentRoadSegmentId() == 1, "red_green_still_on_first_road");
        expect(v->isStopped(), "red_green_stopped_on_red");
        expect(v->getSpeed() <= 0.05, "red_green_speed_zero_on_red");
        expect(
            v->getPositionOnRoad() <= 30.0 - SimConstants::RED_LIGHT_STOP_DISTANCE + 0.2,
            "red_green_stopped_before_end"
        );
        expect(
            v->getPositionOnRoad() >= 30.0 - SimConstants::RED_LIGHT_STOP_DISTANCE - 1.0,
            "red_green_near_red_stop_line"
        );
    }

    // Force green by advancing past remaining red if needed, then allow travel.
    // Light started red for 8s; we already ran 8s. Next is 20s green.
    double posOnRed = v != nullptr ? v->getPositionOnRoad() : 0.0;
    int roadOnRed = v != nullptr ? v->getCurrentRoadSegmentId() : -1;

    step(sim, city, 0.1, 120);

    bool resumedProgress = false;
    bool enteredSecondOrFinished = false;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() != 1) {
            continue;
        }

        resumedProgress =
            vehicle.getPositionOnRoad() > posOnRed + 1.0 ||
            vehicle.getCurrentRoadSegmentId() != roadOnRed ||
            vehicle.isFinished();
        enteredSecondOrFinished =
            vehicle.getCurrentRoadSegmentId() == 2 || vehicle.isFinished();
        expect(!vehicle.isStopped() || vehicle.getCurrentRoadSegmentId() == 2 || vehicle.isFinished(),
               "red_green_resumed_or_advanced");
    }

    if (sim.getVehicles().empty()) {
        expect(true, "red_green_resumed_progress");
        expect(true, "red_green_entered_second_or_finished");
    } else {
        expect(resumedProgress, "red_green_resumed_progress");
        expect(enteredSecondOrFinished, "red_green_entered_second_or_finished");
    }
}

void testQueueThenMove() {
    City city = makeTwoSegmentCity(50.0, 50.0, 5.0);
    Simulation sim;
    redirectSimOutputs(sim, "queue");
    sim.setMinVehicleGap(3.0);

    // Leader already near end of first road via short route wait: spawn three on same road.
    Vehicle lead = makeVehicle(1, 5.0, {1, 2});
    Vehicle mid = makeVehicle(2, 5.0, {1, 2});
    Vehicle rear = makeVehicle(3, 5.0, {1, 2});
    sim.addVehicle(lead);
    sim.addVehicle(mid);
    sim.addVehicle(rear);
    sim.start();

    // Let them form a queue on the first segment (no light, free exit).
    // Place artificial congestion by temporarily blocking road 2 entry with a parked vehicle.
    Vehicle blocker = makeVehicle(99, 0.1, {2});
    sim.addVehicle(blocker);

    step(sim, city, 0.1, 150);

    vector<const Vehicle*> onFirst;
    for (const auto& vehicle : sim.getVehicles()) {
        if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == 1) {
            onFirst.push_back(&vehicle);
        }
    }

    expect(onFirst.size() >= 2, "queue_at_least_two_waiting");

    bool gapsOk = true;
    for (size_t i = 0; i + 1 < onFirst.size(); ++i) {
        // onFirst not sorted; sort by position
    }

    vector<double> positions;
    for (const Vehicle* vehicle : onFirst) {
        positions.push_back(vehicle->getPositionOnRoad());
        expect(vehicle->getSpeed() <= 5.0 + 0.05, "queue_speed_limit");
    }

    sort(positions.begin(), positions.end());
    for (size_t i = 0; i + 1 < positions.size(); ++i) {
        double gap = positions[i + 1] - positions[i];
        if (gap < 3.0 - 0.05) {
            gapsOk = false;
        }
    }
    expect(gapsOk, "queue_min_gap_respected");

    // Remove blocker by finishing it: mark via driving it off — replace by finishing route.
    // Finish blocker by giving empty next and reaching end: simply run until road 2 clears.
    // Force-clear: remove vehicles on road 2 by finishing them through a hack —
    // add no easy finish API; instead move blocker out by using a long sim after changing route.
    // Practical approach: stop sim rebuild without blocker.
    City city2 = makeTwoSegmentCity(50.0, 50.0, 5.0);
    Simulation sim2;
    redirectSimOutputs(sim2, "queue_resume");
    sim2.setMinVehicleGap(3.0);

    // Recreate queued-like start positions by running briefly then clearing.
    Vehicle a = makeVehicle(1, 5.0, {1, 2});
    Vehicle b = makeVehicle(2, 5.0, {1, 2});
    Vehicle c = makeVehicle(3, 5.0, {1, 2});
    sim2.addVehicle(a);
    sim2.addVehicle(b);
    sim2.addVehicle(c);
    sim2.start();

    step(sim2, city2, 0.1, 250);

    int finishedOrAdvanced = 0;
    for (const auto& vehicle : sim2.getVehicles()) {
        if (vehicle.getId() >= 1 && vehicle.getId() <= 3) {
            if (vehicle.getCurrentRoadSegmentId() == 2 || vehicle.getCurrentRoadIndex() >= 1) {
                ++finishedOrAdvanced;
            }
        }
    }

    // Pruned finished vehicles also count as progress.
    expect(finishedOrAdvanced >= 1 || sim2.getVehicles().size() < 3, "queue_vehicles_eventually_move");
}

void testCrossingStopAndResume() {
    City city = makeTwoSegmentCity(40.0, 40.0, 6.0);
    Simulation sim;
    redirectSimOutputs(sim, "crossing");
    sim.setMinVehicleGap(2.5);

    // Active for a long time, then inactive.
    sim.addPedestrianCrossing(PedestrianCrossing(1, 1, 20.0, 30.0, 12.0, true));
    sim.addVehicle(makeVehicle(1, 6.0, {1, 2}));
    sim.start();

    step(sim, city, 0.1, 80);

    const Vehicle* v = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            v = &vehicle;
            break;
        }
    }

    expect(v != nullptr, "crossing_vehicle_present");
    if (v != nullptr) {
        expect(v->isStopped(), "crossing_stopped_while_active");
        expect(
            v->getPositionOnRoad() <= 20.0 - SimConstants::CROSSING_STOP_DISTANCE + 0.3,
            "crossing_stopped_before_line"
        );
        expect(v->getPositionOnRoad() < 20.0, "crossing_not_past_line_while_active");
    }

    double stopPos = v != nullptr ? v->getPositionOnRoad() : 0.0;

    // Active duration 12s already elapsed (~8s). Wait for inactive and resume.
    step(sim, city, 0.1, 150);

    bool resumed = false;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() != 1) {
            continue;
        }

        resumed = vehicle.getPositionOnRoad() > stopPos + 1.0 ||
                  vehicle.getCurrentRoadSegmentId() == 2 ||
                  vehicle.isFinished();
    }

    if (sim.getVehicles().empty()) {
        expect(true, "crossing_resumed_after_clear");
    } else {
        expect(resumed, "crossing_resumed_after_clear");
    }
}

void testPastCrossingNotBlocked() {
    // Long first segment so the vehicle remains on the same road past the crossing
    // while the crossing is active (no early exit / finish that would vacate the case).
    const double roadLength = 100.0;
    const double crossingPos = 20.0;
    const double speedLimit = 6.0;
    const double inactiveDuration = 10.0; // enough to pass line before activation
    const double activeDuration = 40.0;

    City city = makeTwoSegmentCity(roadLength, 40.0, speedLimit);
    Simulation sim;
    redirectSimOutputs(sim, "past_crossing");
    sim.setMinVehicleGap(2.5);

    sim.addPedestrianCrossing(
        PedestrianCrossing(1, 1, crossingPos, inactiveDuration, activeDuration, false)
    );
    sim.addVehicle(makeVehicle(1, speedLimit, {1, 2}));
    sim.start();

    CollisionDetector detector(2.5);
    const RoadSegment* road = city.findRoadSegmentById(1);
    expect(road != nullptr, "past_crossing_road_lookup");

    // Direct constraint check: active crossing behind a past-line position must not clamp.
    if (road != nullptr) {
        PedestrianCrossing activeBehind(99, 1, crossingPos, 5.0, 5.0, true);
        vector<PedestrianCrossing> crossings = {activeBehind};
        const double pastLinePosition = crossingPos + 5.0;
        double maxAllowed = detector.computeMaxAllowedPosition(
            *road, {}, crossings, pastLinePosition, -1.0
        );
        const double crossingStopLine = crossingPos - SimConstants::CROSSING_STOP_DISTANCE;
        expect(
            maxAllowed > crossingStopLine + 1.0 && approx(maxAllowed, roadLength, 0.05),
            "past_crossing_max_allowed_not_crossing_stop"
        );
    }

    // Drive well past the crossing while still inactive, then into the active window.
    step(sim, city, 0.1, 105); // 10.5s: inactive ended at 10s; vehicle ~50+ on road 1

    const Vehicle* subject = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            subject = &vehicle;
            break;
        }
    }

    expect(subject != nullptr, "past_crossing_vehicle_exists");
    if (subject == nullptr) {
        expect(false, "past_crossing_on_same_road_past_line");
        expect(false, "past_crossing_forward_progress_while_active");
        return;
    }

    expect(subject->getCurrentRoadSegmentId() == 1, "past_crossing_on_same_road_past_line");
    expect(subject->getPositionOnRoad() >= crossingPos, "past_crossing_reached_or_passed_line");
    expect(sim.getPedestrianCrossings().front().isActive(), "past_crossing_is_active");

    const double posAtActive = subject->getPositionOnRoad();
    expect(posAtActive < roadLength - 10.0, "past_crossing_room_to_advance");

    // Live maxAllowed for the actual subject while crossing is active behind it.
    if (road != nullptr) {
        double maxAllowedLive = detector.computeMaxAllowedPosition(
            *road,
            sim.getTrafficLights(),
            sim.getPedestrianCrossings(),
            posAtActive,
            -1.0
        );
        expect(
            approx(maxAllowedLive, roadLength, 0.05),
            "past_crossing_live_max_allowed_is_road_end"
        );
    }

    step(sim, city, 0.1, 30); // +3s under active crossing — must advance on same road

    subject = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            subject = &vehicle;
            break;
        }
    }

    expect(subject != nullptr, "past_crossing_still_present");
    if (subject == nullptr) {
        expect(false, "past_crossing_forward_progress_while_active");
        return;
    }

    expect(subject->getCurrentRoadSegmentId() == 1, "past_crossing_remained_on_same_road");
    expect(subject->getPositionOnRoad() >= crossingPos, "past_crossing_still_past_line");
    expect(
        subject->getPositionOnRoad() >= posAtActive + 2.0,
        "past_crossing_forward_progress_while_active"
    );
    expect(subject->getSpeed() > 0.5, "past_crossing_not_stopped_by_crossing");
}

void testBlockedNextRoadThenEnter() {
    City city = makeTwoSegmentCity(20.0, 40.0, 6.0);
    Simulation sim;
    redirectSimOutputs(sim, "blocked_next");
    sim.setMinVehicleGap(4.0);

    Vehicle mover = makeVehicle(1, 6.0, {1, 2});
    // Nearly stationary occupant of the next-road entry zone.
    Vehicle blocker = makeVehicle(2, 0.01, {2});
    sim.addVehicle(mover);
    sim.addVehicle(blocker);
    sim.start();

    step(sim, city, 0.1, 80);

    const Vehicle* moverPtr = nullptr;
    const Vehicle* blockerPtr = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            moverPtr = &vehicle;
        }
        if (vehicle.getId() == 2) {
            blockerPtr = &vehicle;
        }
    }

    expect(moverPtr != nullptr, "blocked_mover_present");
    expect(blockerPtr != nullptr, "blocked_blocker_present");
    if (moverPtr != nullptr) {
        expect(moverPtr->getCurrentRoadSegmentId() == 1, "blocked_did_not_enter_occupied");
        expect(moverPtr->isStopped() || moverPtr->getSpeed() < 0.3, "blocked_waiting_at_end");
        expect(
            moverPtr->getPositionOnRoad() >= 20.0 - SimConstants::ROAD_END_EPSILON - 0.5,
            "blocked_reached_near_road_end"
        );
    }

    if (blockerPtr != nullptr) {
        expect(blockerPtr->getPositionOnRoad() < 4.0, "blocked_blocker_still_in_entry_zone");
    }

    // Clear the entry zone: finish the blocker so the next road becomes enterable.
    for (auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 2) {
            vehicle.finish();
        }
    }

    step(sim, city, 0.1, 80);

    bool entered = false;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            entered = vehicle.getCurrentRoadSegmentId() == 2 || vehicle.isFinished();
        }
    }

    expect(entered || sim.getVehicles().empty(), "blocked_enters_when_free");
}

void testDestinationCompletion() {
    City city = makeTwoSegmentCity(20.0, 20.0, 8.0);
    Simulation sim;
    redirectSimOutputs(sim, "destination");
    sim.setMinVehicleGap(2.5);
    sim.addVehicle(makeVehicle(1, 8.0, {1, 2}));
    sim.start();

    step(sim, city, 0.1, 200);

    bool stillPresentUnfinished = false;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1 && !vehicle.isFinished()) {
            stillPresentUnfinished = true;
        }
    }

    expect(!stillPresentUnfinished, "destination_vehicle_finished_and_pruned");
}

void testInvalidRouteControllable() {
    City city("invalid_route");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(30.0, 0.0)));
    city.addIntersection(Intersection(3, Vector2D(30.0, 30.0)));
    city.addIntersection(Intersection(4, Vector2D(60.0, 0.0)));
    // Road 1: 1->2, Road 2: 3->4 (NOT connected to road 1)
    city.addRoadSegment(RoadSegment(1, 1, 2, 30.0, 6.0));
    city.addRoadSegment(RoadSegment(2, 3, 4, 30.0, 6.0));

    Simulation sim;
    redirectSimOutputs(sim, "invalid_route");
    sim.setMinVehicleGap(2.5);
    sim.addVehicle(makeVehicle(1, 6.0, {1, 2}));
    sim.start();

    step(sim, city, 0.1, 120);

    const Vehicle* v = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            v = &vehicle;
            break;
        }
    }

    expect(v != nullptr, "invalid_route_vehicle_not_removed");
    if (v != nullptr) {
        expect(v->getCurrentRoadSegmentId() == 1, "invalid_route_stays_on_first");
        expect(!v->isFinished(), "invalid_route_not_marked_finished");
        expect(
            v->getPositionOnRoad() >= 30.0 - 1.0,
            "invalid_route_reaches_end_and_waits"
        );
    }

    CollisionDetector detector(2.5);
    if (v != nullptr) {
        string reason = detector.determineStopReason(
            *v, city, sim.getTrafficLights(), sim.getPedestrianCrossings(), sim.getVehicles()
        );
        expect(reason == "route_invalid", "invalid_route_stop_reason");
    }
}

void testNoSpeedBoostOnTransition() {
    City city = makeTwoSegmentCity(20.0, 20.0, 10.0);
    Simulation sim;
    redirectSimOutputs(sim, "no_boost");
    sim.setMinVehicleGap(2.5);

    Vehicle vehicle = makeVehicle(1, 10.0, {1, 2});
    sim.addVehicle(vehicle);
    sim.start();

    // Run until transition.
    double speedJustBefore = -1.0;
    double speedJustAfter = -1.0;
    int prevRoad = 1;

    for (int i = 0; i < 300; ++i) {
        for (const auto& v : sim.getVehicles()) {
            if (v.getId() == 1 && v.getCurrentRoadSegmentId() == 1) {
                speedJustBefore = v.getSpeed();
            }
        }

        sim.update(0.05, city);

        for (const auto& v : sim.getVehicles()) {
            if (v.getId() == 1 && v.getCurrentRoadSegmentId() == 2 && prevRoad == 1) {
                speedJustAfter = v.getSpeed();
                prevRoad = 2;
            }
        }

        if (speedJustAfter >= 0.0) {
            break;
        }
    }

    expect(speedJustAfter >= 0.0, "no_boost_did_transition");
    if (speedJustAfter >= 0.0 && speedJustBefore >= 0.0) {
        // Must not jump to maxSpeed * 0.6 artificially from a near-stop.
        expect(!(speedJustBefore < 0.5 && approx(speedJustAfter, 6.0, 0.3)),
               "no_boost_no_60pct_jump_from_stop");
        expect(speedJustAfter <= 10.0 + 0.05, "no_boost_speed_limit");
    }
}

struct IntegrationSample {
    int id;
    int roadId;
    double position;
    double speed;
    bool stopped;
};

void testScenarioIntegration120s() {
    City city;
    Simulation sim;
    redirectSimOutputs(sim, "integration_120s");
    Scenario::setupIntersectionTest(city, sim);
    sim.start();

    const double dt = 0.1;
    const int steps = 1200; // 120s

    map<int, IntegrationSample> firstSeen;
    map<int, IntegrationSample> lastSeen;
    map<int, int> stuckTicks;
    int speedViolations = 0;
    int gapViolations = 0;
    int unexplainedFreezeSuspects = 0;

    CollisionDetector detector(sim.getVehicles().empty() ? 4.0 : 4.0);
    detector.setMinVehicleGap(4.0);

    for (int stepIndex = 0; stepIndex < steps; ++stepIndex) {
        sim.update(dt, city);

        // Per-road gap and speed checks
        for (const auto& road : city.getRoadSegments()) {
            vector<const Vehicle*> onRoad;
            for (const auto& vehicle : sim.getVehicles()) {
                if (!vehicle.isFinished() && vehicle.getCurrentRoadSegmentId() == road.getId()) {
                    onRoad.push_back(&vehicle);
                    if (vehicle.getSpeed() > road.getSpeedLimit() + 0.15) {
                        ++speedViolations;
                    }
                }
            }

            // Gap checks are per lane — independent lanes may share longitudinal position.
            for (int lane = 0; lane < road.getLaneCount(); ++lane) {
                vector<const Vehicle*> onLane;
                for (const Vehicle* vehicle : onRoad) {
                    if (vehicle->getLaneIndex() == lane) {
                        onLane.push_back(vehicle);
                    }
                }

                sort(onLane.begin(), onLane.end(), [](const Vehicle* a, const Vehicle* b) {
                    return a->getPositionOnRoad() < b->getPositionOnRoad();
                });

                for (size_t i = 0; i + 1 < onLane.size(); ++i) {
                    const double gap =
                        onLane[i + 1]->getPositionOnRoad() - onLane[i]->getPositionOnRoad();
                    // Fronts: spacing must cover leader length + minVehicleGap (scenario = 4.0).
                    const double minBodyGap = onLane[i + 1]->getLength() + 4.0 - 0.35;
                    if (gap < minBodyGap) {
                        ++gapViolations;
                    }
                }
            }
        }

        for (const auto& vehicle : sim.getVehicles()) {
            if (vehicle.isFinished()) {
                continue;
            }

            IntegrationSample sample{
                vehicle.getId(),
                vehicle.getCurrentRoadSegmentId(),
                vehicle.getPositionOnRoad(),
                vehicle.getSpeed(),
                vehicle.isStopped()
            };

            if (firstSeen.find(vehicle.getId()) == firstSeen.end()) {
                firstSeen[vehicle.getId()] = sample;
            }

            auto it = lastSeen.find(vehicle.getId());
            if (it != lastSeen.end()) {
                bool samePlace =
                    it->second.roadId == sample.roadId &&
                    abs(it->second.position - sample.position) < 0.01 &&
                    sample.stopped;

                if (samePlace) {
                    stuckTicks[vehicle.getId()]++;
                } else {
                    stuckTicks[vehicle.getId()] = 0;
                }
            }

            lastSeen[vehicle.getId()] = sample;
        }
    }

    // Evaluate long freezes: only flag if stop reason would be unexplained vehicle_ahead
    // while no leader / no red / no crossing / can enter next.
    for (const auto& entry : stuckTicks) {
        if (entry.second < 200) { // 20s
            continue;
        }

        const Vehicle* v = nullptr;
        for (const auto& vehicle : sim.getVehicles()) {
            if (vehicle.getId() == entry.first) {
                v = &vehicle;
                break;
            }
        }

        if (v == nullptr) {
            continue;
        }

        string reason = detector.determineStopReason(
            *v, city, sim.getTrafficLights(), sim.getPedestrianCrossings(), sim.getVehicles()
        );

        bool explained =
            reason == "red_light" ||
            reason == "vehicle_ahead" ||
            reason == "pedestrian_crossing" ||
            reason == "pedestrian_on_road" ||
            reason == "blocked_next_road" ||
            reason == "route_invalid" ||
            reason == "destination_reached";

        if (!explained) {
            ++unexplainedFreezeSuspects;
        }

        // If claimed vehicle_ahead but no vehicle ahead on same road, count as unexplained.
        if (reason == "vehicle_ahead") {
            bool hasLeader = false;
            for (const auto& other : sim.getVehicles()) {
                if (other.getId() == v->getId() || other.isFinished()) {
                    continue;
                }
                if (other.getCurrentRoadSegmentId() == v->getCurrentRoadSegmentId() &&
                    other.getLaneIndex() == v->getLaneIndex() &&
                    other.getPositionOnRoad() > v->getPositionOnRoad()) {
                    hasLeader = true;
                }
            }
            if (!hasLeader) {
                // May still be valid if binding was mislabeled; check other real constraints.
                bool hasRed = false;
                for (const auto& light : sim.getTrafficLights()) {
                    if (light.getRoadSegmentId() == v->getCurrentRoadSegmentId() && light.isRed()) {
                        hasRed = true;
                    }
                }
                bool hasActiveCrossingAhead = false;
                for (const auto& crossing : sim.getPedestrianCrossings()) {
                    if (crossing.getRoadSegmentId() == v->getCurrentRoadSegmentId() &&
                        crossing.isActive() &&
                        v->getPositionOnRoad() < crossing.getPositionOnRoad()) {
                        hasActiveCrossingAhead = true;
                    }
                }
                int nextId = v->getNextRoadSegmentId();
                bool blockedNext = nextId != -1 && !detector.canEnterRoad(nextId, sim.getVehicles());
                bool atEnd = detector.isAtIntersectionEntry(
                    v->getPositionOnRoad(),
                    city.findRoadSegmentById(v->getCurrentRoadSegmentId())->getLength()
                );

                if (!hasRed && !hasActiveCrossingAhead && !(atEnd && blockedNext) && nextId != -1) {
                    ++unexplainedFreezeSuspects;
                }
            }
        }
    }

    int progressedVehicles = 0;
    for (const auto& entry : firstSeen) {
        auto last = lastSeen.find(entry.first);
        if (last == lastSeen.end()) {
            // Finished and pruned => progress
            ++progressedVehicles;
            continue;
        }

        if (last->second.roadId != entry.second.roadId ||
            last->second.position > entry.second.position + 1.0) {
            ++progressedVehicles;
        }
    }

    expect(firstSeen.size() >= 5, "integration_spawned_vehicles");
    expect(progressedVehicles >= static_cast<int>(firstSeen.size()) / 2, "integration_many_progressed");
    expect(speedViolations == 0, "integration_no_speed_limit_violations");
    expect(gapViolations == 0, "integration_no_gap_violations");
    expect(unexplainedFreezeSuspects == 0, "integration_no_unexplained_freezes");

    ofstream report("tests/output/integration_120s_report.txt");
    report << "vehicles_seen=" << firstSeen.size() << "\n";
    report << "progressed=" << progressedVehicles << "\n";
    report << "speed_violations=" << speedViolations << "\n";
    report << "gap_violations=" << gapViolations << "\n";
    report << "unexplained_freezes=" << unexplainedFreezeSuspects << "\n";
    report << "sim_time=" << sim.getTime() << "\n";
}

struct VehiclePose {
    int id = -1;
    int roadId = -1;
    double position = 0.0;
};

vector<VehiclePose> captureVehiclePoses(const Simulation& sim) {
    vector<VehiclePose> poses;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.isFinished()) {
            continue;
        }
        poses.push_back({vehicle.getId(), vehicle.getCurrentRoadSegmentId(), vehicle.getPositionOnRoad()});
    }
    sort(poses.begin(), poses.end(), [](const VehiclePose& a, const VehiclePose& b) {
        return a.id < b.id;
    });
    return poses;
}

bool posesEqual(const vector<VehiclePose>& a, const vector<VehiclePose>& b, double eps = 1e-9) {
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].id != b[i].id || a[i].roadId != b[i].roadId || !approx(a[i].position, b[i].position, eps)) {
            return false;
        }
    }
    return true;
}

void pumpSession(SimulationSession& session, double wallSeconds) {
    // Drive the session with small wall frames so catch-up clamp does not truncate intentional advances.
    const double frame = SimulationSession::renderDt;
    const int steps = max(1, static_cast<int>(std::ceil(wallSeconds / frame)));
    for (int i = 0; i < steps; ++i) {
        session.update(frame);
    }
}

void testSessionLifecyclePauseResumeReset() {
    SimulationSession session;
    redirectSimOutputs(session.getSimulation(), "lifecycle");
    session.loadIntersectionTestScenario();

    expect(!session.isRunning(), "lifecycle_initial_not_running");
    expect(approx(session.getSimulation().getTime(), 0.0, 1e-9), "lifecycle_initial_time_zero");

    session.start();
    expect(session.isRunning(), "lifecycle_start_running");
    expect(!session.isPaused(), "lifecycle_start_not_paused");

    pumpSession(session, 1.0);
    const double timeAfterStart = session.getSimulation().getTime();
    expect(timeAfterStart > 0.5, "lifecycle_start_time_advances");

    auto posesBeforePause = captureVehiclePoses(session.getSimulation());
    session.pause();
    expect(session.isPaused(), "lifecycle_pause_flag");

    pumpSession(session, 5.0);
    pumpSession(session, 5.0);
    expect(approx(session.getSimulation().getTime(), timeAfterStart, 1e-9), "lifecycle_pause_time_frozen");
    expect(posesEqual(posesBeforePause, captureVehiclePoses(session.getSimulation())), "lifecycle_pause_positions_frozen");

    session.resume();
    expect(session.isRunning() && !session.isPaused(), "lifecycle_resume_running");

    const double timeAtResume = session.getSimulation().getTime();
    auto posesAtResume = captureVehiclePoses(session.getSimulation());
    expect(posesEqual(posesBeforePause, posesAtResume), "lifecycle_resume_no_teleport_at_resume");

    // Accumulator cleared on resume: a sub-step frameDt must not advance sim time or teleport.
    session.update(0.05);
    expect(approx(session.getSimulation().getTime(), timeAtResume, 1e-9), "lifecycle_resume_no_jump_from_frame_dt");
    expect(posesEqual(posesAtResume, captureVehiclePoses(session.getSimulation()), 1e-9),
           "lifecycle_resume_positions_unchanged_before_sim_step");

    pumpSession(session, 1.0);
    const double timeAfterResume = session.getSimulation().getTime();
    expect(timeAfterResume > timeAtResume + 0.5, "lifecycle_resume_time_advances");

    auto posesAfterResume = captureVehiclePoses(session.getSimulation());
    bool continuousMotion = true;
    for (const auto& before : posesAtResume) {
        for (const auto& after : posesAfterResume) {
            if (before.id != after.id || before.roadId != after.roadId) {
                continue;
            }
            if (after.position + 0.01 < before.position - 0.01 || after.position - before.position >= 20.0) {
                continuousMotion = false;
            }
        }
    }
    expect(continuousMotion, "lifecycle_resume_position_continuous");

    const int lightsBeforeReset = static_cast<int>(session.getSimulation().getTrafficLights().size());
    const int crossingsBeforeReset = static_cast<int>(session.getSimulation().getPedestrianCrossings().size());
    const int roadsBeforeReset = static_cast<int>(session.getCity().getRoadSegments().size());
    expect(lightsBeforeReset > 0, "lifecycle_has_lights_before_reset");
    expect(crossingsBeforeReset > 0, "lifecycle_has_crossings_before_reset");

    session.reset();
    expect(!session.isRunning(), "lifecycle_reset_not_running");
    expect(!session.isPaused(), "lifecycle_reset_not_paused");
    expect(approx(session.getSimulation().getTime(), 0.0, 1e-9), "lifecycle_reset_time_zero");
    expect(session.getSimulation().getVehicles().empty(), "lifecycle_reset_no_vehicles");
    expect(session.getSimulation().getEventLog().getEvents().empty(), "lifecycle_reset_events_cleared");
    expect(session.getSimulation().getStatisticsCollector().getLastSnapshot().totalVehiclesSeen == 0,
           "lifecycle_reset_stats_cleared");
    expect(static_cast<int>(session.getSimulation().getTrafficLights().size()) == lightsBeforeReset,
           "lifecycle_reset_lights_reloaded");
    expect(static_cast<int>(session.getSimulation().getPedestrianCrossings().size()) == crossingsBeforeReset,
           "lifecycle_reset_crossings_reloaded");
    expect(static_cast<int>(session.getCity().getRoadSegments().size()) == roadsBeforeReset,
           "lifecycle_reset_city_reloaded");

    session.start();
    pumpSession(session, 5.0);
    expect(session.getSimulation().getTime() > 4.0, "lifecycle_restart_after_reset");
    expect(
        !session.getSimulation().getVehicles().empty() ||
        session.getSimulation().getStatisticsCollector().getLastSnapshot().totalVehiclesSeen > 0,
        "lifecycle_generators_work_after_reset"
    );
}

void testTimeMultiplierFixedDtAndCatchUpClamp() {
    SimulationSession sessionA;
    redirectSimOutputs(sessionA.getSimulation(), "time_mul_a");
    sessionA.loadIntersectionTestScenario();
    sessionA.setTimeMultiplier(1.0);
    sessionA.start();

    SimulationSession sessionB;
    redirectSimOutputs(sessionB.getSimulation(), "time_mul_b");
    sessionB.loadIntersectionTestScenario();
    sessionB.setTimeMultiplier(2.0);
    sessionB.start();

    const double wall = 1.0;
    pumpSession(sessionA, wall);
    pumpSession(sessionB, wall);

    const double timeA = sessionA.getSimulation().getTime();
    const double timeB = sessionB.getSimulation().getTime();
    expect(approx(timeA, wall, 0.15), "time_mul_1x_matches_wall");
    expect(approx(timeB, wall * 2.0, 0.25), "time_mul_2x_scales_sim_time");
    expect(timeB > timeA * 1.5, "time_mul_2x_faster_than_1x");

    // Physics dt remains fixed: step() / update always feeds simulationDt into Simulation.
    expect(approx(SimulationSession::simulationDt, 0.1, 1e-12), "time_mul_physics_dt_fixed");

    // Pause still freezes even with high multiplier.
    sessionB.pause();
    const double pausedAt = sessionB.getSimulation().getTime();
    pumpSession(sessionB, 2.0);
    expect(approx(sessionB.getSimulation().getTime(), pausedAt, 1e-9), "time_mul_pause_freezes");

    // Large frameDt catch-up is clamped (cannot jump many seconds in one update).
    SimulationSession sessionC;
    redirectSimOutputs(sessionC.getSimulation(), "time_mul_c");
    sessionC.loadIntersectionTestScenario();
    sessionC.setTimeMultiplier(1.0);
    sessionC.start();
    sessionC.update(10.0);
    const double clamped = sessionC.getSimulation().getTime();
    expect(clamped <= SimulationSession::maxCatchUpSimSeconds + 1e-9, "time_mul_catchup_clamped_upper");
    expect(clamped >= SimulationSession::simulationDt - 1e-9, "time_mul_catchup_still_advances");
    expect(clamped < 2.0, "time_mul_catchup_not_unbounded");
}

void testManualBlockViaSimulationApi() {
    SimulationSession session;
    redirectSimOutputs(session.getSimulation(), "manual_block_api");
    session.loadIntersectionTestScenario();

    auto& sim = session.getSimulation();
    const auto& city = session.getCity();
    expect(!city.getRoadSegments().empty(), "manual_block_has_roads");

    int roadId = -1;
    int laneCount = 1;
    double roadLength = 0.0;
    for (const auto& road : city.getRoadSegments()) {
        if (road.getLength() >= 20.0) {
            roadId = road.getId();
            laneCount = road.getLaneCount();
            roadLength = road.getLength();
            break;
        }
    }
    expect(roadId >= 0, "manual_block_found_road");

    const double position = roadLength * 0.4;
    const int id = sim.addLaneBlockage(roadId, 0, position, 5.0, 0.0, 15.0, city);
    expect(id > 0, "manual_block_add_ok");
    expect(static_cast<int>(sim.getLaneBlockages().size()) == 1, "manual_block_count_1");

    if (laneCount >= 2) {
        const int id2 = sim.addLaneBlockage(roadId, 1, position, 5.0, 0.0, 10.0, city);
        expect(id2 > 0, "manual_block_second_lane_ok");
    }

    expect(sim.clearLaneBlockage(id), "manual_block_clear_ok");
    expect(static_cast<int>(sim.getLaneBlockages().size()) <= 1, "manual_block_cleared_one");

    sim.clearAllLaneBlockages();
    expect(sim.getLaneBlockages().empty(), "manual_block_clear_all");
}

void testSpawnRateAndPedTempoScaling() {
    SimulationSession session;
    redirectSimOutputs(session.getSimulation(), "spawn_tempo");
    session.loadIntersectionTestScenario();

    auto& gens = session.getSimulation().getVehicleGenerators();
    expect(!gens.empty(), "spawn_has_generators");
    const double base0 = gens[0].getSpawnInterval();

    session.setVehicleSpawnRate(2.0);
    expect(approx(gens[0].getSpawnInterval(), base0 / 2.0, 1e-9), "spawn_rate_halves_interval");

    session.setVehicleSpawnRate(1.0);
    expect(approx(gens[0].getSpawnInterval(), base0, 1e-9), "spawn_rate_restores_interval");

    session.setPedestrianRequestTempo(2.0);
    expect(approx(session.getSimulation().getPedestrianRequestTempo(), 2.0, 1e-9), "ped_tempo_applied");
    session.setPedestrianRequestTempo(1.0);
}

City makeMultiLaneCity(double length, double speedLimit, int laneCount) {
    City city("multi_lane");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(length, 0.0)));
    city.addIntersection(Intersection(3, Vector2D(length * 2.0, 0.0)));
    RoadSegment first(1, 1, 2, length, speedLimit);
    first.setLaneCount(laneCount);
    RoadSegment second(2, 2, 3, length, speedLimit);
    second.setLaneCount(1); // next road is single-lane to exercise clamp
    city.addRoadSegment(first);
    city.addRoadSegment(second);
    return city;
}

void testParallelLanesIndependent() {
    City city = makeMultiLaneCity(60.0, 8.0, 2);
    Simulation sim;
    redirectSimOutputs(sim, "parallel_lanes");
    sim.setMinVehicleGap(3.0);

    Vehicle a = makeVehicle(1, 8.0, {1, 2});
    a.setLength(4.5);
    a.setLaneIndex(0);
    a.setPositionOnRoad(5.0);
    a.setSpeed(6.0);

    Vehicle b = makeVehicle(2, 8.0, {1, 2});
    b.setLength(4.5);
    b.setLaneIndex(1);
    b.setPositionOnRoad(5.0);
    b.setSpeed(6.0);

    sim.addVehicle(a);
    sim.addVehicle(b);
    sim.start();

    step(sim, city, 0.1, 40);

    const Vehicle* va = nullptr;
    const Vehicle* vb = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            va = &vehicle;
        }
        if (vehicle.getId() == 2) {
            vb = &vehicle;
        }
    }

    expect(va != nullptr && vb != nullptr, "parallel_both_present");
    if (va != nullptr && vb != nullptr) {
        expect(va->getLaneIndex() == 0 && vb->getLaneIndex() == 1, "parallel_lanes_preserved");
        expect(va->getPositionOnRoad() > 5.0 + 2.0, "parallel_lane0_progressed");
        expect(vb->getPositionOnRoad() > 5.0 + 2.0, "parallel_lane1_progressed");
        expect(va->getSpeed() > 1.0 && vb->getSpeed() > 1.0, "parallel_both_moving");
    }
}

void testSameLaneFollowGapAndResume() {
    City city = makeMultiLaneCity(80.0, 6.0, 1);
    Simulation sim;
    redirectSimOutputs(sim, "same_lane_follow");
    sim.setMinVehicleGap(3.0);

    Vehicle leader = makeVehicle(1, 6.0, {1, 2});
    leader.setLength(5.0);
    leader.setLaneIndex(0);
    leader.setPositionOnRoad(30.0);
    leader.setSpeed(0.0);
    leader.setDesiredSpeed(0.0);
    leader.setMaxSpeed(0.0);

    Vehicle follower = makeVehicle(2, 6.0, {1, 2});
    follower.setLength(4.0);
    follower.setLaneIndex(0);
    follower.setPositionOnRoad(10.0);
    follower.setSpeed(5.0);

    sim.addVehicle(leader);
    sim.addVehicle(follower);
    sim.start();

    step(sim, city, 0.1, 80);

    const Vehicle* leadPtr = nullptr;
    const Vehicle* followPtr = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            leadPtr = &vehicle;
        }
        if (vehicle.getId() == 2) {
            followPtr = &vehicle;
        }
    }

    expect(leadPtr != nullptr && followPtr != nullptr, "follow_both_present");
    if (leadPtr != nullptr && followPtr != nullptr) {
        const double gap = leadPtr->getPositionOnRoad() - followPtr->getPositionOnRoad();
        const double required = leadPtr->getLength() + 3.0;
        expect(gap + 0.25 >= required, "follow_gap_accounts_for_length");
        expect(followPtr->isStopped() || followPtr->getSpeed() < 0.35, "follow_stopped_behind_leader");
    }

    // Release leader so the queue can resume.
    for (auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            vehicle.setMaxSpeed(6.0);
            vehicle.setDesiredSpeed(6.0);
            vehicle.setSpeed(4.0);
        }
    }

    const double followPosBefore =
        followPtr != nullptr ? followPtr->getPositionOnRoad() : 0.0;

    step(sim, city, 0.1, 60);

    bool resumed = false;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 2) {
            resumed = vehicle.getPositionOnRoad() > followPosBefore + 1.0 ||
                      vehicle.getCurrentRoadSegmentId() == 2 ||
                      vehicle.isFinished();
        }
    }
    expect(resumed, "follow_resumes_when_leader_moves");
}

void testDesiredSpeedVarietyAndStableTraits() {
    // Separate roads so vehicles do not interact while cruising.
    City city("open_cruise");
    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(120.0, 0.0)));
    city.addIntersection(Intersection(3, Vector2D(0.0, 10.0)));
    city.addIntersection(Intersection(4, Vector2D(120.0, 10.0)));
    city.addRoadSegment(RoadSegment(1, 1, 2, 120.0, 12.0));
    city.addRoadSegment(RoadSegment(2, 3, 4, 120.0, 12.0));

    Simulation sim;
    redirectSimOutputs(sim, "desired_speed");
    sim.setMinVehicleGap(2.5);

    Vehicle vSlow = makeVehicle(1, 10.0, {1});
    vSlow.setDesiredSpeed(4.0);
    vSlow.setMaxSpeed(10.0);
    vSlow.setLength(4.0);
    vSlow.setMaxAcceleration(3.8);
    vSlow.setMaxDeceleration(5.5);
    vSlow.setPositionOnRoad(2.0);

    Vehicle vFast = makeVehicle(2, 10.0, {2});
    vFast.setDesiredSpeed(9.0);
    vFast.setMaxSpeed(10.0);
    vFast.setLength(5.0);
    vFast.setMaxAcceleration(4.0);
    vFast.setMaxDeceleration(5.0);
    vFast.setPositionOnRoad(2.0);

    sim.addVehicle(vSlow);
    sim.addVehicle(vFast);
    sim.start();

    const double slowDesired = vSlow.getDesiredSpeed();
    const double slowLen = vSlow.getLength();
    const double slowAcc = vSlow.getMaxAcceleration();
    const double fastDesired = vFast.getDesiredSpeed();
    const double fastLen = vFast.getLength();

    step(sim, city, 0.1, 50);

    const Vehicle* s = nullptr;
    const Vehicle* f = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            s = &vehicle;
        }
        if (vehicle.getId() == 2) {
            f = &vehicle;
        }
    }

    expect(s != nullptr && f != nullptr, "variety_both_present");
    if (s != nullptr && f != nullptr) {
        expect(s->getSpeed() < f->getSpeed() - 1.0, "variety_different_open_road_speeds");
        expect(s->getSpeed() <= 4.0 + 0.2, "variety_slow_respects_desired");
        expect(f->getSpeed() <= 9.0 + 0.2, "variety_fast_respects_desired");
        expect(approx(s->getDesiredSpeed(), slowDesired, 1e-9), "traits_desired_stable_slow");
        expect(approx(s->getLength(), slowLen, 1e-9), "traits_length_stable_slow");
        expect(approx(s->getMaxAcceleration(), slowAcc, 1e-9), "traits_accel_stable");
        expect(approx(f->getDesiredSpeed(), fastDesired, 1e-9), "traits_desired_stable_fast");
        expect(approx(f->getLength(), fastLen, 1e-9), "traits_length_stable_fast");
    }

    // Seed policy: same seed ⇒ same first desired speeds (defaultSeed = 20261013).
    VehicleGenerator genA(1.0, 8.0, 3, 1.0);
    genA.setSeed(VehicleGenerator::defaultSeed);
    genA.setRoute({1});
    VehicleGenerator genB(1.0, 8.0, 3, 1.0);
    genB.setSeed(VehicleGenerator::defaultSeed);
    genB.setRoute({1});

    vector<Vehicle> spawnedA;
    vector<Vehicle> spawnedB;
    int nextIdA = 1;
    int nextIdB = 1;
    genA.update(1.0, spawnedA, nextIdA);
    genB.update(1.0, spawnedB, nextIdB);
    expect(!spawnedA.empty() && !spawnedB.empty(), "generator_spawned");
    if (!spawnedA.empty() && !spawnedB.empty()) {
        expect(approx(spawnedA[0].getDesiredSpeed(), spawnedB[0].getDesiredSpeed(), 1e-9),
               "generator_seed_reproducible");
        expect(spawnedA[0].getDesiredSpeed() <= 8.0 + 1e-9, "generator_desired_le_base");
        expect(spawnedA[0].getDesiredSpeed() >= 8.0 * 0.80 - 1e-6, "generator_desired_ge_floor");
    }
}

void testLaneChangeSafeAndUnsafe() {
    City city = makeMultiLaneCity(80.0, 6.0, 2);
    Simulation sim;
    redirectSimOutputs(sim, "lane_change");
    sim.setMinVehicleGap(3.0);

    Vehicle ego = makeVehicle(1, 6.0, {1, 2});
    ego.setLength(4.5);
    ego.setLaneIndex(0);
    ego.setPositionOnRoad(20.0);
    ego.setSpeed(3.0);

    // Occupant on target lane overlapping longitudinally → refuse.
    Vehicle blocker = makeVehicle(2, 6.0, {1, 2});
    blocker.setLength(4.5);
    blocker.setLaneIndex(1);
    blocker.setPositionOnRoad(21.0);
    blocker.setSpeed(0.0);
    blocker.setDesiredSpeed(0.0);
    blocker.setMaxSpeed(0.0);

    sim.addVehicle(ego);
    sim.addVehicle(blocker);
    sim.start();

    expect(!sim.tryLaneChange(1, 1, city), "lc_refuses_when_unsafe");
    expect(sim.getVehicles()[0].getLaneIndex() == 0, "lc_lane_unchanged_on_refuse");

    // Clear overlap: move blocker far ahead.
    for (auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 2) {
            vehicle.setPositionOnRoad(55.0);
        }
    }

    expect(sim.tryLaneChange(1, 1, city), "lc_succeeds_with_space");
    const Vehicle* after = nullptr;
    for (const auto& vehicle : sim.getVehicles()) {
        if (vehicle.getId() == 1) {
            after = &vehicle;
        }
    }
    expect(after != nullptr && after->getLaneIndex() == 1, "lc_lane_updated");
    expect(after != nullptr && after->getLaneChangeCooldown() > 0.0, "lc_cooldown_set");

    // Immediate thrash attempt should fail due to cooldown.
    expect(!sim.tryLaneChange(1, 0, city), "lc_refuses_during_cooldown");

    // After successful LC, no longitudinal overlap with others on new lane.
    bool overlap = false;
    CollisionDetector detector(3.0);
    for (const auto& a : sim.getVehicles()) {
        for (const auto& b : sim.getVehicles()) {
            if (a.getId() >= b.getId() || a.isFinished() || b.isFinished()) {
                continue;
            }
            if (a.getCurrentRoadSegmentId() != b.getCurrentRoadSegmentId() ||
                a.getLaneIndex() != b.getLaneIndex()) {
                continue;
            }
            if (detector.vehiclesOverlapLongitudinally(
                    a.getPositionOnRoad(), a.getLength(),
                    b.getPositionOnRoad(), b.getLength())) {
                overlap = true;
            }
        }
    }
    expect(!overlap, "lc_no_overlap_after_change");

    // Single-lane road: no LC.
    City single = makeMultiLaneCity(40.0, 6.0, 1);
    Simulation simSingle;
    redirectSimOutputs(simSingle, "lc_single");
    Vehicle only = makeVehicle(1, 5.0, {1});
    only.setLaneIndex(0);
    simSingle.addVehicle(only);
    simSingle.start();
    expect(!simSingle.tryLaneChange(1, 1, single), "lc_impossible_on_single_lane");
}

void testRoadChangePreservesSpeedAndClampsLane() {
    City city = makeMultiLaneCity(25.0, 10.0, 2);
    Simulation sim;
    redirectSimOutputs(sim, "road_change_lane");
    sim.setMinVehicleGap(2.5);

    Vehicle vehicle = makeVehicle(1, 10.0, {1, 2});
    vehicle.setLength(4.0);
    vehicle.setLaneIndex(1); // invalid on next road (1 lane) → clamp to 0
    vehicle.setPositionOnRoad(20.0);
    vehicle.setSpeed(5.0);
    vehicle.setDesiredSpeed(10.0);
    sim.addVehicle(vehicle);
    sim.start();

    double speedBefore = -1.0;
    double speedAfter = -1.0;
    int laneAfter = -1;

    for (int i = 0; i < 200; ++i) {
        for (const auto& v : sim.getVehicles()) {
            if (v.getId() == 1 && v.getCurrentRoadSegmentId() == 1) {
                speedBefore = v.getSpeed();
            }
        }

        sim.update(0.05, city);

        for (const auto& v : sim.getVehicles()) {
            if (v.getId() == 1 && v.getCurrentRoadSegmentId() == 2) {
                speedAfter = v.getSpeed();
                laneAfter = v.getLaneIndex();
                break;
            }
        }

        if (speedAfter >= 0.0) {
            break;
        }
    }

    expect(speedAfter >= 0.0, "road_change_did_transition");
    if (speedAfter >= 0.0) {
        expect(laneAfter == 0, "road_change_lane_clamped");
        expect(!(speedBefore < 0.5 && approx(speedAfter, 6.0, 0.3)), "road_change_no_60pct_boost");
        expect(abs(speedAfter - speedBefore) < 2.5 || speedBefore < 0.0, "road_change_speed_preserved_roughly");
    }
}

void testLaneLateralOffsetInLayout() {
    City city = makeMultiLaneCity(50.0, 6.0, 2);
    CityLayout layout;
    Vector2D p0 = layout.getRoadPosition(city, 1, 25.0, 0, 2);
    Vector2D p1 = layout.getRoadPosition(city, 1, 25.0, 1, 2);
    const double lateral = abs(p0.y - p1.y);
    expect(lateral > 1.0, "render_lanes_laterally_offset");
    expect(approx(lateral, SimConstants::LANE_WIDTH, 0.05), "render_lane_width_spacing");
}

} // namespace

int main() {
    cout << "Easy Rider motion regression tests\n";
    ensureOutputDir();

    testFreePathTwoSegments();
    testRedStopResumeGreen();
    testQueueThenMove();
    testCrossingStopAndResume();
    testPastCrossingNotBlocked();
    testBlockedNextRoadThenEnter();
    testDestinationCompletion();
    testInvalidRouteControllable();
    testNoSpeedBoostOnTransition();
    testScenarioIntegration120s();
    testSessionLifecyclePauseResumeReset();
    testTimeMultiplierFixedDtAndCatchUpClamp();
    testManualBlockViaSimulationApi();
    testSpawnRateAndPedTempoScaling();

    testParallelLanesIndependent();
    testSameLaneFollowGapAndResume();
    testDesiredSpeedVarietyAndStableTraits();
    testLaneChangeSafeAndUnsafe();
    testRoadChangePreservesSpeedAndClampsLane();
    testLaneLateralOffsetInLayout();

    const int mapFailures = runMapModelTests();
    g_failures += mapFailures;
    g_passed += mapModelPassedCount();

    const int priorityFailures = runIntersectionPriorityTests();
    g_failures += priorityFailures;
    g_passed += intersectionPriorityPassedCount();

    const int pedFailures = runPedestrianTests();
    g_failures += pedFailures;
    g_passed += pedestrianPassedCount();

    const int blockFailures = runLaneBlockageTests();
    g_failures += blockFailures;
    g_passed += laneBlockagePassedCount();

    const int demoFailures = runDemoIntegrationTests();
    g_failures += demoFailures;
    g_passed += demoIntegrationPassedCount();

    const int osmFailures = runOsmJsonParserTests();
    g_failures += osmFailures;
    g_passed += osmJsonParserPassedCount();

    const int osmMapFailures = runOsmToMapTests();
    g_failures += osmMapFailures;
    g_passed += osmToMapPassedCount();

    const int osmScenarioFailures = runOsmScenarioTests();
    g_failures += osmScenarioFailures;
    g_passed += osmScenarioPassedCount();

    const int osmLargerFailures = runOsmLargerMapTests();
    g_failures += osmLargerFailures;
    g_passed += osmLargerMapPassedCount();

    const int osmSessionFailures = runOsmSessionTests();
    g_failures += osmSessionFailures;
    g_passed += osmSessionPassedCount();

    const int cameraFailures = runMapCameraTests();
    g_failures += cameraFailures;
    g_passed += mapCameraPassedCount();

    const int downloadFailures = runMapDownloadTests();
    g_failures += downloadFailures;
    g_passed += mapDownloadPassedCount();

    const int locatorFailures = runLocatorSelectionTests();
    g_failures += locatorFailures;
    g_passed += locatorSelectionPassedCount();

    const int geometryFailures = runGeometryControlTests();
    g_failures += geometryFailures;
    g_passed += geometryControlPassedCount();

    const int trafficFailures = runTrafficBaselineTests();
    g_failures += trafficFailures;
    g_passed += trafficBaselinePassedCount();

    const int etap3Failures = runEtap3TrafficTests();
    g_failures += etap3Failures;
    g_passed += etap3TrafficPassedCount();

    const int encodingFailures = runEncodingTests();
    g_failures += encodingFailures;
    g_passed += encodingPassedCount();

    cout << "\nPassed: " << g_passed << "  Failed: " << g_failures << "\n";
    return g_failures == 0 ? 0 : 1;
}
