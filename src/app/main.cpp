#include <thread>
#include <chrono>
#include <iostream>
#include "core/Simulation.h"
#include "core/City.h"
#include "core/VehicleGenerator.h"
#include "core/TrafficLight.h"
#include "core/PedestrianCrossing.h"
#include "core/ConsolePreview.h"

using namespace std;

int main() {
    Simulation sim;
    City city("Intersection Test");

    sim.setStatisticsOutputFiles("simulation_timeline.csv", "simulation_summary.csv");
    sim.setEventLogOutputFile("event_log.json");

    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(0.0, 10.0)));
    city.addIntersection(Intersection(3, Vector2D(10.0, 0.0)));
    city.addIntersection(Intersection(4, Vector2D(20.0, 0.0)));

    city.addRoadSegment(RoadSegment(1, 1, 3, 18.0, 5.0));
    city.addRoadSegment(RoadSegment(2, 2, 3, 10.0, 4.0));
    city.addRoadSegment(RoadSegment(3, 3, 4, 20.0, 6.0));

    Intersection* intersection1 = city.findIntersectionById(1);
    Intersection* intersection2 = city.findIntersectionById(2);
    Intersection* intersection3 = city.findIntersectionById(3);
    Intersection* intersection4 = city.findIntersectionById(4);

    if (intersection1 != nullptr) {
        intersection1->addConnectedRoadId(1);
    }

    if (intersection2 != nullptr) {
        intersection2->addConnectedRoadId(2);
    }

    if (intersection3 != nullptr) {
        intersection3->addConnectedRoadId(1);
        intersection3->addConnectedRoadId(2);
        intersection3->addConnectedRoadId(3);
    }

    if (intersection4 != nullptr) {
        intersection4->addConnectedRoadId(3);
    }

    VehicleGenerator generatorWest(2.0, 4.0, 8, 4.0);
    generatorWest.setRoute({1, 3});

    VehicleGenerator generatorNorth(3.0, 3.0, 6, 4.0);
    generatorNorth.setRoute({2, 3});

    sim.addVehicleGenerator(generatorWest);
    sim.addVehicleGenerator(generatorNorth);
    sim.setMinVehicleGap(3.0);

    TrafficLight westLight(1, 1, 4.0, 4.0, true);
    TrafficLight northLight(2, 2, 4.0, 4.0, false);

    sim.addTrafficLight(westLight);
    sim.addTrafficLight(northLight);

    PedestrianCrossing crossing1(1, 3, 8.0, 5.0, 3.0, false);
    sim.addPedestrianCrossing(crossing1);

    sim.start();

    for (int i = 0; i < 40; i++) {
        sim.update(1.0, city);
        renderConsolePreview(city, sim);
        this_thread::sleep_for(chrono::milliseconds(500));
    }

    sim.stop();
    sim.exportStatisticsSummary();
    sim.exportEventLog();

    cout << endl;
    cout << "Exported: simulation_timeline.csv" << endl;
    cout << "Exported: simulation_summary.csv" << endl;
    cout << "Exported: event_log.json" << endl;

    return 0;
}
