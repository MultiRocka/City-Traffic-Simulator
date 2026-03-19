#include <iostream>
#include "core/Simulation.h"
#include "core/City.h"
#include "core/Vehicle.h"

using namespace std;

int main() {
    Simulation sim;
    City city("Test City");

    city.addIntersection(Intersection(1, Vector2D(0.0, 0.0)));
    city.addIntersection(Intersection(2, Vector2D(100.0, 0.0)));

    city.addRoadSegment(RoadSegment(1, 1, 2, 100.0, 20.0));

    Intersection* intersection1 = city.findIntersectionById(1);
    Intersection* intersection2 = city.findIntersectionById(2);

    if (intersection1 != nullptr) {
        intersection1->addConnectedRoadId(1);
    }

    if (intersection2 != nullptr) {
        intersection2->addConnectedRoadId(1);
    }

    Vehicle vehicle1(1, 1, 0.0, 15.0);
    sim.addVehicle(vehicle1);

    sim.start();

    for (int i = 0; i < 10; i++) {
        sim.update(1.0, city);

        cout << "Time: " << sim.getTime() << " s" << endl;
        cout << "Vehicle position: " << sim.getVehicles()[0].getPositionOnRoad() << " m" << endl;
        cout << "Vehicle speed: " << sim.getVehicles()[0].getSpeed() << " m/s" << endl;
        cout << "Vehicle stopped: " << (sim.getVehicles()[0].isStopped() ? "yes" : "no") << endl;
        cout << "------------------------" << endl;
    }

    sim.stop();

    return 0;
}
