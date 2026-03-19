#pragma once

#include <vector>
#include "core/Vehicle.h"
#include "core/City.h"

using namespace std;

class Simulation {
public:
    Simulation();

    void start();
    void stop();
    void update(double dt, const City& city);

    bool isRunning() const;
    double getTime() const;

    void addVehicle(const Vehicle& vehicle);
    vector<Vehicle>& getVehicles();
    const vector<Vehicle>& getVehicles() const;

private:
    bool running;
    double currentTime;
    vector<Vehicle> vehicles;
};
