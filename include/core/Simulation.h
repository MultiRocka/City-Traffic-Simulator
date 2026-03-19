#pragma once

#include <vector>
#include "core/Vehicle.h"
#include "core/City.h"
#include "core/VehicleGenerator.h"
#include "core/TrafficLight.h"

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

    void addVehicleGenerator(const VehicleGenerator& generator);
    void setMinVehicleGap(double newMinVehicleGap);

    void addTrafficLight(const TrafficLight& trafficLight);
    vector<TrafficLight>& getTrafficLights();
    const vector<TrafficLight>& getTrafficLights() const;

private:
    bool running;
    double currentTime;
    vector<Vehicle> vehicles;
    vector<VehicleGenerator> generators;
    double minVehicleGap;
    vector<TrafficLight> trafficLights;
};
