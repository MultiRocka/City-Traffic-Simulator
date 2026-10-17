#pragma once

#include "core/City.h"
#include "core/Simulation.h"
#include "ui/CityLayout.h"
#include "ui/RenderSnapshot.h"

using namespace std;

class VehicleRenderer {
public:
    void buildVehicles(const City& city, const Simulation& simulation, RenderSnapshot& snapshot) const;

private:
    CityLayout layout;
};
