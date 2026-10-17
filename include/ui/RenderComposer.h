#pragma once

#include "core/City.h"
#include "core/Simulation.h"
#include "ui/RenderSnapshot.h"
#include "ui/CityRenderer.h"
#include "ui/VehicleRenderer.h"
#include "ui/TrafficLightRenderer.h"
#include "ui/PedestrianRenderer.h"

using namespace std;

class RenderComposer {
public:
    RenderSnapshot buildSnapshot(const City& city, const Simulation& simulation) const;
    RenderSnapshot interpolate(const RenderSnapshot& previous, const RenderSnapshot& current, double alpha) const;

private:
    CityRenderer cityRenderer;
    VehicleRenderer vehicleRenderer;
    TrafficLightRenderer trafficLightRenderer;
    PedestrianRenderer pedestrianRenderer;

    static Vector2D lerpPosition(const Vector2D& from, const Vector2D& to, double alpha);
};
