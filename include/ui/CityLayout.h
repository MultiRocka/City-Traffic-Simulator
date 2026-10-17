#pragma once

#include "core/City.h"
#include "core/Vector2D.h"
#include "ui/RenderSnapshot.h"

using namespace std;

struct WorldBounds {
    double minX;
    double minY;
    double maxX;
    double maxY;
};

class CityLayout {
public:
    Vector2D getRoadPosition(const City& city, int roadId, double distanceAlongRoad) const;
    // Lateral offset perpendicular to travel so parallel lanes are visible in world space.
    Vector2D getRoadPosition(
        const City& city,
        int roadId,
        double distanceAlongRoad,
        int laneIndex,
        int laneCount
    ) const;
    Vector2D getIntersectionPosition(const City& city, int intersectionId) const;
    WorldBounds getWorldBounds(const City& city) const;
    WorldBounds getWorldBoundsFromSnapshot(const RenderSnapshot& snapshot) const;
};
