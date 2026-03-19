#pragma once

#include <cmath>

using namespace std;

class Vector2D {
public:
    double x;
    double y;

    Vector2D() : x(0.0), y(0.0) {
    }

    Vector2D(double xValue, double yValue) : x(xValue), y(yValue) {
    }

    double distanceTo(const Vector2D& other) const {
        double dx = other.x - x;
        double dy = other.y - y;
        return sqrt(dx * dx + dy * dy);
    }
};
