#include "ui/CityLayout.h"
#include "core/CollisionDetector.h"
#include <algorithm>
#include <cmath>

using namespace std;

static Vector2D interpolatePosition(const Vector2D& start, const Vector2D& end, double ratio) {
    if (ratio < 0.0) {
        ratio = 0.0;
    }

    if (ratio > 1.0) {
        ratio = 1.0;
    }

    return Vector2D(
        start.x + (end.x - start.x) * ratio,
        start.y + (end.y - start.y) * ratio
    );
}

static vector<Vector2D> buildCenterline(const City& city, const RoadSegment& road) {
    vector<Vector2D> points;
    const Intersection* fromIntersection = city.findIntersectionById(road.getFromIntersectionId());
    const Intersection* toIntersection = city.findIntersectionById(road.getToIntersectionId());
    if (fromIntersection == nullptr || toIntersection == nullptr) {
        return points;
    }

    points.push_back(fromIntersection->getPosition());
    for (const auto& point : road.getIntermediatePoints()) {
        points.push_back(point);
    }
    points.push_back(toIntersection->getPosition());
    return points;
}

static Vector2D positionAlongPolyline(const vector<Vector2D>& points, double fraction) {
    if (points.empty()) {
        return Vector2D();
    }
    if (points.size() == 1) {
        return points.front();
    }

    fraction = max(0.0, min(1.0, fraction));

    double total = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        total += points[i].distanceTo(points[i + 1]);
    }
    if (total <= 1e-9) {
        return points.front();
    }

    double target = fraction * total;
    double walked = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const double seg = points[i].distanceTo(points[i + 1]);
        if (walked + seg >= target || i + 2 == points.size()) {
            const double local = seg > 1e-9 ? (target - walked) / seg : 0.0;
            return interpolatePosition(points[i], points[i + 1], local);
        }
        walked += seg;
    }

    return points.back();
}

static void tangentAtFraction(const vector<Vector2D>& points, double fraction, double& dx, double& dy) {
    dx = 1.0;
    dy = 0.0;
    if (points.size() < 2) {
        return;
    }

    fraction = max(0.0, min(1.0, fraction));
    double total = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        total += points[i].distanceTo(points[i + 1]);
    }
    if (total <= 1e-9) {
        return;
    }

    double target = fraction * total;
    double walked = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const double seg = points[i].distanceTo(points[i + 1]);
        if (walked + seg >= target || i + 2 == points.size()) {
            dx = points[i + 1].x - points[i].x;
            dy = points[i + 1].y - points[i].y;
            return;
        }
        walked += seg;
    }
}

Vector2D CityLayout::getRoadPosition(const City& city, int roadId, double distanceAlongRoad) const {
    return getRoadPosition(city, roadId, distanceAlongRoad, 0, 1);
}

Vector2D CityLayout::getRoadPosition(
    const City& city,
    int roadId,
    double distanceAlongRoad,
    int laneIndex,
    int laneCount
) const {
    const RoadSegment* road = city.findRoadSegmentById(roadId);

    if (road == nullptr || road->getLength() <= 0.0) {
        return Vector2D();
    }

    const vector<Vector2D> centerline = buildCenterline(city, *road);
    if (centerline.size() < 2) {
        return Vector2D();
    }

    // Motion uses authoritative length; map onto polyline by fraction of road.length.
    const double fraction = distanceAlongRoad / road->getLength();
    Vector2D center = positionAlongPolyline(centerline, fraction);

    const int lanes = max(1, laneCount);
    if (lanes <= 1) {
        return center;
    }

    double dx = 0.0;
    double dy = 0.0;
    tangentAtFraction(centerline, fraction, dx, dy);
    const double len = sqrt(dx * dx + dy * dy);
    if (len <= 1e-9) {
        return center;
    }

    // Unit perpendicular (left of travel direction).
    const double px = -dy / len;
    const double py = dx / len;
    const double centered = static_cast<double>(laneIndex) - 0.5 * static_cast<double>(lanes - 1);
    const double offset = centered * SimConstants::LANE_WIDTH;

    return Vector2D(center.x + px * offset, center.y + py * offset);
}

Vector2D CityLayout::getIntersectionPosition(const City& city, int intersectionId) const {
    const Intersection* intersection = city.findIntersectionById(intersectionId);

    if (intersection == nullptr) {
        return Vector2D();
    }

    return intersection->getPosition();
}

WorldBounds CityLayout::getWorldBounds(const City& city) const {
    WorldBounds bounds {0.0, 0.0, 0.0, 0.0};
    bool hasPoints = false;

    auto consider = [&](const Vector2D& position) {
        if (!hasPoints) {
            bounds.minX = position.x;
            bounds.maxX = position.x;
            bounds.minY = position.y;
            bounds.maxY = position.y;
            hasPoints = true;
            return;
        }
        bounds.minX = min(bounds.minX, position.x);
        bounds.maxX = max(bounds.maxX, position.x);
        bounds.minY = min(bounds.minY, position.y);
        bounds.maxY = max(bounds.maxY, position.y);
    };

    for (const auto& intersection : city.getIntersections()) {
        consider(intersection.getPosition());
    }
    for (const auto& road : city.getRoadSegments()) {
        for (const auto& point : road.getIntermediatePoints()) {
            consider(point);
        }
    }

    return bounds;
}

WorldBounds CityLayout::getWorldBoundsFromSnapshot(const RenderSnapshot& snapshot) const {
    WorldBounds bounds {0.0, 0.0, 0.0, 0.0};
    bool hasPoints = false;

    auto considerPoint = [&](const Vector2D& position) {
        if (!hasPoints) {
            bounds.minX = position.x;
            bounds.maxX = position.x;
            bounds.minY = position.y;
            bounds.maxY = position.y;
            hasPoints = true;
            return;
        }

        bounds.minX = min(bounds.minX, position.x);
        bounds.maxX = max(bounds.maxX, position.x);
        bounds.minY = min(bounds.minY, position.y);
        bounds.maxY = max(bounds.maxY, position.y);
    };

    for (const auto& intersection : snapshot.intersections) {
        considerPoint(intersection.position);
    }

    for (const auto& road : snapshot.roads) {
        if (!road.centerline.empty()) {
            for (const auto& point : road.centerline) {
                considerPoint(point);
            }
        } else {
            considerPoint(road.start);
            considerPoint(road.end);
        }
    }

    if (!hasPoints) {
        bounds.minX = -1.0;
        bounds.maxX = 1.0;
        bounds.minY = -1.0;
        bounds.maxY = 1.0;
    } else {
        double padding = 1.0;
        bounds.minX -= padding;
        bounds.maxX += padding;
        bounds.minY -= padding;
        bounds.maxY += padding;
    }

    return bounds;
}
