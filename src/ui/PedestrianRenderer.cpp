#include "ui/PedestrianRenderer.h"
#include "core/CollisionDetector.h"
#include "core/Pedestrian.h"
#include <algorithm>
#include <cmath>

using namespace std;

void PedestrianRenderer::buildCrossings(const City& city, const Simulation& simulation, RenderSnapshot& snapshot) const {
    for (const auto& crossing : simulation.getPedestrianCrossings()) {
        CrossingRenderInfo crossingInfo;
        crossingInfo.crossingId = crossing.getId();
        crossingInfo.roadSegmentId = crossing.getRoadSegmentId();
        crossingInfo.active = crossing.isActive();
        crossingInfo.worldPosition = layout.getRoadPosition(
            city,
            crossing.getRoadSegmentId(),
            crossing.getPositionOnRoad()
        );

        snapshot.crossings.push_back(crossingInfo);
    }
}

void PedestrianRenderer::buildPedestrians(const City& city, const Simulation& simulation, RenderSnapshot& snapshot) const {
    for (const auto& ped : simulation.getPedestrianManager().getPedestrians()) {
        if (ped.isFinished()) {
            continue;
        }

        const RoadSegment* road = city.findRoadSegmentById(ped.getRoadSegmentId());
        if (road == nullptr) {
            continue;
        }

        const PedestrianCrossing* crossing = nullptr;
        for (const auto& candidate : simulation.getPedestrianCrossings()) {
            if (candidate.getId() == ped.getCrossingId()) {
                crossing = &candidate;
                break;
            }
        }
        if (crossing == nullptr) {
            continue;
        }

        const int laneCount = max(1, road->getLaneCount());
        const Vector2D center = layout.getRoadPosition(
            city,
            ped.getRoadSegmentId(),
            crossing->getPositionOnRoad(),
            0,
            1
        );

        const Intersection* fromIntersection = city.findIntersectionById(road->getFromIntersectionId());
        const Intersection* toIntersection = city.findIntersectionById(road->getToIntersectionId());
        Vector2D world = center;

        if (fromIntersection != nullptr && toIntersection != nullptr) {
            const double dx = toIntersection->getPosition().x - fromIntersection->getPosition().x;
            const double dy = toIntersection->getPosition().y - fromIntersection->getPosition().y;
            const double len = sqrt(dx * dx + dy * dy);
            if (len > 1e-9) {
                // Unit perpendicular (left of travel): crosswalk axis.
                const double px = -dy / len;
                const double py = dx / len;
                const double halfWidth = 0.5 * ped.getCrossWidth();
                double lateral = -halfWidth - 1.0; // waiting: just off near curb
                if (ped.isOnRoadway()) {
                    const double t = ped.getCrossWidth() > 1e-6
                        ? ped.getCrossProgress() / ped.getCrossWidth()
                        : 0.0;
                    lateral = -halfWidth + t * ped.getCrossWidth();
                }
                world = Vector2D(center.x + px * lateral, center.y + py * lateral);
            }
        }

        (void)laneCount;

        PedestrianRenderInfo info;
        info.pedestrianId = ped.getId();
        info.crossingId = ped.getCrossingId();
        info.roadSegmentId = ped.getRoadSegmentId();
        info.worldPosition = world;
        info.state = ped.isOnRoadway() ? 1 : 0;
        info.crossProgress = ped.getCrossProgress();
        snapshot.pedestrians.push_back(info);
    }
}
