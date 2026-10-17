#include "ui/LaneBlockageRenderer.h"
#include "core/CollisionDetector.h"
#include <algorithm>

using namespace std;

void LaneBlockageRenderer::buildLaneBlockages(
    const City& city,
    const Simulation& simulation,
    RenderSnapshot& snapshot
) const {
    const double now = simulation.getTime();

    for (const auto& block : simulation.getLaneBlockages()) {
        const RoadSegment* road = city.findRoadSegmentById(block.roadId);
        if (road == nullptr) {
            continue;
        }

        const int laneCount = max(1, road->getLaneCount());
        LaneBlockageRenderInfo info;
        info.blockageId = block.id;
        info.roadSegmentId = block.roadId;
        info.laneIndex = block.laneIndex;
        info.positionAlongRoad = block.positionAlongRoad;
        info.length = block.length;
        info.startTime = block.startTime;
        info.duration = block.duration;
        info.active = block.isActive(now);
        info.hasMergeZone = false;
        info.mergeStartWorld = info.startWorld;
        info.startWorld = layout.getRoadPosition(
            city,
            block.roadId,
            block.positionAlongRoad,
            block.laneIndex,
            laneCount
        );
        info.endWorld = layout.getRoadPosition(
            city,
            block.roadId,
            block.endPosition(),
            block.laneIndex,
            laneCount
        );

        if (laneCount >= 2) {
            const double mergeStart = max(0.0, block.positionAlongRoad - SimConstants::ZIPPER_MERGE_ZONE);
            if (mergeStart + 0.5 < block.positionAlongRoad) {
                info.hasMergeZone = true;
                info.mergeStartWorld = layout.getRoadPosition(
                    city,
                    block.roadId,
                    mergeStart,
                    block.laneIndex,
                    laneCount
                );
            }
        }

        snapshot.laneBlockages.push_back(info);
    }
}
