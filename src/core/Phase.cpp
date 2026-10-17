#include "core/Phase.h"
#include <algorithm>

using namespace std;

Phase::Phase() : id(-1), duration(0.0) {
}

Phase::Phase(int id, double duration, const vector<int>& greenRoadSegmentIds)
    : id(id), duration(duration), greenRoadSegmentIds(greenRoadSegmentIds) {
}

int Phase::getId() const {
    return id;
}

double Phase::getDuration() const {
    return duration;
}

const vector<int>& Phase::getGreenRoadSegmentIds() const {
    return greenRoadSegmentIds;
}

bool Phase::isRoadGreen(int roadSegmentId) const {
    return find(greenRoadSegmentIds.begin(), greenRoadSegmentIds.end(), roadSegmentId) != greenRoadSegmentIds.end();
}
