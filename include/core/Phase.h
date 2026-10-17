#pragma once

#include <vector>

using namespace std;

class Phase {
public:
    Phase();
    Phase(int id, double duration, const vector<int>& greenRoadSegmentIds);

    int getId() const;
    double getDuration() const;
    const vector<int>& getGreenRoadSegmentIds() const;
    bool isRoadGreen(int roadSegmentId) const;

private:
    int id;
    double duration;
    vector<int> greenRoadSegmentIds;
};
