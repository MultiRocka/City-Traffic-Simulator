#pragma once

#include <string>
#include <vector>
#include "core/Vector2D.h"

using namespace std;

// Junction priority mode (enforced by IntersectionRules in Stage 3).
// Unsignalized None is treated as YieldToRight at runtime.
enum class PriorityMode {
    None,
    Signalized,
    PriorityRoad,
    YieldToRight
};

class Intersection {
public:
    Intersection();
    Intersection(int id, const Vector2D& position);

    int getId() const;
    Vector2D getPosition() const;

    void addConnectedRoadId(int roadId);
    const vector<int>& getConnectedRoadIds() const;

    const string& getSourceRef() const;
    void setSourceRef(const string& ref);

    PriorityMode getPriorityMode() const;
    void setPriorityMode(PriorityMode mode);

    const vector<int>& getPriorityRoadIds() const;
    void setPriorityRoadIds(const vector<int>& roadIds);

private:
    int id;
    Vector2D position;
    vector<int> connectedRoadIds;
    string sourceRef;
    PriorityMode priorityMode;
    vector<int> priorityRoadIds;
};
