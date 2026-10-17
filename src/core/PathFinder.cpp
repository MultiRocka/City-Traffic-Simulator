#include "core/PathFinder.h"
#include <queue>
#include <unordered_map>
#include <algorithm>

using namespace std;

vector<int> PathFinder::findShortestRoute(const City& city, int startRoadId, int endRoadId) const {
    if (startRoadId == endRoadId) {
        return {startRoadId};
    }

    if (!city.findRoadSegmentById(startRoadId) || !city.findRoadSegmentById(endRoadId)) {
        return {};
    }

    queue<int> roadQueue;
    unordered_map<int, int> previousRoad;
    unordered_map<int, bool> visited;

    roadQueue.push(startRoadId);
    visited[startRoadId] = true;

    while (!roadQueue.empty()) {
        int currentRoadId = roadQueue.front();
        roadQueue.pop();

        if (currentRoadId == endRoadId) {
            vector<int> route;
            int roadId = endRoadId;

            while (true) {
                route.push_back(roadId);

                if (roadId == startRoadId) {
                    break;
                }

                roadId = previousRoad[roadId];
            }

            reverse(route.begin(), route.end());
            return route;
        }

        for (int nextRoadId : city.getOutgoingRoadIds(currentRoadId)) {
            if (!visited[nextRoadId]) {
                visited[nextRoadId] = true;
                previousRoad[nextRoadId] = currentRoadId;
                roadQueue.push(nextRoadId);
            }
        }
    }

    return {};
}

vector<int> PathFinder::findRouteThroughIntersections(const City& city, int startRoadId, int endIntersectionId) const {
    if (!city.findRoadSegmentById(startRoadId)) {
        return {};
    }

    vector<int> bestRoute;

    for (const auto& roadSegment : city.getRoadSegments()) {
        if (roadSegment.getToIntersectionId() != endIntersectionId) {
            continue;
        }

        vector<int> route = findShortestRoute(city, startRoadId, roadSegment.getId());

        if (route.empty()) {
            continue;
        }

        if (bestRoute.empty() || route.size() < bestRoute.size()) {
            bestRoute = route;
        }
    }

    return bestRoute;
}
