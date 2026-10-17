#pragma once

#include <vector>

using namespace std;

class Vehicle {
public:
    Vehicle();
    // Sets both maxSpeed and desiredSpeed to the same value (Stage 0/1 compatible).
    Vehicle(int id, double maxSpeed);

    int getId() const;
    double getPositionOnRoad() const;
    double getSpeed() const;
    double getMaxSpeed() const;
    double getDesiredSpeed() const;
    double getLength() const;
    double getMaxAcceleration() const;
    double getMaxDeceleration() const;
    int getLaneIndex() const;
    int getPreferLane() const;
    double getLaneChangeCooldown() const;
    bool isStopped() const;
    bool isFinished() const;
    int getCurrentRoadSegmentId() const;
    int getNextRoadSegmentId() const;
    int getCurrentRoadIndex() const;

    void setSpeed(double newSpeed);
    void setPositionOnRoad(double newPosition);
    void setDesiredSpeed(double newDesiredSpeed);
    void setMaxSpeed(double newMaxSpeed);
    void setLength(double newLength);
    void setMaxAcceleration(double newMaxAcceleration);
    void setMaxDeceleration(double newMaxDeceleration);
    void setLaneIndex(int newLaneIndex);
    void setPreferLane(int newPreferLane);
    void setLaneChangeCooldown(double seconds);
    void setRoute(const vector<int>& newRoute);

    // Clamp laneIndex into [0, laneCount-1]. No-op if laneCount < 1.
    void clampLaneIndex(int laneCount);

    // Apply an intentional lane change after caller verified safety (Simulation / tests).
    // Resets cooldown to prevent thrashing. Does not validate gaps.
    void applyLaneChange(int targetLane, double cooldownSeconds = 1.5);

    void update(double dt, double roadLength, double maxAllowedPosition, double speedLimit, double minVehicleGap);
    bool moveToNextRoad();
    void finish();

private:
    double computeTargetSpeed(double maxAllowedPosition, double speedLimit) const;

    int id;
    vector<int> route;
    int currentRoadIndex;
    double positionOnRoad;
    double speed;
    double maxSpeed;
    double desiredSpeed;
    double length;
    double maxAcceleration;
    double maxDeceleration;
    int laneIndex;
    int preferLane; // -1 = no preference (default stay in lane)
    double laneChangeCooldown;
    bool stopped;
    bool finished;

    static constexpr double kDefaultLength = 4.5;
    static constexpr double kDefaultMaxAcceleration = 3.8;
    static constexpr double kDefaultMaxDeceleration = 5.5;
    static constexpr double reactionDistance = 7.0;
};
