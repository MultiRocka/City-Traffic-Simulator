#pragma once

using namespace std;

// How pedestrians and vehicles interact at this crossing (Stage 4).
enum class CrossingMode {
    Timed = 0,        // Legacy timer occupancy + visible ped sync (default)
    Signalized = 1,   // Ped walks when vehicle signal is red; phase held until clear
    Unsignalized = 2  // Ped requests; cars yield (subject to configured refusal)
};

class PedestrianCrossing {
public:
    PedestrianCrossing();
    PedestrianCrossing(
        int id,
        int roadSegmentId,
        double positionOnRoad,
        double inactiveDuration,
        double activeDuration,
        bool startActive
    );

    void update(double dt);

    int getId() const;
    int getRoadSegmentId() const;
    double getPositionOnRoad() const;
    bool isActive() const;
    double getTimer() const;
    double getInactiveDuration() const;
    double getActiveDuration() const;
    CrossingMode getMode() const;
    double getWalkSpeed() const;

    void setMode(CrossingMode newMode);
    void setWalkSpeed(double metersPerSecond);
    // Entity-driven occupancy for Signalized / Unsignalized (ped on roadway).
    void setEntityOccupying(bool occupying);

private:
    int id;
    int roadSegmentId;
    double positionOnRoad;
    double inactiveDuration;
    double activeDuration;
    double timer;
    bool active;
    CrossingMode mode;
    double walkSpeed;
    bool entityOccupying;
};
