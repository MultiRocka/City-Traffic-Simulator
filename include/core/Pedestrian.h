#pragma once

enum class PedestrianState {
    Waiting,   // off-roadway, requesting to cross
    Crossing,  // on roadway
    Finished
};

// Visible pedestrian entity bound to a PedestrianCrossing (Stage 4).
// Cross progress is meters from the near curb toward the far curb.
class Pedestrian {
public:
    Pedestrian();
    Pedestrian(int id, int crossingId, int roadSegmentId, double crossSpeed);

    int getId() const;
    int getCrossingId() const;
    int getRoadSegmentId() const;
    PedestrianState getState() const;
    double getCrossProgress() const;
    double getCrossSpeed() const;
    double getCrossWidth() const;

    void setState(PedestrianState newState);
    void setCrossWidth(double widthMeters);
    void setCrossProgress(double meters);

    // Advance while Crossing; clamps to width and marks Finished when done.
    void update(double dt);

    bool isOnRoadway() const;
    bool isWaiting() const;
    bool isFinished() const;

private:
    int id;
    int crossingId;
    int roadSegmentId;
    PedestrianState state;
    double crossProgress;
    double crossSpeed;
    double crossWidth;
};
