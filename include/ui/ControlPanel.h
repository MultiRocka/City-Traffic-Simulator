#pragma once

#include <cstdint>
#include <string>
#include "core/LaneBlockage.h"
#include "ui/SimulationSession.h"

class ControlPanel {
public:
    explicit ControlPanel(SimulationSession& session);

    void start();
    void pause();
    void resume();
    void stop();
    void reset();
    void stepOnce();
    bool isRunning() const;
    bool isPaused() const;
    double getTickDuration() const;
    void setTickDuration(double tickDuration);

    double getTimeMultiplier() const;
    bool setTimeMultiplier(double multiplier);

    double getVehicleSpawnRate() const;
    bool setVehicleSpawnRate(double rate);

    double getPedestrianRequestTempo() const;
    bool setPedestrianRequestTempo(double tempo);

    // Manual lane blockage. position/length use defaults when not provided.
    // Returns blockage id, or -1 if rejected / invalid input.
    int addManualLaneBlockage(
        int roadId,
        int laneIndex,
        double durationSeconds,
        double positionAlongRoad = -1.0,
        double lengthMeters = 5.0
    );
    bool clearLaneBlockage(int blockageId);
    void clearAllLaneBlockages();

    bool setRandomBlockages(
        bool enabled,
        double intervalSeconds,
        double durationSeconds,
        uint32_t seed
    );
    RandomBlockConfig getRandomBlockConfig() const;

    SimulationSession& getSession();
    const SimulationSession& getSession() const;

private:
    SimulationSession& session;

    static bool isValidPositive(double value, double minValue, double maxValue);
};
