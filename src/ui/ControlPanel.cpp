#include "ui/ControlPanel.h"
#include <algorithm>
#include <cmath>

ControlPanel::ControlPanel(SimulationSession& session) : session(session) {
}

void ControlPanel::start() {
    session.start();
}

void ControlPanel::pause() {
    session.pause();
}

void ControlPanel::resume() {
    session.resume();
}

void ControlPanel::stop() {
    session.stop();
}

void ControlPanel::reset() {
    session.reset();
}

void ControlPanel::stepOnce() {
    session.step();
}

bool ControlPanel::isRunning() const {
    return session.isRunning();
}

bool ControlPanel::isPaused() const {
    return session.isPaused();
}

double ControlPanel::getTickDuration() const {
    return session.getTickDuration();
}

void ControlPanel::setTickDuration(double tickDuration) {
    session.setTickDuration(tickDuration);
}

bool ControlPanel::isValidPositive(double value, double minValue, double maxValue) {
    return std::isfinite(value) && value >= minValue && value <= maxValue;
}

double ControlPanel::getTimeMultiplier() const {
    return session.getTimeMultiplier();
}

bool ControlPanel::setTimeMultiplier(double multiplier) {
    if (!isValidPositive(multiplier, 0.1, 10.0)) {
        return false;
    }
    session.setTimeMultiplier(multiplier);
    return true;
}

double ControlPanel::getVehicleSpawnRate() const {
    return session.getVehicleSpawnRate();
}

bool ControlPanel::setVehicleSpawnRate(double rate) {
    if (!isValidPositive(rate, 0.1, 10.0)) {
        return false;
    }
    session.setVehicleSpawnRate(rate);
    return true;
}

double ControlPanel::getPedestrianRequestTempo() const {
    return session.getPedestrianRequestTempo();
}

bool ControlPanel::setPedestrianRequestTempo(double tempo) {
    if (!isValidPositive(tempo, 0.1, 10.0)) {
        return false;
    }
    session.setPedestrianRequestTempo(tempo);
    return true;
}

int ControlPanel::addManualLaneBlockage(
    int roadId,
    int laneIndex,
    double durationSeconds,
    double positionAlongRoad,
    double lengthMeters
) {
    if (roadId < 0 || laneIndex < 0) {
        return -1;
    }
    if (!isValidPositive(durationSeconds, 0.5, 600.0)) {
        return -1;
    }
    if (!isValidPositive(lengthMeters, 1.0, 50.0)) {
        return -1;
    }

    const auto& city = session.getCity();
    const auto* road = city.findRoadSegmentById(roadId);
    if (road == nullptr) {
        return -1;
    }
    if (laneIndex >= road->getLaneCount()) {
        return -1;
    }

    double position = positionAlongRoad;
    if (position < 0.0) {
        position = road->getLength() * 0.4;
    }
    if (position < 0.0 || position + lengthMeters > road->getLength() + 1e-6) {
        return -1;
    }

    auto& sim = session.getSimulation();
    return sim.addLaneBlockage(
        roadId,
        laneIndex,
        position,
        lengthMeters,
        sim.getTime(),
        durationSeconds,
        city
    );
}

bool ControlPanel::clearLaneBlockage(int blockageId) {
    if (blockageId < 0) {
        return false;
    }
    return session.getSimulation().clearLaneBlockage(blockageId);
}

void ControlPanel::clearAllLaneBlockages() {
    session.getSimulation().clearAllLaneBlockages();
}

bool ControlPanel::setRandomBlockages(
    bool enabled,
    double intervalSeconds,
    double durationSeconds,
    uint32_t seed
) {
    if (enabled) {
        if (!isValidPositive(intervalSeconds, 1.0, 600.0)) {
            return false;
        }
        if (!isValidPositive(durationSeconds, 0.5, 600.0)) {
            return false;
        }
    }

    RandomBlockConfig config = session.getSimulation().getRandomLaneBlockConfig();
    config.enabled = enabled;
    if (enabled) {
        config.minInterval = intervalSeconds;
        config.maxInterval = intervalSeconds;
        config.minDuration = durationSeconds;
        config.maxDuration = durationSeconds;
        config.seed = seed;
    }
    session.getSimulation().setRandomLaneBlockConfig(config);
    session.getSimulation().setLaneBlockSeed(seed);
    return true;
}

RandomBlockConfig ControlPanel::getRandomBlockConfig() const {
    return session.getSimulation().getRandomLaneBlockConfig();
}

SimulationSession& ControlPanel::getSession() {
    return session;
}

const SimulationSession& ControlPanel::getSession() const {
    return session;
}
