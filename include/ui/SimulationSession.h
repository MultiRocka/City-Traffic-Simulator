#pragma once

#include "core/City.h"
#include "core/Simulation.h"
#include "core/Scenario.h"
#include "osm/OsmScenarioBuilder.h"
#include "ui/RenderComposer.h"
#include "ui/RenderSnapshot.h"

#include <string>

enum class SessionScenario {
    Demo,
    IntersectionTest,
    Osm
};

class SimulationSession {
public:
    static constexpr double simulationHz = 10.0;
    static constexpr double simulationDt = 1.0 / simulationHz;
    static constexpr double renderHz = 60.0;
    static constexpr double renderDt = 1.0 / renderHz;
    // Cap sim-time catch-up per wall frame so a lagged render cannot jump many seconds.
    // Physics step size remains simulationDt regardless of time multiplier.
    static constexpr double maxCatchUpSimSeconds = 0.5;

    SimulationSession();

    void loadDemoScenario();
    void loadIntersectionTestScenario();

    // Validate + build fully before swapping. On failure, city/sim/mode stay as they were.
    // Stops the running session first so the swap never happens mid-tick.
    OsmScenarioResult loadOsmFromFile(
        const std::string& path,
        const OsmScenarioConfig& config = {}
    );

    SessionScenario getLoadedScenario() const;
    const std::string& getOsmSourcePath() const;
    const std::string& getLastImportSummary() const;

    // Short multi-line import report (counts, warnings, generated-signals note).
    static std::string formatImportSummary(const OsmScenarioResult& result);

    void start();
    void pause();
    void resume();
    void stop();
    // Clears traffic and reloads the current mode (Demo / IntersectionTest / OSM path).
    void reset();
    void step();
    void update(double frameDt);

    bool isRunning() const;
    bool isPaused() const;
    double getTickDuration() const;
    void setTickDuration(double tickDuration);

    // Stage 6: wall-clock → sim-time scale. Does NOT change physics dt.
    double getTimeMultiplier() const;
    void setTimeMultiplier(double multiplier);

    double getVehicleSpawnRate() const;
    void setVehicleSpawnRate(double rate);

    double getPedestrianRequestTempo() const;
    void setPedestrianRequestTempo(double tempo);

    City& getCity();
    const City& getCity() const;
    Simulation& getSimulation();
    const Simulation& getSimulation() const;

    RenderSnapshot buildRenderSnapshot() const;
    RenderSnapshot buildDisplaySnapshot() const;
    void exportResults() const;

private:
    void captureSimulationFrame();
    void applyRuntimeParameters();
    void reloadCurrentScenario();
    void gateBeforeScenarioSwap();
    void finishScenarioLoad();

    City city;
    Simulation simulation;
    RenderComposer renderComposer;
    SessionScenario loadedScenario;
    std::string osmSourcePath;
    std::string lastImportSummary;
    double tickDuration;
    bool paused;

    double timeMultiplier;
    double vehicleSpawnRate;
    double pedestrianRequestTempo;

    double simulationAccumulator;
    double interpolationAlpha;
    bool hasPreviousSnapshot;
    RenderSnapshot previousSnapshot;
    RenderSnapshot currentSnapshot;
};
