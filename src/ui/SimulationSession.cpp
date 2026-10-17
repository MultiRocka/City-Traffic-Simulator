#include "ui/SimulationSession.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace {

std::string basenameOf(const std::string& path) {
    const auto slash = path.find_last_of("/\\");
    if (slash == std::string::npos) {
        return path;
    }
    return path.substr(slash + 1);
}

} // namespace

SimulationSession::SimulationSession()
    : loadedScenario(SessionScenario::Demo),
      tickDuration(simulationDt),
      paused(false),
      timeMultiplier(1.0),
      vehicleSpawnRate(1.0),
      pedestrianRequestTempo(1.0),
      simulationAccumulator(0.0),
      interpolationAlpha(1.0),
      hasPreviousSnapshot(false) {
    simulation.setStatisticsOutputFiles("simulation_timeline.csv", "simulation_summary.csv");
    simulation.setEventLogOutputFile("event_log.json");
}

void SimulationSession::gateBeforeScenarioSwap() {
    // Never replace city/sim while a frame is mid-update: stop first.
    simulation.stop();
    paused = false;
    simulationAccumulator = 0.0;
}

void SimulationSession::finishScenarioLoad() {
    applyRuntimeParameters();
    currentSnapshot = renderComposer.buildSnapshot(city, simulation);
    previousSnapshot = currentSnapshot;
    hasPreviousSnapshot = true;
    interpolationAlpha = 1.0;
    simulationAccumulator = 0.0;
}

void SimulationSession::loadDemoScenario() {
    gateBeforeScenarioSwap();
    loadedScenario = SessionScenario::Demo;
    osmSourcePath.clear();
    lastImportSummary.clear();
    simulation.clear();
    Scenario::setupDemoCity(city, simulation);
    simulation.captureVehicleSpawnBaselines();
    finishScenarioLoad();
}

void SimulationSession::loadIntersectionTestScenario() {
    gateBeforeScenarioSwap();
    loadedScenario = SessionScenario::IntersectionTest;
    osmSourcePath.clear();
    lastImportSummary.clear();
    simulation.clear();
    Scenario::setupIntersectionTest(city, simulation);
    simulation.captureVehicleSpawnBaselines();
    finishScenarioLoad();
}

OsmScenarioResult SimulationSession::loadOsmFromFile(
    const std::string& path,
    const OsmScenarioConfig& config
) {
    gateBeforeScenarioSwap();

    const OsmScenarioResult result = OsmScenarioBuilder::loadOsmFile(path, city, simulation, config);
    if (!result.ok) {
        // City / Simulation / mode unchanged. Session is stopped; caller may Start again.
        captureSimulationFrame();
        return result;
    }

    loadedScenario = SessionScenario::Osm;
    osmSourcePath = path;
    lastImportSummary = formatImportSummary(result);
    simulation.captureVehicleSpawnBaselines();
    finishScenarioLoad();
    return result;
}

std::string SimulationSession::formatImportSummary(const OsmScenarioResult& result) {
    std::ostringstream out;
    const std::string source = result.report.sourceFile.empty()
        ? "(unknown source)"
        : basenameOf(result.report.sourceFile);

    out << "OSM import: " << source << "\n";
    out << "Intersections: " << result.report.builtIntersectionCount
        << "  Roads: " << result.report.builtRoadCount
        << "  Lanes (dir sum): " << result.report.builtLaneCount << "\n";
    out << "Crossings: " << result.attachedCrossingCount
        << "  Generators: " << result.attachedGeneratorCount
        << "  Routes: " << result.routeCount << "\n";
    out << "Generated signals: " << result.generatedPhaseControllerCount
        << " controllers / " << result.attachedTrafficLightCount << " lights"
        << " (timing NOT from OSM)\n";

    if (!result.report.warnings.empty()) {
        out << "Warnings: " << result.report.warnings.size();
        const size_t show = (std::min)(result.report.warnings.size(), size_t{3});
        for (size_t i = 0; i < show; ++i) {
            out << "\n  - " << result.report.warnings[i];
        }
        if (result.report.warnings.size() > show) {
            out << "\n  - ... +" << (result.report.warnings.size() - show) << " more";
        }
        out << "\n";
    } else {
        out << "Warnings: 0\n";
    }

    for (const auto& note : result.notes) {
        if (note.find("Generated") != std::string::npos &&
            note.find("PhaseController") != std::string::npos) {
            out << note << "\n";
            break;
        }
    }

    if (!result.report.routingLimitsNote.empty()) {
        out << result.report.routingLimitsNote << "\n";
    } else if (!result.report.incompletenessNote.empty()) {
        out << result.report.incompletenessNote << "\n";
    }

    return out.str();
}

SessionScenario SimulationSession::getLoadedScenario() const {
    return loadedScenario;
}

const std::string& SimulationSession::getOsmSourcePath() const {
    return osmSourcePath;
}

const std::string& SimulationSession::getLastImportSummary() const {
    return lastImportSummary;
}

void SimulationSession::reloadCurrentScenario() {
    if (loadedScenario == SessionScenario::IntersectionTest) {
        loadIntersectionTestScenario();
    } else if (loadedScenario == SessionScenario::Osm && !osmSourcePath.empty()) {
        const std::string path = osmSourcePath;
        const OsmScenarioResult result = loadOsmFromFile(path);
        if (!result.ok) {
            // Keep prior map (builder left city/sim unchanged). Snapshots already refreshed.
            lastImportSummary = "Reset re-import failed: " + result.error;
        }
    } else {
        loadDemoScenario();
    }
}

void SimulationSession::applyRuntimeParameters() {
    simulation.setVehicleSpawnRateMultiplier(vehicleSpawnRate);
    simulation.setPedestrianRequestTempo(pedestrianRequestTempo);
}

void SimulationSession::start() {
    simulation.start();
    paused = false;
    simulationAccumulator = 0.0;
    captureSimulationFrame();
}

void SimulationSession::pause() {
    paused = true;
}

void SimulationSession::resume() {
    if (simulation.isRunning()) {
        paused = false;
        simulationAccumulator = 0.0;
    }
}

void SimulationSession::stop() {
    simulation.stop();
    paused = false;
    simulationAccumulator = 0.0;
    captureSimulationFrame();
}

void SimulationSession::reset() {
    simulation.stop();
    paused = false;
    simulationAccumulator = 0.0;
    interpolationAlpha = 1.0;
    hasPreviousSnapshot = false;
    // Keep user-selected tempo / multiplier across reset; re-apply after scenario reload.
    // Mode preserved: Demo / IntersectionTest / OSM path.
    reloadCurrentScenario();
}

void SimulationSession::captureSimulationFrame() {
    previousSnapshot = currentSnapshot;
    hasPreviousSnapshot = true;
    currentSnapshot = renderComposer.buildSnapshot(city, simulation);
    interpolationAlpha = 1.0;
}

void SimulationSession::step() {
    if (!simulation.isRunning() || paused) {
        return;
    }

    previousSnapshot = currentSnapshot;
    hasPreviousSnapshot = true;
    simulation.update(tickDuration, city);
    currentSnapshot = renderComposer.buildSnapshot(city, simulation);
    interpolationAlpha = 1.0;
}

void SimulationSession::update(double frameDt) {
    if (!simulation.isRunning() || paused) {
        interpolationAlpha = 1.0;
        return;
    }

    if (frameDt < 0.0) {
        frameDt = 0.0;
    }

    // Scale wall time into sim time; physics steps always use fixed simulationDt.
    double simAdvance = frameDt * timeMultiplier;
    if (simAdvance > maxCatchUpSimSeconds) {
        simAdvance = maxCatchUpSimSeconds;
    }

    simulationAccumulator += simAdvance;
    if (simulationAccumulator > maxCatchUpSimSeconds) {
        simulationAccumulator = maxCatchUpSimSeconds;
    }

    while (simulationAccumulator >= simulationDt) {
        previousSnapshot = currentSnapshot;
        hasPreviousSnapshot = true;
        simulation.update(simulationDt, city);
        currentSnapshot = renderComposer.buildSnapshot(city, simulation);
        simulationAccumulator -= simulationDt;
    }
    interpolationAlpha = 0.0;

    if (hasPreviousSnapshot) {
        interpolationAlpha = simulationAccumulator / simulationDt;
    } else {
        currentSnapshot = renderComposer.buildSnapshot(city, simulation);
        interpolationAlpha = 1.0;
    }
}

bool SimulationSession::isRunning() const {
    return simulation.isRunning();
}

bool SimulationSession::isPaused() const {
    return paused;
}

double SimulationSession::getTickDuration() const {
    return tickDuration;
}

void SimulationSession::setTickDuration(double newTickDuration) {
    tickDuration = newTickDuration;
}

double SimulationSession::getTimeMultiplier() const {
    return timeMultiplier;
}

void SimulationSession::setTimeMultiplier(double multiplier) {
    if (multiplier < 0.1) {
        multiplier = 0.1;
    }
    if (multiplier > 10.0) {
        multiplier = 10.0;
    }
    timeMultiplier = multiplier;
}

double SimulationSession::getVehicleSpawnRate() const {
    return vehicleSpawnRate;
}

void SimulationSession::setVehicleSpawnRate(double rate) {
    if (rate < 0.1) {
        rate = 0.1;
    }
    if (rate > 10.0) {
        rate = 10.0;
    }
    vehicleSpawnRate = rate;
    simulation.setVehicleSpawnRateMultiplier(vehicleSpawnRate);
}

double SimulationSession::getPedestrianRequestTempo() const {
    return pedestrianRequestTempo;
}

void SimulationSession::setPedestrianRequestTempo(double tempo) {
    if (tempo < 0.1) {
        tempo = 0.1;
    }
    if (tempo > 10.0) {
        tempo = 10.0;
    }
    pedestrianRequestTempo = tempo;
    simulation.setPedestrianRequestTempo(pedestrianRequestTempo);
}

City& SimulationSession::getCity() {
    return city;
}

const City& SimulationSession::getCity() const {
    return city;
}

Simulation& SimulationSession::getSimulation() {
    return simulation;
}

const Simulation& SimulationSession::getSimulation() const {
    return simulation;
}

RenderSnapshot SimulationSession::buildRenderSnapshot() const {
    return buildDisplaySnapshot();
}

RenderSnapshot SimulationSession::buildDisplaySnapshot() const {
    if (!hasPreviousSnapshot) {
        return currentSnapshot;
    }

    return renderComposer.interpolate(previousSnapshot, currentSnapshot, interpolationAlpha);
}

void SimulationSession::exportResults() const {
    simulation.exportStatisticsSummary();
    simulation.exportEventLog();
}
