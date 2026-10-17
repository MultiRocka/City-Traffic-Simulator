#include "core/Scenario.h"
#include "osm/OsmScenarioBuilder.h"
#include "ui/SimulationSession.h"

#include <filesystem>
#include <iostream>
#include <string>

using namespace std;
namespace fs = std::filesystem;

namespace {

int g_failures = 0;
int g_passed = 0;

void expect(bool condition, const string& name) {
    if (condition) {
        ++g_passed;
        cout << "PASS  " << name << "\n";
    } else {
        ++g_failures;
        cout << "FAIL  " << name << "\n";
    }
}

void ensureOutputDir() {
    filesystem::create_directories("tests/output");
}

void redirectSessionOutputs(SimulationSession& session, const string& tag) {
    ensureOutputDir();
    const string base = "tests/output/" + tag;
    session.getSimulation().setEventLogOutputFile(base + "_events.json");
    session.getSimulation().setStatisticsOutputFiles(base + "_timeline.csv", base + "_summary.csv");
}

string fixturePath(const string& name) {
    return string("data/osm/fixtures/") + name;
}

void testFailedLoadKeepsPreviousDemo() {
    SimulationSession session;
    redirectSessionOutputs(session, "osm5_fail_keep_demo");
    session.loadDemoScenario();

    const string demoName = session.getCity().getName();
    const size_t demoRoads = session.getCity().getRoadSegments().size();
    const size_t demoGens = session.getSimulation().getVehicleGenerators().size();

    expect(session.getLoadedScenario() == SessionScenario::Demo, "osm5_fail_mode_demo");
    expect(demoRoads > 0, "osm5_fail_demo_has_roads");

    session.start();
    expect(session.isRunning(), "osm5_fail_started");

    const OsmScenarioResult failed = session.loadOsmFromFile(fixturePath("invalid_not_json.txt"));
    expect(!failed.ok, "osm5_fail_ok_false");
    expect(!failed.error.empty(), "osm5_fail_has_error");
    expect(session.getLoadedScenario() == SessionScenario::Demo, "osm5_fail_mode_unchanged");
    expect(session.getOsmSourcePath().empty(), "osm5_fail_path_empty");
    expect(session.getCity().getName() == demoName, "osm5_fail_city_name");
    expect(session.getCity().getRoadSegments().size() == demoRoads, "osm5_fail_roads");
    expect(session.getSimulation().getVehicleGenerators().size() == demoGens, "osm5_fail_gens");
    expect(!session.isRunning(), "osm5_fail_stopped_after_attempt");
}

void testResetKeepsOsmMap() {
    const string path = fixturePath("shared_junction.overpass.json");
    expect(fs::exists(path), "osm5_reset_fixture_exists");

    SimulationSession session;
    redirectSessionOutputs(session, "osm5_reset_osm");
    session.loadDemoScenario();

    const OsmScenarioResult loaded = session.loadOsmFromFile(path);
    expect(loaded.ok, "osm5_reset_load_ok");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm5_reset_mode_osm");
    expect(session.getOsmSourcePath() == path, "osm5_reset_path_stored");
    expect(!session.getLastImportSummary().empty(), "osm5_reset_has_summary");

    const size_t roadsBefore = session.getCity().getRoadSegments().size();
    const size_t gensBefore = session.getSimulation().getVehicleGenerators().size();
    const string nameBefore = session.getCity().getName();
    expect(roadsBefore > 0, "osm5_reset_has_roads");
    expect(gensBefore > 0, "osm5_reset_has_gens");

    session.start();
    session.update(SimulationSession::renderDt);
    session.update(SimulationSession::renderDt);
    expect(session.isRunning(), "osm5_reset_was_running");

    session.reset();
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm5_reset_mode_kept");
    expect(session.getOsmSourcePath() == path, "osm5_reset_path_kept");
    expect(!session.isRunning(), "osm5_reset_stopped");
    expect(session.getCity().getRoadSegments().size() == roadsBefore, "osm5_reset_roads_same");
    expect(session.getSimulation().getVehicleGenerators().size() == gensBefore, "osm5_reset_gens_same");
    expect(session.getCity().getName() == nameBefore, "osm5_reset_name_same");
    expect(session.getSimulation().getVehicles().empty(), "osm5_reset_no_vehicles");
    expect(session.getSimulation().getTime() == 0.0, "osm5_reset_time_zero");

    // Can restart the same OSM scenario after reset.
    session.start();
    expect(session.isRunning(), "osm5_reset_restart_ok");
}

void testDemoCityAfterOsm() {
    const string path = fixturePath("shared_junction.overpass.json");
    if (!fs::exists(path)) {
        cout << "SKIP  osm5_demo_after_osm (fixture missing)\n";
        return;
    }

    SimulationSession session;
    redirectSessionOutputs(session, "osm5_demo_after");
    const OsmScenarioResult loaded = session.loadOsmFromFile(path);
    expect(loaded.ok, "osm5_demo_after_load");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm5_demo_after_was_osm");

    session.loadDemoScenario();
    expect(session.getLoadedScenario() == SessionScenario::Demo, "osm5_demo_after_mode");
    expect(session.getOsmSourcePath().empty(), "osm5_demo_after_path_cleared");
    expect(session.getCity().getName().find("Demo") != string::npos, "osm5_demo_after_name");
}

void testFailedLoadKeepsPreviousOsm() {
    const string path = fixturePath("shared_junction.overpass.json");
    if (!fs::exists(path)) {
        cout << "SKIP  osm5_fail_keep_osm (fixture missing)\n";
        return;
    }

    SimulationSession session;
    redirectSessionOutputs(session, "osm5_fail_keep_osm");
    const OsmScenarioResult loaded = session.loadOsmFromFile(path);
    expect(loaded.ok, "osm5_keep_osm_load");

    const size_t roads = session.getCity().getRoadSegments().size();
    const OsmScenarioResult failed = session.loadOsmFromFile(fixturePath("invalid_not_json.txt"));
    expect(!failed.ok, "osm5_keep_osm_fail");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "osm5_keep_osm_mode");
    expect(session.getOsmSourcePath() == path, "osm5_keep_osm_path");
    expect(session.getCity().getRoadSegments().size() == roads, "osm5_keep_osm_roads");
}

} // namespace

int runOsmSessionTests() {
    cout << "\n--- OSM session / GUI-path tests (Stage 5) ---\n";
    testFailedLoadKeepsPreviousDemo();
    testResetKeepsOsmMap();
    testDemoCityAfterOsm();
    testFailedLoadKeepsPreviousOsm();
    return g_failures;
}

int osmSessionPassedCount() {
    return g_passed;
}
