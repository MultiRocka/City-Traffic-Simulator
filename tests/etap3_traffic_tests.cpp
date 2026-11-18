#include "core/City.h"
#include "core/Simulation.h"
#include "core/VehicleGenerator.h"
#include "osm/OsmScenarioBuilder.h"
#include "ui/RenderComposer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace std;
namespace fs = std::filesystem;
using Clock = chrono::steady_clock;

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
    fs::create_directories("tests/output");
}

void redirectSimOutputs(Simulation& sim, const string& tag) {
    ensureOutputDir();
    const string base = "tests/output/" + tag;
    sim.setEventLogOutputFile(base + "_events.json");
    sim.setStatisticsOutputFiles(base + "_timeline.csv", base + "_summary.csv");
}

struct Etap3ProbeResult {
    string tag;
    string mapPath;
    bool ok = false;
    string error;

    uint32_t seed = 0;
    bool continuous = false;
    int maxGeneratorsConfig = 0;
    int vehiclesPerGenerator = 0;
    int configuredMaxActive = 0;
    int configuredDemandLimit = 0;
    int generatorsAttached = 0;
    int distinctStarts = 0;
    int demandCapAttached = 0;

    int generated = 0;
    int failedSpawnAttempts = 0;
    int deferredDueToActiveCap = 0;
    int peakActive = 0;
    int endActive = 0;
    int finished = 0;
    int steadyActiveApprox = 0; // median of last 20% of tick samples

    double simTime = 0.0;
    double targetTime = 0.0;
    double routePrepMs = 0.0;
    double meanTickMs = 0.0;
    double meanSnapshotMs = 0.0;
    double maxTickMs = 0.0;
    double maxSnapshotMs = 0.0;

    size_t roads = 0;
    size_t intersections = 0;
};

int countGenerated(const Simulation& sim) {
    int total = 0;
    for (const auto& g : sim.getVehicleGenerators()) {
        total += g.getGeneratedCount();
    }
    return total;
}

int countFailedSpawns(const Simulation& sim) {
    int total = 0;
    for (const auto& g : sim.getVehicleGenerators()) {
        total += g.getFailedSpawnAttempts();
    }
    return total;
}

int countDeferredCap(const Simulation& sim) {
    int total = 0;
    for (const auto& g : sim.getVehicleGenerators()) {
        total += g.getDeferredDueToActiveCap();
    }
    return total;
}

int countDistinctStarts(const Simulation& sim) {
    unordered_set<int> starts;
    for (const auto& g : sim.getVehicleGenerators()) {
        if (!g.getRoute().empty()) {
            starts.insert(g.getRoute().front());
        }
    }
    return static_cast<int>(starts.size());
}

Etap3ProbeResult runProbe(
    const string& tag,
    const string& mapPath,
    OsmScenarioConfig config,
    double targetTime
) {
    Etap3ProbeResult r;
    r.tag = tag;
    r.mapPath = mapPath;
    r.seed = config.seed;
    r.continuous = config.continuousDemand;
    r.maxGeneratorsConfig = config.maxGenerators;
    r.vehiclesPerGenerator = config.vehiclesPerGenerator;
    r.configuredMaxActive = config.maxActiveVehicles;
    r.configuredDemandLimit = config.continuousDemand
        ? config.maxActiveVehicles
        : (config.maxGenerators * config.vehiclesPerGenerator);
    r.targetTime = targetTime;

    if (!fs::exists(mapPath)) {
        r.error = "missing map";
        cout << "SKIP  " << tag << " missing " << mapPath << "\n";
        return r;
    }

    City city;
    Simulation sim;
    redirectSimOutputs(sim, tag);

    const auto t0 = Clock::now();
    const OsmScenarioResult loaded = OsmScenarioBuilder::loadOsmFile(mapPath, city, sim, config);
    const auto t1 = Clock::now();
    r.routePrepMs = chrono::duration<double, milli>(t1 - t0).count();

    if (!loaded.ok) {
        r.error = loaded.error;
        expect(false, tag + "_load_ok");
        return r;
    }
    expect(true, tag + "_load_ok");

    r.ok = true;
    r.generatorsAttached = static_cast<int>(loaded.attachedGeneratorCount);
    r.distinctStarts = countDistinctStarts(sim);
    r.demandCapAttached = config.continuousDemand
        ? config.maxActiveVehicles
        : (r.generatorsAttached * config.vehiclesPerGenerator);
    r.roads = city.getRoadSegments().size();
    r.intersections = city.getIntersections().size();

    expect(sim.getMaxActiveVehicles() == max(0, config.maxActiveVehicles), tag + "_max_active_applied");
    if (config.spatiallyDistributeStarts && r.generatorsAttached >= 8) {
        expect(r.distinctStarts >= 4, tag + "_spatial_starts");
    }

    RenderComposer composer;
    const double dt = 0.1;
    double tickAccum = 0.0;
    double snapAccum = 0.0;
    int ticks = 0;
    int snapSamples = 0;
    vector<int> activeSeries;
    activeSeries.reserve(static_cast<size_t>(targetTime / dt) + 8);

    sim.start();
    while (sim.getTime() < targetTime - 1e-9) {
        const auto tt0 = Clock::now();
        sim.update(dt, city);
        const auto tt1 = Clock::now();
        const double tickMs = chrono::duration<double, milli>(tt1 - tt0).count();
        tickAccum += tickMs;
        r.maxTickMs = max(r.maxTickMs, tickMs);

        if (ticks % 10 == 0) {
            const auto ts0 = Clock::now();
            const RenderSnapshot snap = composer.buildSnapshot(city, sim);
            const auto ts1 = Clock::now();
            const double snapMs = chrono::duration<double, milli>(ts1 - ts0).count();
            snapAccum += snapMs;
            r.maxSnapshotMs = max(r.maxSnapshotMs, snapMs);
            ++snapSamples;
            (void)snap;
        }

        const auto& stats = sim.getStatisticsCollector().getLastSnapshot();
        r.peakActive = max(r.peakActive, stats.activeVehicles);
        activeSeries.push_back(stats.activeVehicles);
        ++ticks;
    }

    const auto& endStats = sim.getStatisticsCollector().getLastSnapshot();
    r.generated = countGenerated(sim);
    r.failedSpawnAttempts = countFailedSpawns(sim);
    r.deferredDueToActiveCap = countDeferredCap(sim);
    r.endActive = endStats.activeVehicles;
    r.finished = endStats.finishedVehicles;
    r.simTime = sim.getTime();
    r.meanTickMs = ticks > 0 ? tickAccum / ticks : 0.0;
    r.meanSnapshotMs = snapSamples > 0 ? snapAccum / snapSamples : 0.0;

    if (!activeSeries.empty()) {
        const size_t start = (activeSeries.size() * 4) / 5;
        vector<int> tail(activeSeries.begin() + static_cast<long>(start), activeSeries.end());
        sort(tail.begin(), tail.end());
        r.steadyActiveApprox = tail[tail.size() / 2];
    }

    expect(r.simTime >= targetTime - 1.0, tag + "_ran_target");
    if (config.maxActiveVehicles > 0) {
        expect(r.peakActive <= config.maxActiveVehicles, tag + "_peak_le_configured_max");
    }
    if (!config.continuousDemand && r.demandCapAttached > 0) {
        expect(r.generated <= r.demandCapAttached, tag + "_generated_le_quota");
    }

    cout << "ETAP3 " << tag
         << " continuous=" << (r.continuous ? 1 : 0)
         << " cfg_max_active=" << r.configuredMaxActive
         << " attached=" << r.generatorsAttached
         << " starts=" << r.distinctStarts
         << " generated=" << r.generated
         << " failed_spawns=" << r.failedSpawnAttempts
         << " deferred_cap=" << r.deferredDueToActiveCap
         << " peak_active=" << r.peakActive
         << " steady_active~=" << r.steadyActiveApprox
         << " end_active=" << r.endActive
         << " finished=" << r.finished
         << " tick_ms=" << r.meanTickMs
         << " snap_ms=" << r.meanSnapshotMs
         << " route_prep_ms=" << r.routePrepMs
         << "\n";

    return r;
}

void writeProbe(ostream& out, const Etap3ProbeResult& r) {
    out << "\n## " << r.tag << "\n";
    out << "map=" << r.mapPath << "\n";
    out << "ok=" << (r.ok ? 1 : 0) << "\n";
    if (!r.error.empty()) {
        out << "error=" << r.error << "\n";
    }
    out << "seed=" << r.seed << "\n";
    out << "continuous=" << (r.continuous ? 1 : 0) << "\n";
    out << "max_generators_config=" << r.maxGeneratorsConfig << "\n";
    out << "vehicles_per_generator=" << r.vehiclesPerGenerator << "\n";
    out << "configured_max_active=" << r.configuredMaxActive << "\n";
    out << "configured_demand_or_cap=" << r.configuredDemandLimit << "\n";
    out << "generators_attached=" << r.generatorsAttached << "\n";
    out << "distinct_starts=" << r.distinctStarts << "\n";
    out << "demand_cap_attached=" << r.demandCapAttached << "\n";
    out << "generated=" << r.generated << "\n";
    out << "failed_spawn_attempts=" << r.failedSpawnAttempts << "\n";
    out << "deferred_due_to_active_cap=" << r.deferredDueToActiveCap << "\n";
    out << "peak_active=" << r.peakActive << "\n";
    out << "steady_active_approx=" << r.steadyActiveApprox << "\n";
    out << "end_active=" << r.endActive << "\n";
    out << "finished=" << r.finished << "\n";
    out << "sim_time=" << r.simTime << " target_time=" << r.targetTime << "\n";
    out << "route_prep_ms=" << r.routePrepMs << "\n";
    out << "mean_tick_ms=" << r.meanTickMs << " max_tick_ms=" << r.maxTickMs << "\n";
    out << "mean_snapshot_ms=" << r.meanSnapshotMs << " max_snapshot_ms=" << r.maxSnapshotMs << "\n";
    out << "roads=" << r.roads << " intersections=" << r.intersections << "\n";
    const double spawnAttempts = static_cast<double>(r.generated + r.failedSpawnAttempts);
    out << "spawn_success_ratio_approx="
        << (spawnAttempts > 0.0 ? static_cast<double>(r.generated) / spawnAttempts : 0.0) << "\n";
    out << "peak_vs_configured_max="
        << (r.configuredMaxActive > 0
                ? static_cast<double>(r.peakActive) / static_cast<double>(r.configuredMaxActive)
                : 0.0)
        << "\n";
}

OsmScenarioConfig makeContinuous(uint32_t seed, int maxGen, int maxActive, double interval) {
    OsmScenarioConfig c;
    c.seed = seed;
    c.continuousDemand = true;
    c.maxGenerators = maxGen;
    c.vehiclesPerGenerator = 0;
    c.maxActiveVehicles = maxActive;
    c.spawnIntervalSeconds = interval;
    c.spatiallyDistributeStarts = true;
    c.phaseGreenSeconds = 8.0;
    c.minVehicleGap = 4.0;
    c.yieldRefusalProbability = 0.05;
    c.maxRouteHops = 12;
    c.maxRouteSearchAttempts = max(240, maxGen * 16);
    return c;
}

OsmScenarioConfig makeFinite(uint32_t seed, int maxGen, int perGen, double interval) {
    OsmScenarioConfig c;
    c.seed = seed;
    c.continuousDemand = false;
    c.maxGenerators = maxGen;
    c.vehiclesPerGenerator = perGen;
    c.maxActiveVehicles = 0;
    c.spawnIntervalSeconds = interval;
    c.spatiallyDistributeStarts = true;
    c.phaseGreenSeconds = 8.0;
    c.minVehicleGap = 4.0;
    c.yieldRefusalProbability = 0.05;
    c.maxRouteHops = 10;
    return c;
}

} // namespace

int runEtap3TrafficTests() {
    g_failures = 0;
    g_passed = 0;
    ensureOutputDir();
    cout << "\n=== Etap 3 continuous traffic / presets ===\n";

    vector<Etap3ProbeResult> results;
    const char* small = "data/osm/warsaw_sample.overpass.json";
    const char* larger = "data/osm/warsaw_1km.overpass.json";
    const uint32_t seed = VehicleGenerator::defaultSeed;

    // Progressive continuous probes: configured maxActive 100→200→500→1000.
    // Achieved peak/steady are measured; maxActive is a soft cap, not a claim of fill.
    struct ContSpec {
        const char* tag;
        const char* path;
        int maxGen;
        int maxActive;
        double interval;
        double simSeconds;
    };
    const vector<ContSpec> continuousSpecs = {
        {"e3_cont_sample_80", small, 12, 80, 1.5, 120.0},
        {"e3_cont_1km_100", larger, 16, 100, 1.2, 160.0},
        {"e3_cont_1km_200", larger, 24, 200, 1.0, 180.0},
        {"e3_cont_1km_500", larger, 36, 500, 0.8, 200.0},
        {"e3_cont_1km_1000", larger, 48, 1000, 0.6, 220.0}
    };

    for (const auto& spec : continuousSpecs) {
        if (!fs::exists(spec.path)) {
            cout << "SKIP  " << spec.tag << "\n";
            continue;
        }
        results.push_back(runProbe(
            spec.tag,
            spec.path,
            makeContinuous(seed, spec.maxGen, spec.maxActive, spec.interval),
            spec.simSeconds
        ));
    }

    // Named presets (visible limit + intensity).
    if (fs::exists(larger)) {
        results.push_back(runProbe(
            "e3_preset_small", larger, OsmScenarioConfig::presetSmall(seed), 150.0));
        results.push_back(runProbe(
            "e3_preset_medium", larger, OsmScenarioConfig::presetMedium(seed), 180.0));
        results.push_back(runProbe(
            "e3_preset_large", larger, OsmScenarioConfig::presetLarge(seed), 200.0));
    }

    // Finite-demand before-style probe with spatial starts (comparability with Etap 1).
    if (fs::exists(larger)) {
        results.push_back(runProbe(
            "e3_finite_1km_100",
            larger,
            makeFinite(seed, 10, 10, 1.2),
            140.0
        ));
        results.push_back(runProbe(
            "e3_finite_1km_200",
            larger,
            makeFinite(seed, 16, 13, 1.0),
            150.0
        ));
    }

    // Success bar: on warsaw_1km, continuous traffic clearly above historical ~40.
    Etap3ProbeResult* mediumOr200 = nullptr;
    for (auto& r : results) {
        if (r.tag == "e3_preset_medium" || r.tag == "e3_cont_1km_200") {
            if (r.ok) {
                mediumOr200 = &r;
            }
        }
    }
    if (mediumOr200 != nullptr) {
        expect(mediumOr200->peakActive > 40, "e3_success_peak_gt_40");
        expect(mediumOr200->steadyActiveApprox > 40, "e3_success_steady_gt_40");
    }

    // Determinism: continuous short pair.
    if (fs::exists(small)) {
        OsmScenarioConfig cfg = makeContinuous(seed + 77u, 8, 40, 1.5);
        Etap3ProbeResult a = runProbe("e3_det_a", small, cfg, 60.0);
        Etap3ProbeResult b = runProbe("e3_det_b", small, cfg, 60.0);
        expect(a.ok && b.ok, "e3_det_both_ok");
        expect(a.generated == b.generated, "e3_det_generated");
        expect(a.peakActive == b.peakActive, "e3_det_peak_active");
        expect(a.failedSpawnAttempts == b.failedSpawnAttempts, "e3_det_failed_spawns");
        expect(a.deferredDueToActiveCap == b.deferredDueToActiveCap, "e3_det_deferred_cap");
        results.push_back(a);
        results.push_back(b);
    }

    ofstream report("tests/output/etap3_traffic_baseline.txt");
    report << "Etap 3 continuous traffic / presets\n";
    report << "deterministic_seed_base=" << seed << "\n";
    report << "note=configured_max_active is a soft concurrent cap; peak_active is achieved.\n";
    report << "note=failed_spawn_attempts = entrance blocked; deferred_due_to_active_cap = at soft cap.\n";
    report << "note=continuousDemand uses vehiclesPerGenerator=0 (unlimited quota per generator).\n";
    for (const auto& r : results) {
        writeProbe(report, r);
    }
    report.close();

    ofstream csv("tests/output/etap3_traffic_baseline.csv");
    csv << "tag,map,ok,seed,continuous,cfg_max_active,attached,distinct_starts,generated,"
           "failed_spawns,deferred_cap,peak_active,steady_active,end_active,finished,"
           "route_prep_ms,mean_tick_ms,mean_snapshot_ms,sim_time\n";
    for (const auto& r : results) {
        csv << r.tag << ","
            << r.mapPath << ","
            << (r.ok ? 1 : 0) << ","
            << r.seed << ","
            << (r.continuous ? 1 : 0) << ","
            << r.configuredMaxActive << ","
            << r.generatorsAttached << ","
            << r.distinctStarts << ","
            << r.generated << ","
            << r.failedSpawnAttempts << ","
            << r.deferredDueToActiveCap << ","
            << r.peakActive << ","
            << r.steadyActiveApprox << ","
            << r.endActive << ","
            << r.finished << ","
            << r.routePrepMs << ","
            << r.meanTickMs << ","
            << r.meanSnapshotMs << ","
            << r.simTime << "\n";
    }
    csv.close();

    return g_failures;
}

int etap3TrafficPassedCount() {
    return g_passed;
}
