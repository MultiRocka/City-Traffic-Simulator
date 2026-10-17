#include "osm/DownloadCache.h"
#include "osm/FakeHttpTransport.h"
#include "osm/GeoBbox.h"
#include "osm/MapDownloadService.h"
#include "osm/OverpassQuery.h"
#include "ui/SimulationSession.h"

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

using namespace std;
using namespace osm_net;
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

string testCacheRoot() {
    return "tests/output/cache_stage3";
}

void resetTestCache() {
    std::error_code ec;
    fs::remove_all(fs::u8path(testCacheRoot()), ec);
    fs::create_directories(fs::u8path(testCacheRoot() + "/geocode"), ec);
    fs::create_directories(fs::u8path(testCacheRoot() + "/overpass"), ec);
}

string readFixture(const string& name) {
    ifstream in(fs::u8path(string("data/osm/fixtures/") + name), ios::binary);
    ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

HttpResponse okBody(const string& body, int status = 200) {
    HttpResponse r;
    r.ok = true;
    r.statusCode = status;
    r.body = body;
    return r;
}

void testBboxFromRadius() {
    // At equator, 111320 m ≈ 1 degree lat; 500 m → ~0.00449 deg.
    const GeoBbox box = bboxFromRadiusMeters(0.0, 0.0, 500.0);
    expect(box.isValid(), "dl_bbox_valid");
    expect(abs(box.north - 0.004491) < 1e-5, "dl_bbox_north");
    expect(abs(box.south + 0.004491) < 1e-5, "dl_bbox_south");
    expect(abs(box.east - box.west - (box.north - box.south)) < 1e-6, "dl_bbox_square_at_equator");
    expect(box.areaKm2() > 0.9 && box.areaKm2() < 1.1, "dl_bbox_area_approx_1km2");

    // Higher latitude: lon span widens for same radius.
    const GeoBbox warsaw = bboxFromRadiusMeters(52.23, 21.01, 500.0);
    expect(warsaw.isValid(), "dl_bbox_warsaw_valid");
    expect((warsaw.east - warsaw.west) > (warsaw.north - warsaw.south), "dl_bbox_warsaw_lon_wider");
}

void testRadiusLimits() {
    string err;
    expect(radiusAllowed(500.0, err), "dl_radius_500_ok");
    expect(!radiusAllowed(5000.0, err), "dl_radius_5000_reject");
    expect(!err.empty(), "dl_radius_5000_error");
    expect(!radiusAllowed(0.0, err), "dl_radius_zero_reject");
    expect(!radiusAllowed(-10.0, err), "dl_radius_neg_reject");
}

void testOverpassQueryContainsRestrictions() {
    const GeoBbox box = bboxFromRadiusMeters(52.2258, 21.0174, 200.0);
    const string q = buildRoadNetworkQuery(box);
    expect(q.find("[\"type\"=\"restriction\"]") != string::npos, "dl_query_has_restriction");
    expect(q.find("highway") != string::npos, "dl_query_has_highway");
    expect(q.find(box.toOverpassBBox()) != string::npos, "dl_query_has_bbox");
}

void testGeocodeTimeout() {
    resetTestCache();
    FakeHttpTransport fake;
    HttpResponse timed;
    timed.timedOut = true;
    timed.error = "HTTP request timed out";
    fake.enqueue(FakeHttpScript{"nominatim", timed});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const GeocodeResult r = svc.searchPlaces("Warszawa");
    expect(!r.ok, "dl_geo_timeout_fail");
    expect(r.error.find("czas") != string::npos || r.error.find("time") != string::npos
           || r.error.find("Limit") != string::npos, "dl_geo_timeout_msg");
    expect(fake.callCount() == 1, "dl_geo_timeout_one_call");
}

void testGeocodeHttpError() {
    resetTestCache();
    FakeHttpTransport fake;
    HttpResponse err;
    err.statusCode = 500;
    err.error = "HTTP status 500";
    fake.enqueue(FakeHttpScript{"nominatim", err});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const GeocodeResult r = svc.searchPlaces("Krakow");
    expect(!r.ok, "dl_geo_http_fail");
    expect(!r.error.empty(), "dl_geo_http_msg");
}

void testGeocodeBadJson() {
    resetTestCache();
    FakeHttpTransport fake;
    fake.enqueue(FakeHttpScript{"nominatim", okBody("<html>error</html>")});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const GeocodeResult r = svc.searchPlaces("Gdansk");
    expect(!r.ok, "dl_geo_bad_json_fail");
}

void testGeocodeCancel() {
    resetTestCache();
    FakeHttpTransport fake;
    FakeHttpScript script;
    script.urlContains = "nominatim";
    script.response = okBody("[]");
    script.beforeReturn = [](std::atomic<bool>* flag) {
        if (flag) {
            flag->store(true);
        }
    };
    fake.enqueue(std::move(script));

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    std::atomic<bool> cancel{false};
    const GeocodeResult r = svc.searchPlaces("Lodz", false, &cancel);
    expect(r.cancelled || !r.ok, "dl_geo_cancel");
}

void testGeocodeCacheHit() {
    resetTestCache();
    FakeHttpTransport fake;
    const string nominatimJson =
        R"([{"display_name":"Test Place","lat":"52.23","lon":"21.01","osm_type":"node","osm_id":1}])";
    fake.enqueue(FakeHttpScript{"nominatim", okBody(nominatimJson)});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const GeocodeResult first = svc.searchPlaces("Test Place");
    expect(first.ok, "dl_geo_cache_first_ok");
    expect(!first.fromCache, "dl_geo_cache_first_net");
    expect(first.hits.size() == 1, "dl_geo_cache_hit_count");
    expect(fake.callCount() == 1, "dl_geo_cache_first_calls");

    const GeocodeResult second = svc.searchPlaces("Test Place");
    expect(second.ok, "dl_geo_cache_second_ok");
    expect(second.fromCache, "dl_geo_cache_second_hit");
    expect(fake.callCount() == 1, "dl_geo_cache_no_refetch");

    // Force bypasses cache.
    fake.enqueue(FakeHttpScript{"nominatim", okBody(nominatimJson)});
    const GeocodeResult forced = svc.searchPlaces("Test Place", true);
    expect(forced.ok && !forced.fromCache, "dl_geo_force_refresh");
    expect(fake.callCount() == 2, "dl_geo_force_calls");
}

void testOverpassSizeLimitReject() {
    resetTestCache();
    FakeHttpTransport fake;
    // Script returns a huge body; FakeHttpTransport enforces maxResponseBytes without truncating for parse.
    string huge(kDefaultMaxOverpassBytes + 100, 'x');
    // Prefix with fake JSON so if truncation happened wrongly, parse might still be attempted.
    huge.replace(0, 20, "{\"elements\":[1]}xxxxx");
    fake.enqueue(FakeHttpScript{"overpass", okBody(huge)});

    MapDownloadLimits limits;
    limits.maxOverpassBytes = 1024; // tiny for test
    MapDownloadService svc(fake, DownloadCache(testCacheRoot()), {}, limits);

    const MapFetchResult r = svc.downloadRoads(52.23, 21.01, 500.0);
    expect(!r.ok, "dl_size_reject_fail");
    expect(r.sizeExceeded, "dl_size_reject_flag");
    expect(r.error.find("mniejsz") != string::npos || r.error.find("limit") != string::npos
           || r.error.find("Limit") != string::npos || r.error.find("size") != string::npos,
           "dl_size_reject_msg");
}

void testOverpassCacheAndImportKeepsOldOnFail() {
    resetTestCache();
    FakeHttpTransport fake;

    // Valid Overpass fixture for download path.
    const string good = readFixture("shared_junction.overpass.json");
    expect(!good.empty(), "dl_fixture_shared_loaded");
    fake.enqueue(FakeHttpScript{"overpass", okBody(good)});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const MapFetchResult fetched = svc.downloadRoads(52.23, 21.01, 250.0);
    expect(fetched.ok, "dl_fetch_ok");
    expect(!fetched.cachePath.empty(), "dl_fetch_path");
    expect(fs::exists(fs::u8path(fetched.cachePath)), "dl_fetch_file_exists");

    // Second call hits cache — no new HTTP.
    const MapFetchResult cached = svc.downloadRoads(52.23, 21.01, 250.0);
    expect(cached.ok && cached.fromCache, "dl_fetch_cache_hit");
    expect(fake.callCount() == 1, "dl_fetch_cache_no_http");

    SimulationSession session;
    session.getSimulation().setEventLogOutputFile("tests/output/dl_keep_old_events.json");
    session.getSimulation().setStatisticsOutputFiles(
        "tests/output/dl_keep_old_timeline.csv",
        "tests/output/dl_keep_old_summary.csv"
    );
    session.loadDemoScenario();
    const string demoName = session.getCity().getName();
    const size_t demoRoads = session.getCity().getRoadSegments().size();

    // Successful import from cached download.
    const OsmScenarioResult okImport = session.loadOsmFromFile(fetched.cachePath);
    expect(okImport.ok, "dl_import_downloaded_ok");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "dl_import_mode_osm");

    // Bad download body written then import fails → keep previous OSM... use invalid path import.
    const string badPath = testCacheRoot() + "/overpass/bad_import.json";
    {
        ofstream out(fs::u8path(badPath), ios::binary);
        out << "not-json-at-all";
    }
    const size_t roadsBefore = session.getCity().getRoadSegments().size();
    const OsmScenarioResult badImport = session.loadOsmFromFile(badPath);
    expect(!badImport.ok, "dl_import_bad_fail");
    expect(session.getLoadedScenario() == SessionScenario::Osm, "dl_import_keep_mode");
    expect(session.getCity().getRoadSegments().size() == roadsBefore, "dl_import_keep_roads");

    // Also: demo kept when first import after demo fails.
    session.loadDemoScenario();
    const OsmScenarioResult failDemo = session.loadOsmFromFile(badPath);
    expect(!failDemo.ok, "dl_import_fail_on_demo");
    expect(session.getLoadedScenario() == SessionScenario::Demo, "dl_import_keep_demo_mode");
    expect(session.getCity().getName() == demoName, "dl_import_keep_demo_name");
    expect(session.getCity().getRoadSegments().size() == demoRoads, "dl_import_keep_demo_roads");
}

void testOverpassHttpErrorAndBadJson() {
    resetTestCache();
    FakeHttpTransport fake;
    HttpResponse err;
    err.statusCode = 504;
    err.error = "HTTP status 504";
    fake.enqueue(FakeHttpScript{"overpass", err});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const MapFetchResult r = svc.downloadRoads(52.0, 21.0, 500.0);
    expect(!r.ok, "dl_overpass_http_fail");

    fake.enqueue(FakeHttpScript{"overpass", okBody("{\"oops\":true}")});
    const MapFetchResult bad = svc.downloadRoads(52.1, 21.1, 500.0);
    expect(!bad.ok, "dl_overpass_bad_json");
}

void testDownloadCancel() {
    resetTestCache();
    FakeHttpTransport fake;
    FakeHttpScript script;
    script.urlContains = "overpass";
    script.response = okBody(
        "{\"elements\":[{\"type\":\"node\",\"id\":1,\"lat\":1,\"lon\":1}]}"
    );
    script.beforeReturn = [](std::atomic<bool>* flag) {
        if (flag) {
            flag->store(true);
        }
    };
    fake.enqueue(std::move(script));

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    std::atomic<bool> cancel{false};
    const MapFetchResult r = svc.downloadRoads(52.0, 21.0, 500.0, false, &cancel);
    expect(r.cancelled || !r.ok, "dl_overpass_cancel");
}

} // namespace

int mapDownloadPassedCount() {
    return g_passed;
}

int runMapDownloadTests() {
    cout << "\n--- Map download / geocode tests (Stage 3 download, offline) ---\n";
    g_passed = 0;
    g_failures = 0;

    fs::create_directories("tests/output");
    resetTestCache();

    testBboxFromRadius();
    testRadiusLimits();
    testOverpassQueryContainsRestrictions();
    testGeocodeTimeout();
    testGeocodeHttpError();
    testGeocodeBadJson();
    testGeocodeCancel();
    testGeocodeCacheHit();
    testOverpassSizeLimitReject();
    testOverpassHttpErrorAndBadJson();
    testDownloadCancel();
    testOverpassCacheAndImportKeepsOldOnFail();

    return g_failures;
}
