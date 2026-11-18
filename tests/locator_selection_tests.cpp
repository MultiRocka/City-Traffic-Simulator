#include "gui/LocatorMap.h"
#include "osm/DownloadCache.h"
#include "osm/FakeHttpTransport.h"
#include "osm/GeoBbox.h"
#include "osm/MapDownloadService.h"
#include "osm/OverpassQuery.h"

#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>

using namespace std;
using namespace gui;
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

bool near(double a, double b, double eps = 1e-6) {
    return std::fabs(a - b) <= eps;
}

string testCacheRoot() {
    return "tests/output/cache_etap4_sel";
}

void resetTestCache() {
    std::error_code ec;
    fs::remove_all(fs::u8path(testCacheRoot()), ec);
    fs::create_directories(fs::u8path(testCacheRoot() + "/geocode"), ec);
    fs::create_directories(fs::u8path(testCacheRoot() + "/overpass"), ec);
}

void testScreenGeoRoundTrip() {
    LocatorCamera camera;
    camera.viewportWidth = 400;
    camera.viewportHeight = 300;
    camera.centerLat = 52.23;
    camera.centerLon = 21.01;
    camera.metersPerPixel = 5.0;

    const LocatorScreenCoord mid = geoToScreen(camera, 52.23, 21.01);
    expect(near(mid.x, 200.0), "loc_center_screen_x");
    expect(near(mid.y, 150.0), "loc_center_screen_y");

    const GeoCoord back = screenToGeo(camera, mid.x, mid.y);
    expect(near(back.lat, 52.23, 1e-9), "loc_roundtrip_lat");
    expect(near(back.lon, 21.01, 1e-9), "loc_roundtrip_lon");

    // Move 100 m north → smaller screen Y.
    const double dLat = 100.0 / kMetersPerDegLat;
    const LocatorScreenCoord north = geoToScreen(camera, 52.23 + dLat, 21.01);
    expect(near(north.x, 200.0, 1e-3), "loc_north_x");
    expect(north.y < mid.y, "loc_north_up");
}

void testZoomKeepsGeoAnchor() {
    LocatorCamera camera;
    camera.viewportWidth = 500;
    camera.viewportHeight = 400;
    camera.centerLat = 50.0;
    camera.centerLon = 19.0;
    camera.metersPerPixel = 10.0;

    const double sx = 120.0;
    const double sy = 80.0;
    const GeoCoord before = screenToGeo(camera, sx, sy);
    camera = zoomLocatorAtScreen(camera, sx, sy, 2.0);
    const GeoCoord after = screenToGeo(camera, sx, sy);
    expect(near(camera.metersPerPixel, 5.0), "loc_zoom_scale");
    expect(near(before.lat, after.lat, 1e-9), "loc_zoom_anchor_lat");
    expect(near(before.lon, after.lon, 1e-9), "loc_zoom_anchor_lon");
}

void testPinSizeBuildsRectangularBbox() {
    MapAreaSelection state;
    state.mode = LocatorGestureMode::PinAndSize;
    setPin(state, 52.23, 21.01);
    setSizeRadius(state, 500.0);

    expect(state.hasSelection, "sel_pin_has_bbox");
    expect(state.selection.isValid(), "sel_pin_bbox_valid");

    const GeoBbox expected = bboxFromRadiusMeters(52.23, 21.01, 500.0);
    expect(near(state.selection.south, expected.south), "sel_pin_south");
    expect(near(state.selection.north, expected.north), "sel_pin_north");
    expect(near(state.selection.west, expected.west), "sel_pin_west");
    expect(near(state.selection.east, expected.east), "sel_pin_east");

    // Honest rectangle — not a circle claim: sides are axis-aligned.
    expect(state.selection.north > state.selection.south, "sel_pin_rect_ns");
    expect(state.selection.east > state.selection.west, "sel_pin_rect_ew");
}

void testRectangleDrawMode() {
    LocatorCamera camera;
    camera.viewportWidth = 400;
    camera.viewportHeight = 400;
    camera.centerLat = 52.0;
    camera.centerLon = 21.0;
    camera.metersPerPixel = 10.0;

    MapAreaSelection state;
    state.mode = LocatorGestureMode::DrawRectangle;

    // Drag from center toward bottom-right.
    expect(locatorMouseDown(camera, state, 200.0, 200.0), "sel_rect_down");
    expect(state.drawingRect, "sel_rect_drawing");
    expect(locatorMouseMove(camera, state, 280.0, 260.0), "sel_rect_move");
    expect(locatorMouseUp(state), "sel_rect_up");
    expect(state.hasSelection && state.selection.isValid(), "sel_rect_done");
    expect(!state.drawingRect, "sel_rect_not_drawing");

    const SelectionMetrics m = measureSelection(state.selection);
    expect(m.widthMeters > 0.0 && m.heightMeters > 0.0, "sel_rect_dims");
    expect(m.withinLimits, "sel_rect_within_limits");
}

void testGestureModesDoNotCollide() {
    LocatorCamera camera;
    camera.viewportWidth = 300;
    camera.viewportHeight = 300;
    camera.centerLat = 52.0;
    camera.centerLon = 21.0;
    camera.metersPerPixel = 8.0;

    MapAreaSelection state;
    state.mode = LocatorGestureMode::Pan;
    const double latBefore = camera.centerLat;
    locatorMouseDown(camera, state, 100.0, 100.0);
    locatorMouseMove(camera, state, 140.0, 100.0);
    locatorMouseUp(state);
    expect(camera.centerLon < 21.0, "sel_pan_moves");
    expect(!state.hasSelection, "sel_pan_no_bbox");

    // Switch to pin: click places pin without requiring prior pan state.
    clearSelection(state);
    camera.centerLat = latBefore;
    camera.centerLon = 21.0;
    state.mode = LocatorGestureMode::PinAndSize;
    setSizeRadius(state, 250.0);
    locatorMouseDown(camera, state, 150.0, 150.0);
    locatorMouseUp(state);
    expect(state.hasPin && state.hasSelection, "sel_pin_after_pan_mode");

    // Rectangle mode replaces selection rather than panning.
    const double lonPinned = camera.centerLon;
    state.mode = LocatorGestureMode::DrawRectangle;
    locatorMouseDown(camera, state, 100.0, 100.0);
    locatorMouseMove(camera, state, 200.0, 200.0);
    locatorMouseUp(state);
    expect(near(camera.centerLon, lonPinned), "sel_rect_no_pan");
    expect(state.hasSelection, "sel_rect_has_bbox");
}

void testSearchCentersPinWithoutDownload() {
    LocatorCamera camera;
    MapAreaSelection state;
    state.mode = LocatorGestureMode::PinAndSize;
    setSizeRadius(state, 500.0);

    applySearchHit(camera, state, 50.06, 19.94);
    expect(near(camera.centerLat, 50.06), "sel_search_center_lat");
    expect(near(camera.centerLon, 19.94), "sel_search_center_lon");
    expect(state.hasPin, "sel_search_pin");
    expect(state.hasSelection, "sel_search_bbox");
    // applySearchHit must not invoke any HTTP — verified by absence of transport use here.
}

void testClearSelection() {
    MapAreaSelection state;
    setPin(state, 52.0, 21.0);
    setSizeRadius(state, 500.0);
    expect(state.hasSelection, "sel_clear_pre");
    clearSelection(state);
    expect(!state.hasPin && !state.hasSelection, "sel_clear_post");
}

void testBboxQueryCacheConsistency() {
    resetTestCache();
    const GeoBbox box = bboxFromRadiusMeters(52.2258, 21.0174, 400.0);
    expect(box.isValid(), "sel_consist_box");

    const string key = box.toKey(5);
    const string overpassFrag = box.toOverpassBBox();
    expect(key == overpassFrag, "sel_consist_key_eq_overpass");

    const string query = buildRoadNetworkQuery(box);
    expect(query.find(overpassFrag) != string::npos, "sel_consist_query_has_bbox");

    DownloadCache cache(testCacheRoot());
    const string path = cache.overpassPath(key);
    expect(path.find(cache.hashKey("v1|" + key)) != string::npos
           || path.find("/overpass/") != string::npos, "sel_consist_cache_path");

    // Fake download: recorded POST body must contain the same bbox fragment.
    FakeHttpTransport fake;
    const string body =
        "{\"elements\":[{\"type\":\"node\",\"id\":1,\"lat\":52.22,\"lon\":21.01},"
        "{\"type\":\"way\",\"id\":2,\"nodes\":[1],\"tags\":{\"highway\":\"residential\"}}]}";
    // looksLikeOverpassJson requires non-empty elements — fixture above is enough.
    HttpResponse ok;
    ok.ok = true;
    ok.statusCode = 200;
    ok.body = body;
    // Enrich body to pass looksLikeOverpassJson (needs elements array — already).
    fake.enqueue(FakeHttpScript{"overpass", ok});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const MapFetchResult fetched = svc.downloadRoadsForBbox(box);
    expect(fetched.ok, "sel_consist_fetch_ok");
    expect(fetched.bbox.toKey(5) == key, "sel_consist_result_bbox_key");
    expect(fetched.cachePath == path, "sel_consist_result_cache_path");

    expect(fake.callCount() == 1, "sel_consist_one_http");
    expect(!fake.recordedRequests().empty(), "sel_consist_recorded");
    const string& reqBody = fake.recordedRequests().front().body;
    // POST body is form-urlencoded ("data=" + urlEncode(query)); commas → %2C.
    string encodedBbox;
    encodedBbox.reserve(overpassFrag.size() * 3);
    for (unsigned char ch : overpassFrag) {
        if ((ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9')
            || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
            encodedBbox.push_back(static_cast<char>(ch));
        } else if (ch == ' ') {
            encodedBbox.push_back('+');
        } else {
            static const char* hex = "0123456789ABCDEF";
            encodedBbox.push_back('%');
            encodedBbox.push_back(hex[(ch >> 4) & 0xF]);
            encodedBbox.push_back(hex[ch & 0xF]);
        }
    }
    expect(reqBody.find(encodedBbox) != string::npos
           || reqBody.find(overpassFrag) != string::npos, "sel_consist_post_has_bbox");

    // Second call: cache hit uses same key path, no new HTTP.
    const MapFetchResult cached = svc.downloadRoadsForBbox(box);
    expect(cached.ok && cached.fromCache, "sel_consist_cache_hit");
    expect(cached.cachePath == path, "sel_consist_cache_same_path");
    expect(fake.callCount() == 1, "sel_consist_no_refetch");
}

void testPinSizeMatchesDownloadRoadsPath() {
    resetTestCache();
    MapAreaSelection state;
    state.mode = LocatorGestureMode::PinAndSize;
    setPin(state, 52.23, 21.01);
    setSizeRadius(state, 250.0);

    const GeoBbox fromPin = state.selection;
    const GeoBbox fromRadius = bboxFromRadiusMeters(52.23, 21.01, 250.0);
    expect(fromPin.toKey(5) == fromRadius.toKey(5), "sel_pin_eq_radius_key");

    FakeHttpTransport fake;
    HttpResponse ok;
    ok.ok = true;
    ok.statusCode = 200;
    ok.body = "{\"elements\":[{\"type\":\"node\",\"id\":1,\"lat\":1,\"lon\":1}]}";
    fake.enqueue(FakeHttpScript{"overpass", ok});
    fake.enqueue(FakeHttpScript{"overpass", ok});

    MapDownloadService svc(fake, DownloadCache(testCacheRoot()));
    const MapFetchResult a = svc.downloadRoadsForBbox(fromPin);
    // Force second path through legacy radius API with empty cache for a different box —
    // same box should hit cache from first call if we don't force.
    const MapFetchResult b = svc.downloadRoads(52.23, 21.01, 250.0);
    expect(a.ok && b.ok, "sel_pin_radius_both_ok");
    expect(b.fromCache, "sel_pin_radius_cache_shared");
    expect(a.cachePath == b.cachePath, "sel_pin_radius_same_cache");
    expect(a.bbox.toKey(5) == b.bbox.toKey(5), "sel_pin_radius_same_bbox");
}

void testBboxAllowedLimits() {
    string err;
    const GeoBbox ok = bboxFromRadiusMeters(52.0, 21.0, 500.0);
    expect(bboxAllowed(ok, err), "sel_limits_ok");

    // Huge rectangle (~10 km side).
    GeoBbox huge;
    huge.south = 52.0;
    huge.north = 52.1; // ~11 km
    huge.west = 21.0;
    huge.east = 21.15;
    expect(!bboxAllowed(huge, err), "sel_limits_reject_huge");
    expect(!err.empty(), "sel_limits_error");
}

void testMetricsWidthHeightArea() {
    const GeoBbox box = bboxFromRadiusMeters(0.0, 0.0, 500.0);
    const SelectionMetrics m = measureSelection(box);
    expect(near(m.widthMeters, 1000.0, 2.0), "sel_metrics_width");
    expect(near(m.heightMeters, 1000.0, 2.0), "sel_metrics_height");
    expect(m.areaKm2 > 0.9 && m.areaKm2 < 1.1, "sel_metrics_area");
    expect(m.withinLimits, "sel_metrics_ok");
}

} // namespace

int locatorSelectionPassedCount() {
    return g_passed;
}

int runLocatorSelectionTests() {
    cout << "\n--- Locator / map area selection tests (Etap 4, offline) ---\n";
    g_passed = 0;
    g_failures = 0;

    fs::create_directories("tests/output");
    resetTestCache();

    testScreenGeoRoundTrip();
    testZoomKeepsGeoAnchor();
    testPinSizeBuildsRectangularBbox();
    testRectangleDrawMode();
    testGestureModesDoNotCollide();
    testSearchCentersPinWithoutDownload();
    testClearSelection();
    testBboxQueryCacheConsistency();
    testPinSizeMatchesDownloadRoadsPath();
    testBboxAllowedLimits();
    testMetricsWidthHeightArea();

    return g_failures;
}
