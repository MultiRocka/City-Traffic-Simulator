#include "gui/MapCamera.h"

#include <cmath>
#include <iostream>
#include <string>

using namespace std;
using namespace gui;

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

void testClampMetersPerPixel() {
    expect(near(clampMetersPerPixel(0.0), 1.0), "cam_clamp_zero");
    expect(near(clampMetersPerPixel(-5.0), 1.0), "cam_clamp_negative");
    expect(near(clampMetersPerPixel(kMinMetersPerPixel * 0.1), kMinMetersPerPixel), "cam_clamp_min");
    expect(near(clampMetersPerPixel(kMaxMetersPerPixel * 10.0), kMaxMetersPerPixel), "cam_clamp_max");
    expect(near(clampMetersPerPixel(1.25), 1.25), "cam_clamp_passthrough");
}

void testWorldScreenRoundTrip() {
    MapCameraState camera;
    camera.viewportWidth = 800;
    camera.viewportHeight = 600;
    camera.centerX = 120.0;
    camera.centerY = -40.0;
    camera.metersPerPixel = 0.5;

    const ScreenCoord screen = worldToScreen(camera, 120.0, -40.0);
    expect(near(screen.x, 400.0), "cam_center_screen_x");
    expect(near(screen.y, 300.0), "cam_center_screen_y");

    const WorldCoord back = screenToWorld(camera, screen.x, screen.y);
    expect(near(back.x, 120.0), "cam_roundtrip_x");
    expect(near(back.y, -40.0), "cam_roundtrip_y");

    const ScreenCoord offset = worldToScreen(camera, 220.0, 60.0);
    // +100 m east → +200 px; +100 m north → -200 px (Y flipped).
    expect(near(offset.x, 600.0), "cam_offset_screen_x");
    expect(near(offset.y, 100.0), "cam_offset_screen_y");
}

void testZoomKeepsAnchor() {
    MapCameraState camera;
    camera.viewportWidth = 1000;
    camera.viewportHeight = 800;
    camera.centerX = 50.0;
    camera.centerY = 25.0;
    camera.metersPerPixel = 1.0;

    const double sx = 250.0;
    const double sy = 200.0;
    const WorldCoord before = screenToWorld(camera, sx, sy);
    camera = zoomAtScreenPoint(camera, sx, sy, 2.0);
    const WorldCoord after = screenToWorld(camera, sx, sy);

    expect(near(camera.metersPerPixel, 0.5), "cam_zoom_scale");
    expect(near(before.x, after.x, 1e-9), "cam_zoom_anchor_x");
    expect(near(before.y, after.y, 1e-9), "cam_zoom_anchor_y");
}

void testZoomBounds() {
    MapCameraState camera;
    camera.viewportWidth = 640;
    camera.viewportHeight = 480;
    camera.metersPerPixel = kMinMetersPerPixel;

    camera = zoomAtScreenPoint(camera, 320.0, 240.0, 8.0);
    expect(near(camera.metersPerPixel, kMinMetersPerPixel), "cam_zoom_min_bound");

    camera.metersPerPixel = kMaxMetersPerPixel;
    camera = zoomAtScreenPoint(camera, 320.0, 240.0, 0.125);
    expect(near(camera.metersPerPixel, kMaxMetersPerPixel), "cam_zoom_max_bound");
}

void testPanDoesNotChangeScale() {
    MapCameraState camera;
    camera.viewportWidth = 500;
    camera.viewportHeight = 400;
    camera.centerX = 0.0;
    camera.centerY = 0.0;
    camera.metersPerPixel = 2.0;

    camera = panByScreenDelta(camera, 50.0, -25.0);
    expect(near(camera.metersPerPixel, 2.0), "cam_pan_scale_unchanged");
    // Drag right → content moves right → center moves left in world X.
    expect(near(camera.centerX, -100.0), "cam_pan_center_x");
    // Drag up (negative dy in screen) → centerY decreases (world down).
    expect(near(camera.centerY, -50.0), "cam_pan_center_y");
}

void testFitToBounds() {
    WorldBounds bounds;
    bounds.minX = 0.0;
    bounds.maxX = 200.0;
    bounds.minY = 0.0;
    bounds.maxY = 100.0;

    const MapCameraState camera = fitToWorldBounds(bounds, 860, 660, 30);
    expect(near(camera.centerX, 100.0), "cam_fit_center_x");
    expect(near(camera.centerY, 50.0), "cam_fit_center_y");

    // Drawable 800x600; spanX/800=0.25, spanY/600≈0.166 → use max = 0.25.
    expect(near(camera.metersPerPixel, 0.25), "cam_fit_scale");

    const ScreenCoord minCorner = worldToScreen(camera, 0.0, 0.0);
    const ScreenCoord maxCorner = worldToScreen(camera, 200.0, 100.0);
    expect(minCorner.x < maxCorner.x, "cam_fit_x_order");
    expect(minCorner.y > maxCorner.y, "cam_fit_y_flipped");
}

} // namespace

int runMapCameraTests() {
    g_failures = 0;
    g_passed = 0;
    cout << "\n--- map camera ---\n";
    testClampMetersPerPixel();
    testWorldScreenRoundTrip();
    testZoomKeepsAnchor();
    testZoomBounds();
    testPanDoesNotChangeScale();
    testFitToBounds();
    return g_failures;
}

int mapCameraPassedCount() {
    return g_passed;
}
