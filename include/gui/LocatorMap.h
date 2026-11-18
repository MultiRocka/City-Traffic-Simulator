#pragma once

#include "osm/GeoBbox.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace gui {

// Geographic locator camera for the download-area picker (lat/lon degrees).
// View-only: never mutates Overpass data or the live simulation map.
struct LocatorCamera {
    double centerLat = 52.2297;   // Warsaw default
    double centerLon = 21.0122;
    double metersPerPixel = 8.0;  // scale at center latitude
    int viewportWidth = 1;
    int viewportHeight = 1;
};

struct GeoCoord {
    double lat = 0.0;
    double lon = 0.0;
};

struct LocatorScreenCoord {
    double x = 0.0;
    double y = 0.0;
};

inline constexpr double kLocatorMinMetersPerPixel = 0.5;
inline constexpr double kLocatorMaxMetersPerPixel = 200.0;
inline constexpr double kLocatorDefaultZoomFactor = 1.2;
inline constexpr double kMetersPerDegLat = 111320.0;
inline constexpr double kPinHitRadiusPx = 12.0;

inline double clampLocatorMetersPerPixel(double metersPerPixel) {
    if (!std::isfinite(metersPerPixel) || metersPerPixel <= 0.0) {
        return 8.0;
    }
    return std::clamp(metersPerPixel, kLocatorMinMetersPerPixel, kLocatorMaxMetersPerPixel);
}

inline LocatorCamera clampLocatorCamera(LocatorCamera camera) {
    camera.metersPerPixel = clampLocatorMetersPerPixel(camera.metersPerPixel);
    camera.viewportWidth = std::max(1, camera.viewportWidth);
    camera.viewportHeight = std::max(1, camera.viewportHeight);
    if (!std::isfinite(camera.centerLat)) {
        camera.centerLat = 0.0;
    }
    if (!std::isfinite(camera.centerLon)) {
        camera.centerLon = 0.0;
    }
    camera.centerLat = std::clamp(camera.centerLat, -85.0, 85.0);
    camera.centerLon = std::clamp(camera.centerLon, -180.0, 180.0);
    return camera;
}

inline double metersPerDegLonAt(double latDeg) {
    const double latRad = latDeg * 3.14159265358979323846 / 180.0;
    const double cosLat = std::cos(latRad);
    return kMetersPerDegLat * std::max(0.01, std::abs(cosLat));
}

inline LocatorScreenCoord geoToScreen(const LocatorCamera& cameraIn, double latDeg, double lonDeg) {
    const LocatorCamera camera = clampLocatorCamera(cameraIn);
    const double mppLon = metersPerDegLonAt(camera.centerLat);
    const double dxM = (lonDeg - camera.centerLon) * mppLon;
    const double dyM = (latDeg - camera.centerLat) * kMetersPerDegLat;
    const double x = static_cast<double>(camera.viewportWidth) * 0.5 + dxM / camera.metersPerPixel;
    // North is up → smaller screen Y.
    const double y = static_cast<double>(camera.viewportHeight) * 0.5 - dyM / camera.metersPerPixel;
    return {x, y};
}

inline GeoCoord screenToGeo(const LocatorCamera& cameraIn, double screenX, double screenY) {
    const LocatorCamera camera = clampLocatorCamera(cameraIn);
    const double mppLon = metersPerDegLonAt(camera.centerLat);
    const double dxM = (screenX - static_cast<double>(camera.viewportWidth) * 0.5) * camera.metersPerPixel;
    const double dyM = (static_cast<double>(camera.viewportHeight) * 0.5 - screenY) * camera.metersPerPixel;
    return {
        camera.centerLat + dyM / kMetersPerDegLat,
        camera.centerLon + dxM / mppLon
    };
}

inline LocatorCamera setLocatorViewport(LocatorCamera camera, int width, int height) {
    camera.viewportWidth = std::max(1, width);
    camera.viewportHeight = std::max(1, height);
    return clampLocatorCamera(camera);
}

inline LocatorCamera zoomLocatorAtScreen(
    LocatorCamera camera,
    double screenX,
    double screenY,
    double zoomFactor
) {
    camera = clampLocatorCamera(camera);
    if (!std::isfinite(zoomFactor) || zoomFactor <= 0.0) {
        return camera;
    }
    const GeoCoord anchor = screenToGeo(camera, screenX, screenY);
    camera.metersPerPixel = clampLocatorMetersPerPixel(camera.metersPerPixel / zoomFactor);

    // Lat first (independent of lon scale), then lon with updated centerLat so
    // meters-per-degree-lon matches the post-zoom center.
    GeoCoord after = screenToGeo(camera, screenX, screenY);
    camera.centerLat += anchor.lat - after.lat;
    camera = clampLocatorCamera(camera);

    after = screenToGeo(camera, screenX, screenY);
    camera.centerLon += anchor.lon - after.lon;
    return clampLocatorCamera(camera);
}

// Positive dx/dy move content with the pointer (drag-to-pan).
inline LocatorCamera panLocatorByScreenDelta(LocatorCamera camera, double dxPixels, double dyPixels) {
    camera = clampLocatorCamera(camera);
    const double mppLon = metersPerDegLonAt(camera.centerLat);
    camera.centerLon -= (dxPixels * camera.metersPerPixel) / mppLon;
    camera.centerLat += (dyPixels * camera.metersPerPixel) / kMetersPerDegLat;
    return clampLocatorCamera(camera);
}

inline LocatorCamera centerLocatorOn(LocatorCamera camera, double latDeg, double lonDeg) {
    camera.centerLat = latDeg;
    camera.centerLon = lonDeg;
    return clampLocatorCamera(camera);
}

// --- Selection modes (gesture state machine; no Win32 dependency) ---

enum class LocatorGestureMode {
    Pan = 0,
    PinAndSize = 1,
    DrawRectangle = 2
};

struct MapAreaSelection {
    LocatorGestureMode mode = LocatorGestureMode::PinAndSize;

    bool hasPin = false;
    double pinLat = 0.0;
    double pinLon = 0.0;
    double sizeRadiusMeters = 500.0; // pin+size → rectangular bbox half-side

    bool hasSelection = false;
    osm_net::GeoBbox selection; // authoritative download rectangle

    // Live rectangle drag (DrawRectangle mode).
    bool drawingRect = false;
    double rectAnchorLat = 0.0;
    double rectAnchorLon = 0.0;
    double rectCurrentLat = 0.0;
    double rectCurrentLon = 0.0;

    // Pan / pin-drag interaction flags.
    bool panning = false;
    bool draggingPin = false;
    double lastScreenX = 0.0;
    double lastScreenY = 0.0;
};

inline osm_net::GeoBbox bboxFromCorners(double latA, double lonA, double latB, double lonB) {
    osm_net::GeoBbox box;
    box.south = std::min(latA, latB);
    box.north = std::max(latA, latB);
    box.west = std::min(lonA, lonB);
    box.east = std::max(lonA, lonB);
    return box;
}

inline void applyPinSizeToSelection(MapAreaSelection& state) {
    if (!state.hasPin) {
        state.hasSelection = false;
        state.selection = {};
        return;
    }
    state.selection = osm_net::bboxFromRadiusMeters(state.pinLat, state.pinLon, state.sizeRadiusMeters);
    state.hasSelection = state.selection.isValid();
}

inline void clearSelection(MapAreaSelection& state) {
    state.hasPin = false;
    state.hasSelection = false;
    state.selection = {};
    state.drawingRect = false;
    state.draggingPin = false;
    state.panning = false;
}

inline void setPin(MapAreaSelection& state, double latDeg, double lonDeg) {
    state.hasPin = true;
    state.pinLat = latDeg;
    state.pinLon = lonDeg;
    if (state.mode == LocatorGestureMode::PinAndSize) {
        applyPinSizeToSelection(state);
    }
}

inline void setSizeRadius(MapAreaSelection& state, double radiusMeters) {
    state.sizeRadiusMeters = radiusMeters;
    if (state.mode == LocatorGestureMode::PinAndSize && state.hasPin) {
        applyPinSizeToSelection(state);
    }
}

inline bool hitTestPin(const LocatorCamera& camera, const MapAreaSelection& state, double screenX, double screenY) {
    if (!state.hasPin) {
        return false;
    }
    const LocatorScreenCoord pin = geoToScreen(camera, state.pinLat, state.pinLon);
    const double dx = screenX - pin.x;
    const double dy = screenY - pin.y;
    return (dx * dx + dy * dy) <= (kPinHitRadiusPx * kPinHitRadiusPx);
}

// Mouse down — returns true if the event was consumed by the selection tool.
inline bool locatorMouseDown(
    LocatorCamera& camera,
    MapAreaSelection& state,
    double screenX,
    double screenY
) {
    state.lastScreenX = screenX;
    state.lastScreenY = screenY;

    if (state.mode == LocatorGestureMode::Pan) {
        state.panning = true;
        state.draggingPin = false;
        state.drawingRect = false;
        return true;
    }

    if (state.mode == LocatorGestureMode::PinAndSize) {
        if (hitTestPin(camera, state, screenX, screenY)) {
            state.draggingPin = true;
            state.panning = false;
            return true;
        }
        const GeoCoord geo = screenToGeo(camera, screenX, screenY);
        setPin(state, geo.lat, geo.lon);
        state.draggingPin = true;
        state.panning = false;
        return true;
    }

    // DrawRectangle: click-drag defines corners. Does not auto-download.
    const GeoCoord geo = screenToGeo(camera, screenX, screenY);
    state.drawingRect = true;
    state.rectAnchorLat = geo.lat;
    state.rectAnchorLon = geo.lon;
    state.rectCurrentLat = geo.lat;
    state.rectCurrentLon = geo.lon;
    state.hasSelection = false;
    state.selection = {};
    state.panning = false;
    state.draggingPin = false;
    return true;
}

inline bool locatorMouseMove(
    LocatorCamera& camera,
    MapAreaSelection& state,
    double screenX,
    double screenY
) {
    const double dx = screenX - state.lastScreenX;
    const double dy = screenY - state.lastScreenY;
    state.lastScreenX = screenX;
    state.lastScreenY = screenY;

    if (state.panning && state.mode == LocatorGestureMode::Pan) {
        camera = panLocatorByScreenDelta(camera, dx, dy);
        return true;
    }

    if (state.draggingPin && state.mode == LocatorGestureMode::PinAndSize) {
        const GeoCoord geo = screenToGeo(camera, screenX, screenY);
        setPin(state, geo.lat, geo.lon);
        return true;
    }

    if (state.drawingRect && state.mode == LocatorGestureMode::DrawRectangle) {
        const GeoCoord geo = screenToGeo(camera, screenX, screenY);
        state.rectCurrentLat = geo.lat;
        state.rectCurrentLon = geo.lon;
        state.selection = bboxFromCorners(
            state.rectAnchorLat, state.rectAnchorLon,
            state.rectCurrentLat, state.rectCurrentLon
        );
        state.hasSelection = state.selection.isValid();
        // Keep pin at rectangle center for label / search continuity.
        if (state.hasSelection) {
            state.hasPin = true;
            state.pinLat = 0.5 * (state.selection.south + state.selection.north);
            state.pinLon = 0.5 * (state.selection.west + state.selection.east);
        }
        return true;
    }

    return false;
}

inline bool locatorMouseUp(MapAreaSelection& state) {
    const bool consumed = state.panning || state.draggingPin || state.drawingRect;
    state.panning = false;
    state.draggingPin = false;
    if (state.drawingRect) {
        state.drawingRect = false;
        state.selection = bboxFromCorners(
            state.rectAnchorLat, state.rectAnchorLon,
            state.rectCurrentLat, state.rectCurrentLon
        );
        state.hasSelection = state.selection.isValid();
        if (state.hasSelection) {
            state.hasPin = true;
            state.pinLat = 0.5 * (state.selection.south + state.selection.north);
            state.pinLon = 0.5 * (state.selection.west + state.selection.east);
        }
    }
    return consumed;
}

// Wheel zoom is always available (does not collide with mode tools).
inline void locatorWheelZoom(
    LocatorCamera& camera,
    double screenX,
    double screenY,
    bool zoomIn
) {
    camera = zoomLocatorAtScreen(
        camera,
        screenX,
        screenY,
        zoomIn ? kLocatorDefaultZoomFactor : (1.0 / kLocatorDefaultZoomFactor)
    );
}

struct SelectionMetrics {
    double widthMeters = 0.0;
    double heightMeters = 0.0;
    double areaKm2 = 0.0;
    bool withinLimits = false;
    std::string limitError;
};

inline SelectionMetrics measureSelection(const osm_net::GeoBbox& box) {
    SelectionMetrics m;
    if (!box.isValid()) {
        m.limitError = "Brak poprawnego prostokąta wyboru.";
        return m;
    }
    const auto dims = osm_net::bboxDimensionsMeters(box);
    m.widthMeters = dims.widthMeters;
    m.heightMeters = dims.heightMeters;
    m.areaKm2 = box.areaKm2();
    m.withinLimits = osm_net::bboxAllowed(box, m.limitError);
    return m;
}

// Place-search result: center map + set pin; never starts a download.
inline void applySearchHit(
    LocatorCamera& camera,
    MapAreaSelection& state,
    double latDeg,
    double lonDeg
) {
    camera = centerLocatorOn(camera, latDeg, lonDeg);
    setPin(state, latDeg, lonDeg);
    if (state.mode == LocatorGestureMode::DrawRectangle) {
        // Search in rectangle mode still drops a pin+default size so the user
        // sees a concrete rectangle; they can redraw afterward.
        state.mode = LocatorGestureMode::PinAndSize;
        applyPinSizeToSelection(state);
    }
}

} // namespace gui
