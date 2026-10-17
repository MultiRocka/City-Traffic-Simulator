#pragma once

#include "ui/CityLayout.h"

#include <algorithm>
#include <cmath>

namespace gui {

// View-only camera. Never mutates core world / vehicle coordinates.
struct MapCameraState {
    double centerX = 0.0;       // world meters at viewport center
    double centerY = 0.0;
    double metersPerPixel = 1.0;
    int viewportWidth = 1;
    int viewportHeight = 1;
    int paddingPx = 30;
};

struct ScreenCoord {
    double x = 0.0;
    double y = 0.0;
};

struct WorldCoord {
    double x = 0.0;
    double y = 0.0;
};

inline constexpr double kMinMetersPerPixel = 0.05;
inline constexpr double kMaxMetersPerPixel = 25.0;
inline constexpr double kDefaultZoomFactor = 1.15;

inline double clampMetersPerPixel(double metersPerPixel) {
    if (!std::isfinite(metersPerPixel) || metersPerPixel <= 0.0) {
        return 1.0;
    }
    return std::clamp(metersPerPixel, kMinMetersPerPixel, kMaxMetersPerPixel);
}

inline MapCameraState clampCameraScale(MapCameraState camera) {
    camera.metersPerPixel = clampMetersPerPixel(camera.metersPerPixel);
    camera.viewportWidth = std::max(1, camera.viewportWidth);
    camera.viewportHeight = std::max(1, camera.viewportHeight);
    camera.paddingPx = std::max(0, camera.paddingPx);
    return camera;
}

inline int drawableWidth(const MapCameraState& camera) {
    return std::max(1, camera.viewportWidth - camera.paddingPx * 2);
}

inline int drawableHeight(const MapCameraState& camera) {
    return std::max(1, camera.viewportHeight - camera.paddingPx * 2);
}

// World +Y → screen up (matches prior MapView convention).
inline ScreenCoord worldToScreen(const MapCameraState& cameraIn, double worldX, double worldY) {
    const MapCameraState camera = clampCameraScale(cameraIn);
    const double screenX =
        static_cast<double>(camera.viewportWidth) * 0.5
        + (worldX - camera.centerX) / camera.metersPerPixel;
    const double screenY =
        static_cast<double>(camera.viewportHeight) * 0.5
        - (worldY - camera.centerY) / camera.metersPerPixel;
    return {screenX, screenY};
}

inline WorldCoord screenToWorld(const MapCameraState& cameraIn, double screenX, double screenY) {
    const MapCameraState camera = clampCameraScale(cameraIn);
    const double worldX =
        camera.centerX
        + (screenX - static_cast<double>(camera.viewportWidth) * 0.5) * camera.metersPerPixel;
    const double worldY =
        camera.centerY
        - (screenY - static_cast<double>(camera.viewportHeight) * 0.5) * camera.metersPerPixel;
    return {worldX, worldY};
}

inline MapCameraState setViewportSize(MapCameraState camera, int width, int height) {
    camera.viewportWidth = std::max(1, width);
    camera.viewportHeight = std::max(1, height);
    return clampCameraScale(camera);
}

// Keep the world point under (screenX, screenY) fixed while changing scale.
inline MapCameraState zoomAtScreenPoint(
    MapCameraState camera,
    double screenX,
    double screenY,
    double zoomFactor
) {
    camera = clampCameraScale(camera);
    if (!std::isfinite(zoomFactor) || zoomFactor <= 0.0) {
        return camera;
    }

    const WorldCoord anchor = screenToWorld(camera, screenX, screenY);
    camera.metersPerPixel = clampMetersPerPixel(camera.metersPerPixel / zoomFactor);

    const WorldCoord after = screenToWorld(camera, screenX, screenY);
    camera.centerX += anchor.x - after.x;
    camera.centerY += anchor.y - after.y;
    return clampCameraScale(camera);
}

// Positive dx/dy move the view content with the pointer (drag-to-pan).
inline MapCameraState panByScreenDelta(MapCameraState camera, double dxPixels, double dyPixels) {
    camera = clampCameraScale(camera);
    camera.centerX -= dxPixels * camera.metersPerPixel;
    camera.centerY += dyPixels * camera.metersPerPixel;
    return camera;
}

inline MapCameraState fitToWorldBounds(
    const WorldBounds& bounds,
    int viewportWidth,
    int viewportHeight,
    int paddingPx = 30
) {
    MapCameraState camera;
    camera.viewportWidth = std::max(1, viewportWidth);
    camera.viewportHeight = std::max(1, viewportHeight);
    camera.paddingPx = std::max(0, paddingPx);

    double spanX = bounds.maxX - bounds.minX;
    double spanY = bounds.maxY - bounds.minY;
    if (!(spanX > 0.0) || !std::isfinite(spanX)) {
        spanX = 1.0;
    }
    if (!(spanY > 0.0) || !std::isfinite(spanY)) {
        spanY = 1.0;
    }

    camera.centerX = 0.5 * (bounds.minX + bounds.maxX);
    camera.centerY = 0.5 * (bounds.minY + bounds.maxY);

    const double drawW = static_cast<double>(drawableWidth(camera));
    const double drawH = static_cast<double>(drawableHeight(camera));
    const double scaleX = spanX / drawW;
    const double scaleY = spanY / drawH;
    camera.metersPerPixel = clampMetersPerPixel(std::max(scaleX, scaleY));
    return camera;
}

} // namespace gui
