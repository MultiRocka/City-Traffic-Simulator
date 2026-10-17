#pragma once

#ifndef UNICODE
#define UNICODE
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include "gui/MapCamera.h"
#include "ui/RenderSnapshot.h"
#include "ui/CityLayout.h"

namespace gui {

struct Viewport {
    int x;
    int y;
    int width;
    int height;
};

class MapView {
public:
    MapView();
    ~MapView();

    void setViewport(const Viewport& viewport);
    void paint(HWND hwnd, HDC targetDeviceContext, const RenderSnapshot& snapshot);

    void fitToSnapshot(const RenderSnapshot& snapshot);
    void zoomAt(int screenX, int screenY, double zoomFactor);
    void beginPan(int screenX, int screenY);
    void updatePan(int screenX, int screenY);
    void endPan();
    bool isPanning() const;

    const MapCameraState& camera() const { return cameraState; }

private:
    Viewport viewport {0, 0, 800, 600};
    CityLayout layout;
    MapCameraState cameraState;
    bool hasFittedCamera;
    bool panning;
    int panLastX;
    int panLastY;

    HDC bufferDeviceContext;
    HBITMAP bufferBitmap;
    HBITMAP previousBitmap;
    int bufferWidth;
    int bufferHeight;

    void ensureBuffer(HDC targetDeviceContext, int width, int height);
    void releaseBuffer();
    void renderScene(HDC deviceContext, const RenderSnapshot& snapshot) const;
    void drawLegend(HDC deviceContext) const;
    void worldToScreenPixels(double worldX, double worldY, int& screenX, int& screenY) const;
};

} // namespace gui
