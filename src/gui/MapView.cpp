#include "gui/MapView.h"
#include "core/CollisionDetector.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace gui {

MapView::MapView()
    : hasFittedCamera(false),
      panning(false),
      panLastX(0),
      panLastY(0),
      bufferDeviceContext(nullptr),
      bufferBitmap(nullptr),
      previousBitmap(nullptr),
      bufferWidth(0),
      bufferHeight(0) {
}

MapView::~MapView() {
    releaseBuffer();
}

void MapView::setViewport(const Viewport& newViewport) {
    viewport = newViewport;
    cameraState = setViewportSize(cameraState, newViewport.width, newViewport.height);
}

void MapView::releaseBuffer() {
    if (bufferDeviceContext != nullptr && previousBitmap != nullptr) {
        SelectObject(bufferDeviceContext, previousBitmap);
        previousBitmap = nullptr;
    }

    if (bufferBitmap != nullptr) {
        DeleteObject(bufferBitmap);
        bufferBitmap = nullptr;
    }

    if (bufferDeviceContext != nullptr) {
        DeleteDC(bufferDeviceContext);
        bufferDeviceContext = nullptr;
    }

    bufferWidth = 0;
    bufferHeight = 0;
}

void MapView::ensureBuffer(HDC targetDeviceContext, int width, int height) {
    if (width <= 0 || height <= 0) {
        return;
    }

    if (bufferDeviceContext != nullptr && bufferWidth == width && bufferHeight == height) {
        return;
    }

    releaseBuffer();

    bufferDeviceContext = CreateCompatibleDC(targetDeviceContext);
    bufferBitmap = CreateCompatibleBitmap(targetDeviceContext, width, height);
    previousBitmap = static_cast<HBITMAP>(SelectObject(bufferDeviceContext, bufferBitmap));
    bufferWidth = width;
    bufferHeight = height;
}

void MapView::worldToScreenPixels(double worldX, double worldY, int& screenX, int& screenY) const {
    const ScreenCoord screen = worldToScreen(cameraState, worldX, worldY);
    screenX = static_cast<int>(std::lround(screen.x));
    screenY = static_cast<int>(std::lround(screen.y));
}

void MapView::fitToSnapshot(const RenderSnapshot& snapshot) {
    const WorldBounds bounds = layout.getWorldBoundsFromSnapshot(snapshot);
    cameraState = fitToWorldBounds(
        bounds,
        std::max(1, viewport.width),
        std::max(1, viewport.height),
        cameraState.paddingPx
    );
    hasFittedCamera = true;
}

void MapView::zoomAt(int screenX, int screenY, double zoomFactor) {
    cameraState = zoomAtScreenPoint(
        cameraState,
        static_cast<double>(screenX),
        static_cast<double>(screenY),
        zoomFactor
    );
}

void MapView::beginPan(int screenX, int screenY) {
    panning = true;
    panLastX = screenX;
    panLastY = screenY;
}

void MapView::updatePan(int screenX, int screenY) {
    if (!panning) {
        return;
    }
    const int dx = screenX - panLastX;
    const int dy = screenY - panLastY;
    panLastX = screenX;
    panLastY = screenY;
    if (dx != 0 || dy != 0) {
        cameraState = panByScreenDelta(cameraState, static_cast<double>(dx), static_cast<double>(dy));
    }
}

void MapView::endPan() {
    panning = false;
}

bool MapView::isPanning() const {
    return panning;
}

void MapView::drawLegend(HDC deviceContext) const {
    const int left = 10;
    const int top = 10;
    const int boxW = 172;
    const int boxH = 122;
    RECT panel {left, top, left + boxW, top + boxH};

    HBRUSH panelBrush = CreateSolidBrush(RGB(22, 26, 32));
    HPEN panelPen = CreatePen(PS_SOLID, 1, RGB(78, 86, 98));
    HGDIOBJ oldBrush = SelectObject(deviceContext, panelBrush);
    HGDIOBJ oldPen = SelectObject(deviceContext, panelPen);
    RoundRect(deviceContext, panel.left, panel.top, panel.right, panel.bottom, 8, 8);
    SelectObject(deviceContext, oldBrush);
    SelectObject(deviceContext, oldPen);
    DeleteObject(panelBrush);
    DeleteObject(panelPen);

    HFONT legendFont = CreateFontW(
        -12, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI"
    );
    HGDIOBJ oldFont = SelectObject(deviceContext, legendFont);

    SetBkMode(deviceContext, TRANSPARENT);
    SetTextColor(deviceContext, RGB(226, 232, 240));
    TextOutW(deviceContext, left + 10, top + 6, L"Legenda", 7);

    auto swatch = [&](int y, COLORREF color, const wchar_t* label) {
        HBRUSH brush = CreateSolidBrush(color);
        RECT r {left + 10, y, left + 22, y + 12};
        FillRect(deviceContext, &r, brush);
        DeleteObject(brush);
        TextOutW(deviceContext, left + 28, y - 1, label, lstrlenW(label));
    };

    swatch(top + 28, RGB(50, 140, 220), L"Pojazd (jedzie)");
    swatch(top + 44, RGB(220, 120, 40), L"Pojazd (stoi)");
    swatch(top + 60, RGB(80, 200, 220), L"Pieszy");
    swatch(top + 76, RGB(40, 180, 70), L"Sygnał zielony");
    swatch(top + 92, RGB(180, 40, 40), L"Blokada pasa");

    SelectObject(deviceContext, oldFont);
    DeleteObject(legendFont);
}

void MapView::renderScene(HDC deviceContext, const RenderSnapshot& snapshot) const {
    RECT background {0, 0, viewport.width, viewport.height};
    HBRUSH backgroundBrush = CreateSolidBrush(RGB(28, 32, 38));
    FillRect(deviceContext, &background, backgroundBrush);
    DeleteObject(backgroundBrush);

    // Directed road centerlines (polyline when present) + per-lane offsets when laneCount > 1.
    for (const auto& road : snapshot.roads) {
        const int lanes = std::max(1, road.laneCount);
        std::vector<Vector2D> points = road.centerline;
        if (points.size() < 2) {
            points = {road.start, road.end};
        }

        for (int lane = 0; lane < lanes; ++lane) {
            const COLORREF color = (lanes > 1 && lane > 0)
                ? RGB(70, 70, 78)
                : RGB(90, 90, 90);
            const int width = (lane == 0) ? 3 : 2;
            HPEN roadPen = CreatePen(PS_SOLID, width, color);
            HGDIOBJ oldPen = SelectObject(deviceContext, roadPen);

            bool started = false;
            int prevX = 0;
            int prevY = 0;
            for (size_t i = 0; i + 1 < points.size(); ++i) {
                const double dx = points[i + 1].x - points[i].x;
                const double dy = points[i + 1].y - points[i].y;
                const double len = std::sqrt(dx * dx + dy * dy);
                double ox = 0.0;
                double oy = 0.0;
                if (lanes > 1 && len > 1e-9) {
                    const double px = -dy / len;
                    const double py = dx / len;
                    const double centered = static_cast<double>(lane) - 0.5 * static_cast<double>(lanes - 1);
                    const double offset = centered * SimConstants::LANE_WIDTH;
                    ox = px * offset;
                    oy = py * offset;
                }

                int x1 = 0;
                int y1 = 0;
                int x2 = 0;
                int y2 = 0;
                worldToScreenPixels(points[i].x + ox, points[i].y + oy, x1, y1);
                worldToScreenPixels(points[i + 1].x + ox, points[i + 1].y + oy, x2, y2);

                if (!started) {
                    MoveToEx(deviceContext, x1, y1, nullptr);
                    started = true;
                    prevX = x1;
                    prevY = y1;
                } else if (x1 != prevX || y1 != prevY) {
                    MoveToEx(deviceContext, x1, y1, nullptr);
                }
                LineTo(deviceContext, x2, y2);
                prevX = x2;
                prevY = y2;
            }

            SelectObject(deviceContext, oldPen);
            DeleteObject(roadPen);
        }
    }

    HBRUSH intersectionBrush = CreateSolidBrush(RGB(120, 120, 120));
    for (const auto& intersection : snapshot.intersections) {
        int x = 0;
        int y = 0;
        worldToScreenPixels(intersection.position.x, intersection.position.y, x, y);
        RECT rect {x - 6, y - 6, x + 6, y + 6};
        FillRect(deviceContext, &rect, intersectionBrush);
    }
    DeleteObject(intersectionBrush);

    for (const auto& crossing : snapshot.crossings) {
        int x = 0;
        int y = 0;
        worldToScreenPixels(crossing.worldPosition.x, crossing.worldPosition.y, x, y);
        HBRUSH brush = CreateSolidBrush(crossing.active ? RGB(240, 220, 60) : RGB(160, 160, 160));
        RECT rect {x - 5, y - 5, x + 5, y + 5};
        FillRect(deviceContext, &rect, brush);
        DeleteObject(brush);
    }

    for (const auto& block : snapshot.laneBlockages) {
        if (!block.active) {
            continue;
        }

        if (block.hasMergeZone) {
            int mx1 = 0;
            int my1 = 0;
            int mx2 = 0;
            int my2 = 0;
            worldToScreenPixels(block.mergeStartWorld.x, block.mergeStartWorld.y, mx1, my1);
            worldToScreenPixels(block.startWorld.x, block.startWorld.y, mx2, my2);
            HPEN mergePen = CreatePen(PS_DOT, 2, RGB(220, 160, 40));
            HGDIOBJ prevPen = SelectObject(deviceContext, mergePen);
            MoveToEx(deviceContext, mx1, my1, nullptr);
            LineTo(deviceContext, mx2, my2);
            SelectObject(deviceContext, prevPen);
            DeleteObject(mergePen);
        }

        int x1 = 0;
        int y1 = 0;
        int x2 = 0;
        int y2 = 0;
        worldToScreenPixels(block.startWorld.x, block.startWorld.y, x1, y1);
        worldToScreenPixels(block.endWorld.x, block.endWorld.y, x2, y2);
        HPEN blockPen = CreatePen(PS_SOLID, 5, RGB(180, 40, 40));
        HGDIOBJ prevPen = SelectObject(deviceContext, blockPen);
        MoveToEx(deviceContext, x1, y1, nullptr);
        LineTo(deviceContext, x2, y2);
        SelectObject(deviceContext, prevPen);
        DeleteObject(blockPen);
    }

    for (const auto& pedestrian : snapshot.pedestrians) {
        int x = 0;
        int y = 0;
        worldToScreenPixels(pedestrian.worldPosition.x, pedestrian.worldPosition.y, x, y);
        HBRUSH brush = CreateSolidBrush(
            pedestrian.state == 1 ? RGB(180, 240, 80) : RGB(80, 200, 220)
        );
        RECT rect {x - 4, y - 4, x + 4, y + 4};
        FillRect(deviceContext, &rect, brush);
        DeleteObject(brush);
    }

    for (const auto& light : snapshot.trafficLights) {
        int x = 0;
        int y = 0;
        worldToScreenPixels(light.worldPosition.x, light.worldPosition.y, x, y);
        HBRUSH brush = CreateSolidBrush(light.green ? RGB(40, 180, 70) : RGB(200, 50, 50));
        RECT rect {x - 6, y - 6, x + 6, y + 6};
        FillRect(deviceContext, &rect, brush);
        DeleteObject(brush);
    }

    for (const auto& vehicle : snapshot.vehicles) {
        if (vehicle.finished) {
            continue;
        }

        int x = 0;
        int y = 0;
        worldToScreenPixels(vehicle.worldPosition.x, vehicle.worldPosition.y, x, y);
        HBRUSH brush = CreateSolidBrush(vehicle.stopped ? RGB(220, 120, 40) : RGB(50, 140, 220));
        RECT rect {x - 8, y - 5, x + 8, y + 5};
        FillRect(deviceContext, &rect, brush);
        DeleteObject(brush);
    }

    drawLegend(deviceContext);
}

void MapView::paint(HWND hwnd, HDC targetDeviceContext, const RenderSnapshot& snapshot) {
    RECT clientRect {};
    GetClientRect(hwnd, &clientRect);
    int width = clientRect.right - clientRect.left;
    int height = clientRect.bottom - clientRect.top;

    if (width <= 0 || height <= 0) {
        return;
    }

    viewport.width = width;
    viewport.height = height;
    cameraState = setViewportSize(cameraState, width, height);

    if (!hasFittedCamera) {
        fitToSnapshot(snapshot);
    }

    ensureBuffer(targetDeviceContext, width, height);

    if (bufferDeviceContext == nullptr) {
        renderScene(targetDeviceContext, snapshot);
        return;
    }

    renderScene(bufferDeviceContext, snapshot);
    BitBlt(targetDeviceContext, 0, 0, width, height, bufferDeviceContext, 0, 0, SRCCOPY);
}

} // namespace gui

