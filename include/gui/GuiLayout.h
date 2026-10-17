#pragma once

#include <algorithm>
#include <cmath>

namespace gui {

// Proportional chrome metrics for the Win32 main window (pixels at 96 DPI baseline,
// scaled by dpiScale from LOGPIXELSY/96).
struct GuiLayoutMetrics {
    int clientWidth = 0;
    int clientHeight = 0;
    double dpiScale = 1.0;

    int actionBarHeight = 48;
    int hintStripHeight = 22;
    int statusBarHeight = 26;
    int rightPanelWidth = 320;
    int gap = 8;
    int pad = 10;

    int mapX = 0;
    int mapY = 0;
    int mapWidth = 0;
    int mapHeight = 0;

    int rightX = 0;
    int rightY = 0;
    int rightWidth = 0;
    int rightHeight = 0;

    int statsHeight = 0;
    int controlsHeight = 0;
    int logHeight = 0;

    int statusY = 0;
    int hintY = 0;

    static constexpr int kMinClientWidth = 1100;
    static constexpr int kMinClientHeight = 680;

    int S(int px) const {
        return std::max(1, static_cast<int>(std::lround(static_cast<double>(px) * dpiScale)));
    }

    void compute(int width, int height, double scale = 1.0) {
        dpiScale = (scale > 0.01) ? scale : 1.0;
        clientWidth = std::max(1, width);
        clientHeight = std::max(1, height);

        actionBarHeight = S(48);
        hintStripHeight = S(22);
        statusBarHeight = S(26);
        gap = S(8);
        pad = S(10);

        // Right column ~24% of width, clamped for readable Polish labels.
        rightPanelWidth = std::clamp(clientWidth * 24 / 100, S(300), S(380));

        const int contentTop = actionBarHeight + hintStripHeight;
        const int contentBottom = clientHeight - statusBarHeight;
        const int contentHeight = std::max(S(80), contentBottom - contentTop);

        hintY = actionBarHeight;
        mapX = gap;
        mapY = contentTop + gap;
        mapWidth = std::max(S(160), clientWidth - rightPanelWidth - gap * 3);
        mapHeight = std::max(S(160), contentHeight - gap * 2);

        rightX = mapX + mapWidth + gap;
        rightY = mapY;
        rightWidth = std::max(S(220), clientWidth - rightX - gap);
        rightHeight = mapHeight;

        // Stack: stats ~22%, controls ~50% (tempo/blocks/random sections), log remainder.
        statsHeight = std::max(S(100), rightHeight * 22 / 100);
        controlsHeight = std::max(S(300), rightHeight * 50 / 100);
        logHeight = std::max(S(90), rightHeight - statsHeight - controlsHeight - gap * 2);

        statusY = clientHeight - statusBarHeight;
    }
};

} // namespace gui
