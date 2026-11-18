#pragma once

#ifndef UNICODE
#define UNICODE
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <string>

#include "osm/GeoBbox.h"

namespace gui {

struct MapDownloadDialogResult {
    bool accepted = false;
    std::string cachedOverpassPath;
    std::string placeLabel;
    double radiusMeters = 500.0;
    osm_net::GeoBbox bbox;
};

// Modal Win32 dialog: text search + locator map (pin+size / rectangle) → download.
// Network work runs on a worker thread; UI updates via PostMessage only.
// On cancel/close: sets cancel flag and joins the worker.
// Locator underlay is a drawn grid (no tile API) — see etap4 provider notes.
class MapDownloadDialog {
public:
    static MapDownloadDialogResult run(HWND owner);
};

} // namespace gui
