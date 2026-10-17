#pragma once

#ifndef UNICODE
#define UNICODE
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <string>

namespace gui {

struct MapDownloadDialogResult {
    bool accepted = false;
    std::string cachedOverpassPath;
    std::string placeLabel;
    double radiusMeters = 500.0;
};

// Modal Win32 dialog: place name → Szukaj → pick → area size → download.
// Network work runs on a worker thread; UI updates via PostMessage only.
// On cancel/close: sets cancel flag and joins the worker.
class MapDownloadDialog {
public:
    static MapDownloadDialogResult run(HWND owner);
};

} // namespace gui
