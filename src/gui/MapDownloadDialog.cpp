#include "gui/MapDownloadDialog.h"

#include "gui/LocatorMap.h"
#include "osm/MapDownloadService.h"
#include "osm/WinHttpTransport.h"

#include <windowsx.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

namespace gui {
namespace {

constexpr int IDC_PLACE_EDIT = 501;
constexpr int IDC_SEARCH_BTN = 502;
constexpr int IDC_RESULTS = 503;
constexpr int IDC_AREA_COMBO = 504;
constexpr int IDC_FORCE_CHECK = 505;
constexpr int IDC_DOWNLOAD_BTN = 506;
constexpr int IDC_CANCEL_BTN = 507;
constexpr int IDC_STATUS = 508;
constexpr int IDC_HINT = 509;
constexpr int IDC_ATTRIB = 510;
constexpr int IDC_LOCATOR = 511;
constexpr int IDC_MODE_PAN = 512;
constexpr int IDC_MODE_PIN = 513;
constexpr int IDC_MODE_RECT = 514;
constexpr int IDC_CLEAR_SEL = 515;
constexpr int IDC_METRICS = 516;
constexpr int IDC_PLACE_LABEL = 517;
constexpr int IDC_OFFLINE_NOTE = 518;

constexpr UINT WM_DL_PROGRESS = WM_APP + 40;
constexpr UINT WM_DL_GEOCODE_DONE = WM_APP + 41;
constexpr UINT WM_DL_FETCH_DONE = WM_APP + 42;

std::string wideToUtf8(const std::wstring& wide) {
    if (wide.empty()) {
        return {};
    }
    const int size = WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, nullptr, 0, nullptr, nullptr);
    if (size <= 1) {
        return {};
    }
    std::string out(static_cast<size_t>(size - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, wide.c_str(), -1, out.data(), size, nullptr, nullptr);
    return out;
}

std::wstring utf8ToWide(const std::string& utf8) {
    if (utf8.empty()) {
        return {};
    }
    const int size = MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, nullptr, 0);
    if (size <= 1) {
        return {};
    }
    std::wstring out(static_cast<size_t>(size - 1), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, utf8.c_str(), -1, out.data(), size);
    return out;
}

struct ProgressMsg {
    std::wstring text;
};

struct GeocodeDoneMsg {
    osm_net::GeocodeResult result;
};

struct FetchDoneMsg {
    osm_net::MapFetchResult result;
    std::string placeLabel;
    double radiusMeters = 500.0;
    osm_net::GeoBbox bbox;
};

struct DialogState {
    HWND hwnd = nullptr;
    HWND placeEdit = nullptr;
    HWND searchBtn = nullptr;
    HWND resultsList = nullptr;
    HWND areaCombo = nullptr;
    HWND forceCheck = nullptr;
    HWND downloadBtn = nullptr;
    HWND cancelBtn = nullptr;
    HWND statusLabel = nullptr;
    HWND locator = nullptr;
    HWND modePan = nullptr;
    HWND modePin = nullptr;
    HWND modeRect = nullptr;
    HWND clearBtn = nullptr;
    HWND metricsLabel = nullptr;
    HWND placeLabel = nullptr;

    std::vector<osm_net::PlaceHit> hits;
    MapDownloadDialogResult out;
    std::string selectedPlaceName;

    LocatorCamera camera;
    MapAreaSelection selection;

    std::atomic<bool> cancelFlag{false};
    std::atomic<bool> busy{false};
    std::thread worker;
    std::mutex joinMutex;

    osm_net::WinHttpTransport transport;
    std::unique_ptr<osm_net::MapDownloadService> service;

    void ensureService() {
        if (!service) {
            service = std::make_unique<osm_net::MapDownloadService>(transport);
        }
    }

    void joinWorker() {
        std::lock_guard<std::mutex> lock(joinMutex);
        if (worker.joinable()) {
            worker.join();
        }
    }

    void setBusy(bool value) {
        busy = value;
        EnableWindow(placeEdit, value ? FALSE : TRUE);
        EnableWindow(searchBtn, value ? FALSE : TRUE);
        EnableWindow(resultsList, value ? FALSE : TRUE);
        EnableWindow(areaCombo, value ? FALSE : TRUE);
        EnableWindow(forceCheck, value ? FALSE : TRUE);
        EnableWindow(downloadBtn, value ? FALSE : TRUE);
        EnableWindow(locator, value ? FALSE : TRUE);
        EnableWindow(modePan, value ? FALSE : TRUE);
        EnableWindow(modePin, value ? FALSE : TRUE);
        EnableWindow(modeRect, value ? FALSE : TRUE);
        EnableWindow(clearBtn, value ? FALSE : TRUE);
        SetWindowTextW(cancelBtn, value ? L"Anuluj czekanie" : L"Zamknij");
    }

    void setStatus(const std::wstring& text) {
        SetWindowTextW(statusLabel, text.c_str());
    }

    void syncModeRadios() {
        SendMessageW(modePan, BM_SETCHECK,
            selection.mode == LocatorGestureMode::Pan ? BST_CHECKED : BST_UNCHECKED, 0);
        SendMessageW(modePin, BM_SETCHECK,
            selection.mode == LocatorGestureMode::PinAndSize ? BST_CHECKED : BST_UNCHECKED, 0);
        SendMessageW(modeRect, BM_SETCHECK,
            selection.mode == LocatorGestureMode::DrawRectangle ? BST_CHECKED : BST_UNCHECKED, 0);
    }

    void refreshMetrics() {
        std::wostringstream ss;
        if (!selection.hasSelection || !selection.selection.isValid()) {
            ss << L"Wybór: brak  ·  Limit: bok ≤ ~4 km, powierzchnia ≤ ~20 km²";
        } else {
            const SelectionMetrics m = measureSelection(selection.selection);
            ss << L"Bbox: " << utf8ToWide(selection.selection.toKey(5))
               << L"  ·  szer. " << static_cast<int>(std::lround(m.widthMeters)) << L" m"
               << L"  ·  wys. " << static_cast<int>(std::lround(m.heightMeters)) << L" m"
               << L"  ·  ~" << std::fixed;
            ss.precision(2);
            ss << m.areaKm2 << L" km²";
            if (!m.withinLimits) {
                ss << L"  ·  POZA LIMITEM";
            }
        }
        SetWindowTextW(metricsLabel, ss.str().c_str());

        std::wstring placeText = L"Miejsce: ";
        if (!selectedPlaceName.empty()) {
            placeText += utf8ToWide(selectedPlaceName);
        } else if (selection.hasPin) {
            placeText += L"(pinezka bez nazwy)";
        } else {
            placeText += L"—";
        }
        SetWindowTextW(placeLabel, placeText.c_str());

        if (locator != nullptr) {
            InvalidateRect(locator, nullptr, FALSE);
        }
    }

    void applyAreaComboToSelection() {
        const int areaIndex = static_cast<int>(SendMessageW(areaCombo, CB_GETCURSEL, 0, 0));
        const int idx = (areaIndex >= 0 && areaIndex < osm_net::kAreaSizeChoiceCount)
            ? areaIndex
            : osm_net::kDefaultAreaSizeIndex;
        setSizeRadius(selection, osm_net::kAreaSizeChoices[idx].radiusMeters);
        refreshMetrics();
    }
};

DialogState* stateFromHwnd(HWND hwnd) {
    return reinterpret_cast<DialogState*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
}

void postProgress(HWND hwnd, const std::string& utf8) {
    auto* msg = new ProgressMsg{utf8ToWide(utf8)};
    if (!PostMessageW(hwnd, WM_DL_PROGRESS, 0, reinterpret_cast<LPARAM>(msg))) {
        delete msg;
    }
}

void paintLocator(HWND hwnd, DialogState* st) {
    PAINTSTRUCT ps {};
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT rc {};
    GetClientRect(hwnd, &rc);
    const int w = rc.right - rc.left;
    const int h = rc.bottom - rc.top;
    st->camera = setLocatorViewport(st->camera, w, h);

    HBRUSH bg = CreateSolidBrush(RGB(232, 238, 244));
    FillRect(hdc, &rc, bg);
    DeleteObject(bg);

    // Simple geographic grid — not OSM tiles (no tile fetch / no paid API).
    HPEN gridPen = CreatePen(PS_SOLID, 1, RGB(198, 208, 218));
    HGDIOBJ oldPen = SelectObject(hdc, gridPen);
    const int step = 40;
    for (int x = 0; x < w; x += step) {
        MoveToEx(hdc, x, 0, nullptr);
        LineTo(hdc, x, h);
    }
    for (int y = 0; y < h; y += step) {
        MoveToEx(hdc, 0, y, nullptr);
        LineTo(hdc, w, y);
    }

    // Crosshair at center.
    HPEN crossPen = CreatePen(PS_DOT, 1, RGB(140, 150, 160));
    SelectObject(hdc, crossPen);
    MoveToEx(hdc, w / 2, 0, nullptr);
    LineTo(hdc, w / 2, h);
    MoveToEx(hdc, 0, h / 2, nullptr);
    LineTo(hdc, w, h / 2);

    // Selection rectangle (authoritative download bbox).
    if (st->selection.hasSelection && st->selection.selection.isValid()) {
        const auto& box = st->selection.selection;
        const LocatorScreenCoord sw = geoToScreen(st->camera, box.south, box.west);
        const LocatorScreenCoord ne = geoToScreen(st->camera, box.north, box.east);
        const int left = static_cast<int>(std::floor(std::min(sw.x, ne.x)));
        const int right = static_cast<int>(std::ceil(std::max(sw.x, ne.x)));
        const int top = static_cast<int>(std::floor(std::min(sw.y, ne.y)));
        const int bottom = static_cast<int>(std::ceil(std::max(sw.y, ne.y)));

        const SelectionMetrics metrics = measureSelection(box);
        const COLORREF fillColor = metrics.withinLimits ? RGB(70, 130, 200) : RGB(200, 80, 70);
        // Hatch fill + outline (GDI has no alpha without layered windows).
        HBRUSH hatch = CreateHatchBrush(HS_BDIAGONAL, fillColor);
        HPEN boxPen = CreatePen(PS_SOLID, 2, fillColor);
        HGDIOBJ prevBrush = SelectObject(hdc, hatch);
        HGDIOBJ prevPen = SelectObject(hdc, boxPen);
        SetBkMode(hdc, TRANSPARENT);
        Rectangle(hdc, left, top, right, bottom);
        SelectObject(hdc, prevBrush);
        SelectObject(hdc, prevPen);
        DeleteObject(hatch);
        DeleteObject(boxPen);
    }

    // Pin.
    if (st->selection.hasPin) {
        const LocatorScreenCoord pin = geoToScreen(st->camera, st->selection.pinLat, st->selection.pinLon);
        const int px = static_cast<int>(std::lround(pin.x));
        const int py = static_cast<int>(std::lround(pin.y));
        HBRUSH pinBrush = CreateSolidBrush(RGB(200, 40, 40));
        HPEN pinPen = CreatePen(PS_SOLID, 1, RGB(120, 20, 20));
        SelectObject(hdc, pinPen);
        SelectObject(hdc, pinBrush);
        Ellipse(hdc, px - 6, py - 6, px + 6, py + 6);
        MoveToEx(hdc, px, py, nullptr);
        LineTo(hdc, px, py + 14);
        DeleteObject(pinBrush);
        DeleteObject(pinPen);
    }

    // Offline / underlay honesty caption.
    SetBkMode(hdc, TRANSPARENT);
    SetTextColor(hdc, RGB(70, 80, 90));
    const wchar_t* caption =
        L"Locator (siatka) — nie kafle OSM. Prostokąt = obszar Overpass. "
        L"Kółko myszy: zoom.";
    TextOutW(hdc, 8, h - 22, caption, static_cast<int>(wcslen(caption)));

    SelectObject(hdc, oldPen);
    DeleteObject(crossPen);
    DeleteObject(gridPen);
    EndPaint(hwnd, &ps);
}

LRESULT CALLBACK locatorProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    DialogState* st = stateFromHwnd(GetParent(hwnd));
    if (st == nullptr) {
        // During create, parent user-data may already be set; fall back.
        st = stateFromHwnd(hwnd);
    }
    // Locator stores DialogState* on itself as well.
    if (msg == WM_CREATE) {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lParam);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(cs->lpCreateParams));
        return 0;
    }
    st = reinterpret_cast<DialogState*>(GetWindowLongPtr(hwnd, GWLP_USERDATA));
    if (st == nullptr) {
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }

    switch (msg) {
        case WM_PAINT:
            paintLocator(hwnd, st);
            return 0;
        case WM_LBUTTONDOWN: {
            if (st->busy.load()) {
                return 0;
            }
            SetCapture(hwnd);
            RECT rc {};
            GetClientRect(hwnd, &rc);
            st->camera = setLocatorViewport(st->camera, rc.right - rc.left, rc.bottom - rc.top);
            locatorMouseDown(st->camera, st->selection,
                static_cast<double>(LOWORD(lParam)), static_cast<double>(HIWORD(lParam)));
            st->refreshMetrics();
            return 0;
        }
        case WM_MOUSEMOVE: {
            if ((wParam & MK_LBUTTON) == 0) {
                return 0;
            }
            locatorMouseMove(st->camera, st->selection,
                static_cast<double>(LOWORD(lParam)), static_cast<double>(HIWORD(lParam)));
            st->refreshMetrics();
            return 0;
        }
        case WM_LBUTTONUP: {
            locatorMouseUp(st->selection);
            ReleaseCapture();
            st->refreshMetrics();
            return 0;
        }
        case WM_MOUSEWHEEL: {
            POINT pt {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
            ScreenToClient(hwnd, &pt);
            RECT rc {};
            GetClientRect(hwnd, &rc);
            st->camera = setLocatorViewport(st->camera, rc.right - rc.left, rc.bottom - rc.top);
            const bool zoomIn = GET_WHEEL_DELTA_WPARAM(wParam) > 0;
            locatorWheelZoom(st->camera, static_cast<double>(pt.x), static_cast<double>(pt.y), zoomIn);
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        }
        case WM_ERASEBKGND:
            return 1;
        default:
            break;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

void startGeocode(DialogState* st) {
    if (st->busy.load()) {
        return;
    }
    wchar_t buf[512] {};
    GetWindowTextW(st->placeEdit, buf, 512);
    const std::string query = wideToUtf8(buf);
    if (query.empty()) {
        st->setStatus(L"Wpisz nazwę miejsca, potem kliknij Szukaj.");
        return;
    }

    const bool force = SendMessageW(st->forceCheck, BM_GETCHECK, 0, 0) == BST_CHECKED;
    st->cancelFlag = false;
    st->setBusy(true);
    st->setStatus(L"Szukanie…");
    st->joinWorker();

    HWND hwnd = st->hwnd;
    st->worker = std::thread([st, query, force, hwnd]() {
        st->ensureService();
        osm_net::GeocodeResult result = st->service->searchPlaces(
            query,
            force,
            &st->cancelFlag,
            [hwnd](const std::string& phase) { postProgress(hwnd, phase); }
        );
        auto* msg = new GeocodeDoneMsg{std::move(result)};
        if (!PostMessageW(hwnd, WM_DL_GEOCODE_DONE, 0, reinterpret_cast<LPARAM>(msg))) {
            delete msg;
        }
    });
}

void applySelectedHitToLocator(DialogState* st, int sel) {
    if (sel < 0 || sel >= static_cast<int>(st->hits.size())) {
        return;
    }
    const osm_net::PlaceHit& hit = st->hits[static_cast<size_t>(sel)];
    st->selectedPlaceName = hit.displayName;
    // Center + pin only — never auto-start Overpass download.
    applySearchHit(st->camera, st->selection, hit.lat, hit.lon);
    st->applyAreaComboToSelection();
    st->setStatus(L"Ustawiono pinezkę na wyniku wyszukiwania. Dopasuj obszar, potem Pobierz.");
    st->refreshMetrics();
}

void startDownload(DialogState* st) {
    if (st->busy.load()) {
        return;
    }
    if (!st->selection.hasSelection || !st->selection.selection.isValid()) {
        st->setStatus(L"Zaznacz obszar: pinezka+rozmiar albo narysuj prostokąt.");
        return;
    }

    const SelectionMetrics metrics = measureSelection(st->selection.selection);
    if (!metrics.withinLimits) {
        st->setStatus(utf8ToWide(metrics.limitError.empty()
            ? "Obszar poza limitem."
            : metrics.limitError));
        return;
    }

    const bool largeWarn = metrics.areaKm2 > 2.0
        || metrics.widthMeters > 1500.0
        || metrics.heightMeters > 1500.0;
    if (largeWarn) {
        const int answer = MessageBoxW(
            st->hwnd,
            L"Wybrano większy obszar (prostokątny bbox).\n"
            L"Pobranie może potrwać dłużej i zająć więcej miejsca.\n\n"
            L"Kontynuować?",
            L"Ostrzeżenie — większy obszar",
            MB_YESNO | MB_ICONWARNING
        );
        if (answer != IDYES) {
            return;
        }
    }

    const bool force = SendMessageW(st->forceCheck, BM_GETCHECK, 0, 0) == BST_CHECKED;
    const osm_net::GeoBbox bbox = st->selection.selection;
    const double radius = st->selection.sizeRadiusMeters;
    std::string placeLabel = st->selectedPlaceName;
    if (placeLabel.empty() && st->selection.hasPin) {
        placeLabel = "Wybór na mapie";
    }

    st->cancelFlag = false;
    st->setBusy(true);
    st->setStatus(L"Pobieranie mapy (Overpass)…");
    st->joinWorker();

    HWND hwnd = st->hwnd;
    st->worker = std::thread([st, bbox, radius, force, hwnd, placeLabel]() {
        st->ensureService();
        // Explicit bbox path: shown rect ≡ query ≡ cache key.
        osm_net::MapFetchResult result = st->service->downloadRoadsForBbox(
            bbox,
            force,
            &st->cancelFlag,
            [hwnd](const std::string& phase) { postProgress(hwnd, phase); }
        );
        auto* msg = new FetchDoneMsg{std::move(result), placeLabel, radius, bbox};
        if (!PostMessageW(hwnd, WM_DL_FETCH_DONE, 0, reinterpret_cast<LPARAM>(msg))) {
            delete msg;
        }
    });
}

void onCancelOrClose(DialogState* st) {
    if (st->busy.load()) {
        st->cancelFlag = true;
        st->setStatus(L"Anulowanie…");
        return;
    }
    st->cancelFlag = true;
    st->joinWorker();
    DestroyWindow(st->hwnd);
}

LRESULT CALLBACK dialogProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (msg == WM_CREATE) {
        auto* cs = reinterpret_cast<CREATESTRUCTW*>(lParam);
        auto* st = reinterpret_cast<DialogState*>(cs->lpCreateParams);
        SetWindowLongPtr(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(st));
        st->hwnd = hwnd;

        CreateWindowExW(0, L"STATIC",
            L"1) Szukaj miejsca  2) Pinezka / prostokąt na locatorze  3) Sprawdź wymiary  4) Pobierz",
            WS_CHILD | WS_VISIBLE, 12, 8, 760, 18, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_HINT)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC", L"Miejsce:", WS_CHILD | WS_VISIBLE,
            12, 34, 60, 20, hwnd, nullptr, cs->hInstance, nullptr);
        st->placeEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
            WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | WS_TABSTOP,
            72, 30, 520, 26, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_PLACE_EDIT)), cs->hInstance, nullptr);
        st->searchBtn = CreateWindowExW(0, L"BUTTON", L"Szukaj",
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
            600, 30, 100, 26, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_SEARCH_BTN)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC", L"Wyniki:", WS_CHILD | WS_VISIBLE,
            12, 64, 60, 18, hwnd, nullptr, cs->hInstance, nullptr);
        st->resultsList = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", L"",
            WS_CHILD | WS_VISIBLE | WS_VSCROLL | LBS_NOTIFY | WS_TABSTOP,
            12, 84, 280, 150, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_RESULTS)), cs->hInstance, nullptr);

        // Locator canvas (drawn grid — not tile underlay).
        st->locator = CreateWindowExW(
            WS_EX_CLIENTEDGE,
            L"EasyRiderLocatorMap",
            L"",
            WS_CHILD | WS_VISIBLE | WS_TABSTOP,
            304, 84, 460, 260,
            hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_LOCATOR)),
            cs->hInstance,
            st
        );

        CreateWindowExW(0, L"STATIC", L"Gest:", WS_CHILD | WS_VISIBLE,
            12, 244, 40, 18, hwnd, nullptr, cs->hInstance, nullptr);
        st->modePan = CreateWindowExW(0, L"BUTTON", L"Pan",
            WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_GROUP | WS_TABSTOP,
            52, 242, 55, 20, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_MODE_PAN)), cs->hInstance, nullptr);
        st->modePin = CreateWindowExW(0, L"BUTTON", L"Pinezka+rozmiar",
            WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_TABSTOP,
            110, 242, 120, 20, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_MODE_PIN)), cs->hInstance, nullptr);
        st->modeRect = CreateWindowExW(0, L"BUTTON", L"Prostokąt",
            WS_CHILD | WS_VISIBLE | BS_AUTORADIOBUTTON | WS_TABSTOP,
            232, 242, 70, 20, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_MODE_RECT)), cs->hInstance, nullptr);
        st->selection.mode = LocatorGestureMode::PinAndSize;
        st->syncModeRadios();

        CreateWindowExW(0, L"STATIC",
            L"Rozmiar (pinezka → prostokątny bbox):",
            WS_CHILD | WS_VISIBLE, 12, 272, 280, 18, hwnd, nullptr, cs->hInstance, nullptr);
        st->areaCombo = CreateWindowExW(0, L"COMBOBOX", L"",
            WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_TABSTOP,
            12, 292, 200, 200, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_AREA_COMBO)), cs->hInstance, nullptr);
        for (int i = 0; i < osm_net::kAreaSizeChoiceCount; ++i) {
            SendMessageW(st->areaCombo, CB_ADDSTRING, 0,
                reinterpret_cast<LPARAM>(osm_net::kAreaSizeChoices[i].labelUi));
        }
        SendMessageW(st->areaCombo, CB_SETCURSEL, osm_net::kDefaultAreaSizeIndex, 0);

        st->clearBtn = CreateWindowExW(0, L"BUTTON", L"Wyczyść wybór",
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
            220, 292, 80, 24, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_CLEAR_SEL)), cs->hInstance, nullptr);

        st->forceCheck = CreateWindowExW(0, L"BUTTON", L"Wymuś ponowne pobranie (Ignore cache)",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP,
            12, 324, 280, 22, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_FORCE_CHECK)), cs->hInstance, nullptr);

        st->placeLabel = CreateWindowExW(0, L"STATIC", L"Miejsce: —",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            304, 350, 460, 18, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_PLACE_LABEL)), cs->hInstance, nullptr);

        st->metricsLabel = CreateWindowExW(0, L"STATIC", L"Wybór: brak",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            12, 352, 750, 36, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_METRICS)), cs->hInstance, nullptr);

        st->downloadBtn = CreateWindowExW(0, L"BUTTON", L"Pobierz i zaimportuj",
            WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON | WS_TABSTOP,
            12, 396, 180, 30, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_DOWNLOAD_BTN)), cs->hInstance, nullptr);
        st->cancelBtn = CreateWindowExW(0, L"BUTTON", L"Zamknij",
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
            200, 396, 140, 30, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_CANCEL_BTN)), cs->hInstance, nullptr);

        st->statusLabel = CreateWindowExW(0, L"STATIC",
            L"Gotowe — Szukaj tylko po kliknięciu. Locator bez kafelków (offline OK).",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            12, 434, 750, 36, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_STATUS)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC",
            L"Dane dróg: © OpenStreetMap (ODbL) · Nominatim · Overpass. "
            L"Locator ≠ kafle; Pobierz nie ściąga archiwum kafelków. Cache: data/cache/",
            WS_CHILD | WS_VISIBLE, 12, 474, 750, 36, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_ATTRIB)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC",
            L"Offline: poprzednia mapa zostaje; import lokalny działa; brak „prawdziwej” mapy w locatorze.",
            WS_CHILD | WS_VISIBLE, 12, 512, 750, 20, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_OFFLINE_NOTE)), cs->hInstance, nullptr);

        st->applyAreaComboToSelection();
        st->refreshMetrics();
        return 0;
    }

    DialogState* st = stateFromHwnd(hwnd);
    if (st == nullptr && msg != WM_DESTROY) {
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }

    switch (msg) {
        case WM_COMMAND: {
            const int id = LOWORD(wParam);
            const int code = HIWORD(wParam);
            if (id == IDC_SEARCH_BTN) {
                startGeocode(st);
                return 0;
            }
            if (id == IDC_DOWNLOAD_BTN) {
                startDownload(st);
                return 0;
            }
            if (id == IDC_CANCEL_BTN) {
                onCancelOrClose(st);
                return 0;
            }
            if (id == IDC_CLEAR_SEL) {
                clearSelection(st->selection);
                st->selectedPlaceName.clear();
                st->refreshMetrics();
                st->setStatus(L"Wyczyszczono wybór.");
                return 0;
            }
            if (id == IDC_MODE_PAN) {
                st->selection.mode = LocatorGestureMode::Pan;
                st->selection.drawingRect = false;
                st->selection.draggingPin = false;
                st->syncModeRadios();
                return 0;
            }
            if (id == IDC_MODE_PIN) {
                st->selection.mode = LocatorGestureMode::PinAndSize;
                st->selection.drawingRect = false;
                if (st->selection.hasPin) {
                    applyPinSizeToSelection(st->selection);
                }
                st->syncModeRadios();
                st->refreshMetrics();
                return 0;
            }
            if (id == IDC_MODE_RECT) {
                st->selection.mode = LocatorGestureMode::DrawRectangle;
                st->selection.draggingPin = false;
                st->syncModeRadios();
                st->setStatus(L"Tryb prostokąta: przeciągnij na locatorze. Pobieranie tylko po Pobierz.");
                return 0;
            }
            if (id == IDC_AREA_COMBO && code == CBN_SELCHANGE) {
                st->applyAreaComboToSelection();
                return 0;
            }
            if (id == IDC_RESULTS && code == LBN_SELCHANGE) {
                const int sel = static_cast<int>(SendMessageW(st->resultsList, LB_GETCURSEL, 0, 0));
                applySelectedHitToLocator(st, sel);
                return 0;
            }
            // Deliberately ignore EN_CHANGE on place edit — no search-on-keystroke.
            return 0;
        }
        case WM_DL_PROGRESS: {
            auto* msgProgress = reinterpret_cast<ProgressMsg*>(lParam);
            if (msgProgress != nullptr) {
                st->setStatus(msgProgress->text);
                delete msgProgress;
            }
            return 0;
        }
        case WM_DL_GEOCODE_DONE: {
            st->joinWorker();
            st->setBusy(false);
            auto* done = reinterpret_cast<GeocodeDoneMsg*>(lParam);
            if (done == nullptr) {
                return 0;
            }
            SendMessageW(st->resultsList, LB_RESETCONTENT, 0, 0);
            st->hits.clear();
            if (done->result.cancelled) {
                st->setStatus(L"Szukanie anulowane.");
            } else if (!done->result.ok) {
                st->setStatus(utf8ToWide(
                    done->result.error.empty() ? "Geokodowanie nie powiodło się." : done->result.error
                ));
            } else if (done->result.hits.empty()) {
                st->setStatus(L"Brak wyników. Spróbuj innej nazwy.");
            } else {
                st->hits = done->result.hits;
                for (const auto& hit : st->hits) {
                    // Primary label = place name only (no OSM technical IDs).
                    SendMessageW(st->resultsList, LB_ADDSTRING, 0,
                        reinterpret_cast<LPARAM>(utf8ToWide(hit.displayName).c_str()));
                }
                SendMessageW(st->resultsList, LB_SETCURSEL, 0, 0);
                applySelectedHitToLocator(st, 0);
                std::wstring status = done->result.fromCache
                    ? L"Wyniki z cache. Pinezka ustawiona — dopasuj obszar, potem Pobierz."
                    : L"Znaleziono wyniki. Pinezka ustawiona — bez auto-pobierania.";
                st->setStatus(status);
            }
            delete done;
            return 0;
        }
        case WM_DL_FETCH_DONE: {
            st->joinWorker();
            st->setBusy(false);
            auto* done = reinterpret_cast<FetchDoneMsg*>(lParam);
            if (done == nullptr) {
                return 0;
            }
            if (done->result.cancelled) {
                st->setStatus(L"Pobieranie anulowane — poprzednia mapa bez zmian.");
            } else if (!done->result.ok) {
                st->setStatus(utf8ToWide(
                    done->result.error.empty() ? "Pobieranie nie powiodło się." : done->result.error
                ));
            } else {
                st->out.accepted = true;
                st->out.cachedOverpassPath = done->result.cachePath;
                st->out.placeLabel = done->placeLabel;
                st->out.radiusMeters = done->radiusMeters;
                st->out.bbox = done->result.bbox.isValid() ? done->result.bbox : done->bbox;
                std::wstring status = done->result.fromCache
                    ? L"Użyto cache mapy. Zamykanie — import…"
                    : L"Pobrano. Zamykanie — import…";
                st->setStatus(status);
                DestroyWindow(hwnd);
            }
            delete done;
            return 0;
        }
        case WM_CLOSE:
            onCancelOrClose(st);
            return 0;
        case WM_DESTROY: {
            if (st != nullptr) {
                st->cancelFlag = true;
                st->joinWorker();
            }
            PostQuitMessage(0);
            return 0;
        }
        default:
            break;
    }
    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

} // namespace

MapDownloadDialogResult MapDownloadDialog::run(HWND owner) {
    MapDownloadDialogResult result;
    DialogState state;
    state.out = result;

    static bool registered = false;
    if (!registered) {
        WNDCLASSEXW wc {};
        wc.cbSize = sizeof(wc);
        wc.lpfnWndProc = dialogProc;
        wc.hInstance = GetModuleHandleW(nullptr);
        wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
        wc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        wc.lpszClassName = L"EasyRiderMapDownloadDialog";
        RegisterClassExW(&wc);

        WNDCLASSEXW loc {};
        loc.cbSize = sizeof(loc);
        loc.lpfnWndProc = locatorProc;
        loc.hInstance = GetModuleHandleW(nullptr);
        loc.hCursor = LoadCursor(nullptr, IDC_CROSS);
        loc.hbrBackground = reinterpret_cast<HBRUSH>(COLOR_WINDOW + 1);
        loc.lpszClassName = L"EasyRiderLocatorMap";
        RegisterClassExW(&loc);
        registered = true;
    }

    RECT ownerRect {};
    if (owner != nullptr) {
        GetWindowRect(owner, &ownerRect);
    }
    const int width = 800;
    const int height = 580;
    const int x = owner != nullptr
        ? ownerRect.left + std::max(0L, (ownerRect.right - ownerRect.left - width) / 2)
        : CW_USEDEFAULT;
    const int y = owner != nullptr
        ? ownerRect.top + std::max(0L, (ownerRect.bottom - ownerRect.top - height) / 2)
        : CW_USEDEFAULT;

    HWND hwnd = CreateWindowExW(
        WS_EX_DLGMODALFRAME,
        L"EasyRiderMapDownloadDialog",
        L"Szukaj miejsca / wybór obszaru (pinezka lub prostokąt)",
        WS_CAPTION | WS_SYSMENU | WS_POPUP | WS_VISIBLE,
        x, y, width, height,
        owner,
        nullptr,
        GetModuleHandleW(nullptr),
        &state
    );
    if (hwnd == nullptr) {
        return result;
    }

    if (owner != nullptr) {
        EnableWindow(owner, FALSE);
    }

    MSG msg {};
    while (GetMessageW(&msg, nullptr, 0, 0) > 0) {
        if (!IsDialogMessageW(hwnd, &msg)) {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    if (owner != nullptr) {
        EnableWindow(owner, TRUE);
        SetForegroundWindow(owner);
    }

    state.cancelFlag = true;
    state.joinWorker();
    return state.out;
}

} // namespace gui
