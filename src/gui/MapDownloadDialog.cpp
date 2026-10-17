#include "gui/MapDownloadDialog.h"

#include "osm/MapDownloadService.h"
#include "osm/WinHttpTransport.h"

#include <algorithm>
#include <atomic>
#include <memory>
#include <mutex>
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

    std::vector<osm_net::PlaceHit> hits;
    MapDownloadDialogResult out;

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
        // Cancel stays enabled while busy so the user can abort the wait.
        SetWindowTextW(cancelBtn, value ? L"Anuluj czekanie" : L"Zamknij");
    }

    void setStatus(const std::wstring& text) {
        SetWindowTextW(statusLabel, text.c_str());
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

void startDownload(DialogState* st) {
    if (st->busy.load()) {
        return;
    }
    const int sel = static_cast<int>(SendMessageW(st->resultsList, LB_GETCURSEL, 0, 0));
    if (sel < 0 || sel >= static_cast<int>(st->hits.size())) {
        st->setStatus(L"Wybierz wynik z listy.");
        return;
    }

    const int areaIndex = static_cast<int>(SendMessageW(st->areaCombo, CB_GETCURSEL, 0, 0));
    const int idx = (areaIndex >= 0 && areaIndex < osm_net::kAreaSizeChoiceCount)
        ? areaIndex
        : osm_net::kDefaultAreaSizeIndex;
    const auto& choice = osm_net::kAreaSizeChoices[idx];

    if (choice.warnBeforeDownload) {
        const int answer = MessageBoxW(
            st->hwnd,
            L"Wybrano większy obszar (prostokątny bbox wokół środka).\n"
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
    const osm_net::PlaceHit hit = st->hits[static_cast<size_t>(sel)];
    const double radius = choice.radiusMeters;

    st->cancelFlag = false;
    st->setBusy(true);
    st->setStatus(L"Pobieranie mapy…");
    st->joinWorker();

    HWND hwnd = st->hwnd;
    st->worker = std::thread([st, hit, radius, force, hwnd]() {
        st->ensureService();
        osm_net::MapFetchResult result = st->service->downloadRoads(
            hit.lat,
            hit.lon,
            radius,
            force,
            &st->cancelFlag,
            [hwnd](const std::string& phase) { postProgress(hwnd, phase); }
        );
        auto* msg = new FetchDoneMsg{std::move(result), hit.displayName, radius};
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
            L"1) Wpisz nazwę miejsca  2) Szukaj  3) Wybierz wynik  4) Rozmiar obszaru  5) Pobierz",
            WS_CHILD | WS_VISIBLE, 12, 10, 560, 18, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_HINT)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC", L"Miejsce:", WS_CHILD | WS_VISIBLE,
            12, 38, 60, 20, hwnd, nullptr, cs->hInstance, nullptr);
        st->placeEdit = CreateWindowExW(WS_EX_CLIENTEDGE, L"EDIT", L"",
            WS_CHILD | WS_VISIBLE | ES_AUTOHSCROLL | WS_TABSTOP,
            72, 34, 380, 26, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_PLACE_EDIT)), cs->hInstance, nullptr);
        st->searchBtn = CreateWindowExW(0, L"BUTTON", L"Szukaj",
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
            460, 34, 100, 26, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_SEARCH_BTN)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC", L"Wyniki:", WS_CHILD | WS_VISIBLE,
            12, 70, 60, 18, hwnd, nullptr, cs->hInstance, nullptr);
        st->resultsList = CreateWindowExW(WS_EX_CLIENTEDGE, L"LISTBOX", L"",
            WS_CHILD | WS_VISIBLE | WS_VSCROLL | LBS_NOTIFY | WS_TABSTOP,
            12, 90, 548, 140, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_RESULTS)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC",
            L"Rozmiar (promień → prostokątny bbox, nie koło):",
            WS_CHILD | WS_VISIBLE, 12, 240, 360, 18, hwnd, nullptr, cs->hInstance, nullptr);
        st->areaCombo = CreateWindowExW(0, L"COMBOBOX", L"",
            WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_TABSTOP,
            12, 260, 320, 200, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_AREA_COMBO)), cs->hInstance, nullptr);
        for (int i = 0; i < osm_net::kAreaSizeChoiceCount; ++i) {
            SendMessageW(st->areaCombo, CB_ADDSTRING, 0,
                reinterpret_cast<LPARAM>(osm_net::kAreaSizeChoices[i].labelUi));
        }
        SendMessageW(st->areaCombo, CB_SETCURSEL, osm_net::kDefaultAreaSizeIndex, 0);

        st->forceCheck = CreateWindowExW(0, L"BUTTON", L"Wymuś ponowne pobranie (Ignore cache)",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX | WS_TABSTOP,
            340, 262, 220, 22, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_FORCE_CHECK)), cs->hInstance, nullptr);

        st->downloadBtn = CreateWindowExW(0, L"BUTTON", L"Pobierz i zaimportuj",
            WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON | WS_TABSTOP,
            12, 300, 180, 30, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_DOWNLOAD_BTN)), cs->hInstance, nullptr);
        st->cancelBtn = CreateWindowExW(0, L"BUTTON", L"Zamknij",
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON | WS_TABSTOP,
            200, 300, 140, 30, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_CANCEL_BTN)), cs->hInstance, nullptr);

        st->statusLabel = CreateWindowExW(0, L"STATIC", L"Gotowe — Szukaj tylko po kliknięciu (nie przy pisaniu).",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            12, 340, 548, 40, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_STATUS)), cs->hInstance, nullptr);

        CreateWindowExW(0, L"STATIC",
            L"Dane: © OpenStreetMap (ODbL) · geokodowanie Nominatim · Overpass. Cache: data/cache/",
            WS_CHILD | WS_VISIBLE, 12, 385, 548, 36, hwnd,
            reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_ATTRIB)), cs->hInstance, nullptr);

        return 0;
    }

    DialogState* st = stateFromHwnd(hwnd);
    if (st == nullptr && msg != WM_DESTROY) {
        return DefWindowProcW(hwnd, msg, wParam, lParam);
    }

    switch (msg) {
        case WM_COMMAND: {
            const int id = LOWORD(wParam);
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
                    SendMessageW(st->resultsList, LB_ADDSTRING, 0,
                        reinterpret_cast<LPARAM>(utf8ToWide(hit.displayName).c_str()));
                }
                SendMessageW(st->resultsList, LB_SETCURSEL, 0, 0);
                std::wstring status = done->result.fromCache
                    ? L"Wyniki z cache. Wybierz pozycję i rozmiar obszaru."
                    : L"Znaleziono wyniki. Wybierz pozycję i rozmiar obszaru (prostokąt).";
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
                std::wstring status = done->result.fromCache
                    ? L"Użyto cache mapy. Zamykanie — import…"
                    : L"Pobrano. Zamykanie — import…";
                st->setStatus(status);
                // Close dialog so MainWindow can import on UI thread.
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
        registered = true;
    }

    RECT ownerRect {};
    if (owner != nullptr) {
        GetWindowRect(owner, &ownerRect);
    }
    const int width = 590;
    const int height = 470;
    const int x = owner != nullptr
        ? ownerRect.left + std::max(0L, (ownerRect.right - ownerRect.left - width) / 2)
        : CW_USEDEFAULT;
    const int y = owner != nullptr
        ? ownerRect.top + std::max(0L, (ownerRect.bottom - ownerRect.top - height) / 2)
        : CW_USEDEFAULT;

    HWND hwnd = CreateWindowExW(
        WS_EX_DLGMODALFRAME,
        L"EasyRiderMapDownloadDialog",
        L"Szukaj miejsca i pobierz mapę (ograniczony obszar)",
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

    // Ensure worker finished even if destroy raced.
    state.cancelFlag = true;
    state.joinWorker();
    return state.out;
}

} // namespace gui
