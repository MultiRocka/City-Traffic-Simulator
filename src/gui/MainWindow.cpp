#include "gui/MainWindow.h"
#include "gui/MapDownloadDialog.h"
#include <sstream>
#include <iomanip>
#include <cmath>
#include <commctrl.h>
#include <commdlg.h>
#include <windowsx.h>

namespace gui {

namespace {
    constexpr int IDC_START = 101;
    constexpr int IDC_PAUSE = 102;
    constexpr int IDC_STOP = 103;
    constexpr int IDC_STATISTICS = 104;
    constexpr int IDC_EVENTLOG = 105;
    constexpr int IDC_MAPVIEW = 106;
    constexpr int IDC_RESET = 108;
    constexpr int IDC_RESUME = 109;
    constexpr int IDC_STATUS = 110;
    constexpr int IDC_LOAD_OSM = 111;
    constexpr int IDC_DEMO_CITY = 112;
    constexpr int IDC_FIT_MAP = 113;
    constexpr int IDC_SEARCH_PLACE = 116;
    constexpr int IDC_CONTROLS_GROUP = 114;
    constexpr int IDC_LEGEND_HINT = 115;

    constexpr int IDC_VEHICLE_RATE = 201;
    constexpr int IDC_PED_TEMPO = 202;
    constexpr int IDC_TIME_MUL = 203;
    constexpr int IDC_APPLY_TEMPO = 204;

    constexpr int IDC_BLOCK_ROAD = 211;
    constexpr int IDC_BLOCK_LANE = 212;
    constexpr int IDC_BLOCK_DUR = 213;
    constexpr int IDC_ADD_BLOCK = 214;
    constexpr int IDC_CLEAR_BLOCKS = 215;

    constexpr int IDC_RANDOM_ENABLE = 221;
    constexpr int IDC_RANDOM_INTERVAL = 222;
    constexpr int IDC_RANDOM_DURATION = 223;
    constexpr int IDC_RANDOM_SEED = 224;
    constexpr int IDC_APPLY_RANDOM = 225;

    constexpr COLORREF kChromeBg = RGB(232, 236, 241);
    constexpr COLORREF kActionBarBg = RGB(245, 247, 250);
    constexpr COLORREF kStatusBg = RGB(238, 241, 245);
    constexpr COLORREF kTextPrimary = RGB(28, 32, 38);
    constexpr COLORREF kTextMuted = RGB(70, 78, 90);

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

    MainWindow* getWindowInstance(HWND hwnd) {
        return reinterpret_cast<MainWindow*>(GetWindowLongPtrW(hwnd, GWLP_USERDATA));
    }

    void setWindowInstance(HWND hwnd, MainWindow* instance) {
        SetWindowLongPtrW(hwnd, GWLP_USERDATA, reinterpret_cast<LONG_PTR>(instance));
    }

    void setEditDouble(HWND edit, double value, int precision = 2) {
        std::wostringstream stream;
        stream << std::fixed << std::setprecision(precision) << value;
        SetWindowTextW(edit, stream.str().c_str());
    }

    void setEditInt(HWND edit, int value) {
        std::wostringstream stream;
        stream << value;
        SetWindowTextW(edit, stream.str().c_str());
    }

    void enableWindow(HWND hwnd, bool enabled) {
        if (hwnd != nullptr) {
            EnableWindow(hwnd, enabled ? TRUE : FALSE);
        }
    }

    std::wstring basenameWide(const std::string& path) {
        if (path.empty()) {
            return L"";
        }
        const auto slash = path.find_last_of("/\\");
        const std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
        return utf8ToWide(base);
    }
}

MainWindow::MainWindow()
    : controlPanel(session),
      statisticsPanel(session),
      eventLogPanel(session),
      instanceHandle(nullptr),
      windowHandle(nullptr),
      mapViewHandle(nullptr),
      statusBarHandle(nullptr),
      legendHintHandle(nullptr),
      statisticsHandle(nullptr),
      controlsGroupHandle(nullptr),
      eventLogHandle(nullptr),
      startButtonHandle(nullptr),
      pauseButtonHandle(nullptr),
      resumeButtonHandle(nullptr),
      stopButtonHandle(nullptr),
      resetButtonHandle(nullptr),
      loadOsmButtonHandle(nullptr),
      searchPlaceButtonHandle(nullptr),
      demoCityButtonHandle(nullptr),
      fitMapButtonHandle(nullptr),
      tempoSectionLabel(nullptr),
      vehicleRateLabel(nullptr),
      vehicleRateEdit(nullptr),
      pedTempoLabel(nullptr),
      pedTempoEdit(nullptr),
      timeMulLabel(nullptr),
      timeMulEdit(nullptr),
      applyTempoButton(nullptr),
      blockSectionLabel(nullptr),
      blockRoadLabel(nullptr),
      blockRoadEdit(nullptr),
      blockLaneLabel(nullptr),
      blockLaneEdit(nullptr),
      blockDurLabel(nullptr),
      blockDurEdit(nullptr),
      addBlockButton(nullptr),
      clearBlocksButton(nullptr),
      randomSectionLabel(nullptr),
      randomEnableCheck(nullptr),
      randomIntervalLabel(nullptr),
      randomIntervalEdit(nullptr),
      randomDurationLabel(nullptr),
      randomDurationEdit(nullptr),
      randomSeedLabel(nullptr),
      randomSeedEdit(nullptr),
      applyRandomButton(nullptr),
      uiFont(nullptr),
      uiFontBold(nullptr),
      chromeBrush(nullptr),
      actionBarBrush(nullptr),
      statusBrush(nullptr),
      dpiScale(1.0),
      running(false),
      statsRefreshCounter(0) {
}

MainWindow::~MainWindow() {
    if (uiFont != nullptr) {
        DeleteObject(uiFont);
        uiFont = nullptr;
    }
    if (uiFontBold != nullptr) {
        DeleteObject(uiFontBold);
        uiFontBold = nullptr;
    }
    if (chromeBrush != nullptr) {
        DeleteObject(chromeBrush);
        chromeBrush = nullptr;
    }
    if (actionBarBrush != nullptr) {
        DeleteObject(actionBarBrush);
        actionBarBrush = nullptr;
    }
    if (statusBrush != nullptr) {
        DeleteObject(statusBrush);
        statusBrush = nullptr;
    }
}

bool MainWindow::initialize(HINSTANCE hInstance) {
    instanceHandle = hInstance;

    INITCOMMONCONTROLSEX icc {};
    icc.dwSize = sizeof(icc);
    icc.dwICC = ICC_STANDARD_CLASSES;
    InitCommonControlsEx(&icc);

    WNDCLASSEXW windowClass {};
    windowClass.cbSize = sizeof(WNDCLASSEXW);
    windowClass.style = CS_HREDRAW | CS_VREDRAW;
    windowClass.lpfnWndProc = MainWindow::windowProcedure;
    windowClass.hInstance = hInstance;
    windowClass.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    windowClass.hbrBackground = nullptr;
    windowClass.lpszClassName = L"EasyRiderMainWindow";

    if (RegisterClassExW(&windowClass) == 0) {
        return false;
    }

    WNDCLASSEXW mapClass {};
    mapClass.cbSize = sizeof(WNDCLASSEXW);
    mapClass.style = 0;
    mapClass.lpfnWndProc = MainWindow::mapViewProcedure;
    mapClass.hInstance = hInstance;
    mapClass.hCursor = LoadCursorW(nullptr, IDC_HAND);
    mapClass.hbrBackground = nullptr;
    mapClass.lpszClassName = L"EasyRiderMapView";

    if (RegisterClassExW(&mapClass) == 0) {
        return false;
    }

    windowHandle = CreateWindowExW(
        0,
        L"EasyRiderMainWindow",
        L"Easy Rider — Symulator ruchu",
        WS_OVERLAPPEDWINDOW | WS_CLIPCHILDREN,
        CW_USEDEFAULT,
        CW_USEDEFAULT,
        windowWidth,
        windowHeight,
        nullptr,
        nullptr,
        hInstance,
        this
    );

    if (windowHandle == nullptr) {
        return false;
    }

    HDC screenDc = GetDC(windowHandle);
    const int dpiY = screenDc != nullptr ? GetDeviceCaps(screenDc, LOGPIXELSY) : 96;
    if (screenDc != nullptr) {
        ReleaseDC(windowHandle, screenDc);
    }
    dpiScale = static_cast<double>(dpiY) / 96.0;
    if (dpiScale < 0.5) {
        dpiScale = 1.0;
    }

    const int fontPx = -MulDiv(10, dpiY, 72);
    const int fontBoldPx = -MulDiv(11, dpiY, 72);

    uiFont = CreateFontW(
        fontPx, 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI"
    );
    uiFontBold = CreateFontW(
        fontBoldPx, 0, 0, 0, FW_SEMIBOLD, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        CLEARTYPE_QUALITY, DEFAULT_PITCH | FF_DONTCARE, L"Segoe UI"
    );

    chromeBrush = CreateSolidBrush(kChromeBg);
    actionBarBrush = CreateSolidBrush(kActionBarBg);
    statusBrush = CreateSolidBrush(kStatusBg);

    createChildWindows();
    ShowWindow(windowHandle, SW_SHOW);
    UpdateWindow(windowHandle);
    running = true;
    return true;
}

void MainWindow::createChildWindows() {
    auto makeStatic = [&](const wchar_t* text, int id, DWORD extraStyle = 0) -> HWND {
        return CreateWindowExW(
            0, L"STATIC", text,
            WS_CHILD | WS_VISIBLE | SS_LEFT | extraStyle,
            0, 0, 10, 10,
            windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), instanceHandle, nullptr
        );
    };
    auto makeEdit = [&](int id) -> HWND {
        return CreateWindowExW(
            WS_EX_CLIENTEDGE, L"EDIT", L"",
            WS_CHILD | WS_VISIBLE | ES_LEFT | ES_AUTOHSCROLL,
            0, 0, 10, 10,
            windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), instanceHandle, nullptr
        );
    };
    auto makeButton = [&](const wchar_t* text, int id) -> HWND {
        return CreateWindowExW(
            0, L"BUTTON", text,
            WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
            0, 0, 10, 10,
            windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(id)), instanceHandle, nullptr
        );
    };

    statusBarHandle = makeStatic(L"", IDC_STATUS, SS_CENTERIMAGE);
    legendHintHandle = makeStatic(
        L"Mapa: kółko = zoom · przeciągnij = przesuń · „Dopasuj mapę” = cały obszar",
        IDC_LEGEND_HINT,
        SS_CENTERIMAGE
    );

    statisticsHandle = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"EDIT", L"Statystyki",
        WS_CHILD | WS_VISIBLE | ES_LEFT | ES_MULTILINE | ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
        0, 0, 10, 10,
        windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_STATISTICS)), instanceHandle, nullptr
    );

    controlsGroupHandle = CreateWindowExW(
        0, L"BUTTON", L"Parametry",
        WS_CHILD | WS_VISIBLE | BS_GROUPBOX,
        0, 0, 10, 10,
        windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_CONTROLS_GROUP)), instanceHandle, nullptr
    );

    eventLogHandle = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"EDIT", L"",
        WS_CHILD | WS_VISIBLE | ES_LEFT | ES_MULTILINE | ES_READONLY | WS_VSCROLL | ES_AUTOVSCROLL,
        0, 0, 10, 10,
        windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_EVENTLOG)), instanceHandle, nullptr
    );

    mapViewHandle = CreateWindowExW(
        WS_EX_CLIENTEDGE, L"EasyRiderMapView", L"",
        WS_CHILD | WS_VISIBLE,
        0, 0, 10, 10,
        windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_MAPVIEW)), instanceHandle, nullptr
    );

    startButtonHandle = makeButton(L"Start", IDC_START);
    pauseButtonHandle = makeButton(L"Pauza", IDC_PAUSE);
    resumeButtonHandle = makeButton(L"Wznów", IDC_RESUME);
    stopButtonHandle = makeButton(L"Stop", IDC_STOP);
    resetButtonHandle = makeButton(L"Reset", IDC_RESET);
    loadOsmButtonHandle = makeButton(L"Wczytaj OSM…", IDC_LOAD_OSM);
    searchPlaceButtonHandle = makeButton(L"Szukaj miejsca…", IDC_SEARCH_PLACE);
    demoCityButtonHandle = makeButton(L"Miasto demo", IDC_DEMO_CITY);
    fitMapButtonHandle = makeButton(L"Dopasuj mapę", IDC_FIT_MAP);

    tempoSectionLabel = makeStatic(L"Tempo", IDC_VEHICLE_RATE + 40);
    vehicleRateLabel = makeStatic(L"Pojazdy ×", IDC_VEHICLE_RATE + 50);
    vehicleRateEdit = makeEdit(IDC_VEHICLE_RATE);
    pedTempoLabel = makeStatic(L"Piesi ×", IDC_PED_TEMPO + 50);
    pedTempoEdit = makeEdit(IDC_PED_TEMPO);
    timeMulLabel = makeStatic(L"Czas ×", IDC_TIME_MUL + 50);
    timeMulEdit = makeEdit(IDC_TIME_MUL);
    applyTempoButton = makeButton(L"Zastosuj tempo", IDC_APPLY_TEMPO);

    blockSectionLabel = makeStatic(L"Blokady pasów", IDC_BLOCK_ROAD + 40);
    blockRoadLabel = makeStatic(L"Id drogi", IDC_BLOCK_ROAD + 50);
    blockRoadEdit = makeEdit(IDC_BLOCK_ROAD);
    blockLaneLabel = makeStatic(L"Pas", IDC_BLOCK_LANE + 50);
    blockLaneEdit = makeEdit(IDC_BLOCK_LANE);
    blockDurLabel = makeStatic(L"Czas [s]", IDC_BLOCK_DUR + 50);
    blockDurEdit = makeEdit(IDC_BLOCK_DUR);
    addBlockButton = makeButton(L"Dodaj blokadę", IDC_ADD_BLOCK);
    clearBlocksButton = makeButton(L"Wyczyść blokady", IDC_CLEAR_BLOCKS);

    randomSectionLabel = makeStatic(L"Losowe blokady", IDC_RANDOM_ENABLE + 40);
    randomEnableCheck = CreateWindowExW(
        0, L"BUTTON", L"Włącz losowe",
        WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
        0, 0, 10, 10,
        windowHandle, reinterpret_cast<HMENU>(static_cast<INT_PTR>(IDC_RANDOM_ENABLE)), instanceHandle, nullptr
    );
    randomIntervalLabel = makeStatic(L"Int. [s]", IDC_RANDOM_INTERVAL + 50);
    randomIntervalEdit = makeEdit(IDC_RANDOM_INTERVAL);
    randomDurationLabel = makeStatic(L"Dur. [s]", IDC_RANDOM_DURATION + 50);
    randomDurationEdit = makeEdit(IDC_RANDOM_DURATION);
    randomSeedLabel = makeStatic(L"Seed", IDC_RANDOM_SEED + 50);
    randomSeedEdit = makeEdit(IDC_RANDOM_SEED);
    applyRandomButton = makeButton(L"Zastosuj losowe", IDC_APPLY_RANDOM);

    applyFontToChildren();
    syncParameterEditsFromSession();
    setWindowInstance(mapViewHandle, this);
    updateControlEnabledState();
    setStatusMessage(L"Gotowe. Łódź, żółć, przejście, prędkość, skrzyżowanie");
}

void MainWindow::applyFontToChildren() {
    HWND fonts[] = {
        statusBarHandle, legendHintHandle, statisticsHandle, controlsGroupHandle, eventLogHandle,
        startButtonHandle, pauseButtonHandle, resumeButtonHandle, stopButtonHandle, resetButtonHandle,
        loadOsmButtonHandle, searchPlaceButtonHandle, demoCityButtonHandle, fitMapButtonHandle,
        tempoSectionLabel, vehicleRateLabel, vehicleRateEdit, pedTempoLabel, pedTempoEdit,
        timeMulLabel, timeMulEdit, applyTempoButton,
        blockSectionLabel, blockRoadLabel, blockRoadEdit, blockLaneLabel, blockLaneEdit,
        blockDurLabel, blockDurEdit, addBlockButton, clearBlocksButton,
        randomSectionLabel, randomEnableCheck, randomIntervalLabel, randomIntervalEdit,
        randomDurationLabel, randomDurationEdit, randomSeedLabel, randomSeedEdit, applyRandomButton
    };
    if (uiFont != nullptr) {
        for (HWND hwnd : fonts) {
            if (hwnd != nullptr) {
                SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(uiFont), TRUE);
            }
        }
    }
    if (uiFontBold != nullptr) {
        HWND boldTargets[] = {tempoSectionLabel, blockSectionLabel, randomSectionLabel, controlsGroupHandle};
        for (HWND hwnd : boldTargets) {
            if (hwnd != nullptr) {
                SendMessageW(hwnd, WM_SETFONT, reinterpret_cast<WPARAM>(uiFontBold), TRUE);
            }
        }
    }
}

void MainWindow::syncParameterEditsFromSession() {
    setEditDouble(vehicleRateEdit, controlPanel.getVehicleSpawnRate());
    setEditDouble(pedTempoEdit, controlPanel.getPedestrianRequestTempo());
    setEditDouble(timeMulEdit, controlPanel.getTimeMultiplier());
    setEditInt(blockRoadEdit, 1);
    setEditInt(blockLaneEdit, 0);
    setEditDouble(blockDurEdit, 12.0, 1);

    const RandomBlockConfig cfg = controlPanel.getRandomBlockConfig();
    SendMessageW(randomEnableCheck, BM_SETCHECK, cfg.enabled ? BST_CHECKED : BST_UNCHECKED, 0);
    setEditDouble(randomIntervalEdit, cfg.minInterval, 1);
    setEditDouble(randomDurationEdit, cfg.minDuration, 1);
    setEditInt(randomSeedEdit, static_cast<int>(cfg.seed));
}

void MainWindow::paintChrome(HWND hwnd, HDC hdc) {
    RECT client {};
    GetClientRect(hwnd, &client);

    if (chromeBrush != nullptr) {
        FillRect(hdc, &client, chromeBrush);
    }

    RECT action {0, 0, client.right, layoutMetrics.actionBarHeight};
    if (actionBarBrush != nullptr) {
        FillRect(hdc, &action, actionBarBrush);
    }

    RECT hint {0, layoutMetrics.hintY, client.right, layoutMetrics.hintY + layoutMetrics.hintStripHeight};
    if (actionBarBrush != nullptr) {
        FillRect(hdc, &hint, actionBarBrush);
    }

    HPEN sepPen = CreatePen(PS_SOLID, 1, RGB(200, 206, 214));
    HGDIOBJ oldPen = SelectObject(hdc, sepPen);
    MoveToEx(hdc, 0, layoutMetrics.actionBarHeight - 1, nullptr);
    LineTo(hdc, client.right, layoutMetrics.actionBarHeight - 1);
    MoveToEx(hdc, 0, layoutMetrics.hintY + layoutMetrics.hintStripHeight - 1, nullptr);
    LineTo(hdc, client.right, layoutMetrics.hintY + layoutMetrics.hintStripHeight - 1);
    MoveToEx(hdc, 0, layoutMetrics.statusY, nullptr);
    LineTo(hdc, client.right, layoutMetrics.statusY);
    SelectObject(hdc, oldPen);
    DeleteObject(sepPen);

    RECT status {0, layoutMetrics.statusY, client.right, client.bottom};
    if (statusBrush != nullptr) {
        FillRect(hdc, &status, statusBrush);
    }
}

void MainWindow::layoutChildWindows(int clientWidth, int clientHeight) {
    layoutMetrics.compute(clientWidth, clientHeight, dpiScale);
    const auto& m = layoutMetrics;

    MoveWindow(statusBarHandle, m.pad, m.statusY + 2, clientWidth - m.pad * 2, m.statusBarHeight - 4, TRUE);
    MoveWindow(legendHintHandle, m.pad, m.hintY + 1, clientWidth - m.pad * 2, m.hintStripHeight - 2, TRUE);

    MoveWindow(mapViewHandle, m.mapX, m.mapY, m.mapWidth, m.mapHeight, TRUE);

    const int rightX = m.rightX;
    const int rightW = m.rightWidth;
    int y = m.rightY;

    MoveWindow(statisticsHandle, rightX, y, rightW, m.statsHeight, TRUE);
    y += m.statsHeight + m.gap;

    MoveWindow(controlsGroupHandle, rightX, y, rightW, m.controlsHeight, TRUE);

    const int cx = rightX + m.S(12);
    const int cw = rightW - m.S(24);
    const int rowH = m.S(24);
    const int labelW = m.S(100);
    const int editW = m.S(64);
    const int rowGap = m.S(26);
    int cy = y + m.S(20);

    MoveWindow(tempoSectionLabel, cx, cy, cw, m.S(18), TRUE);
    cy += m.S(20);
    MoveWindow(vehicleRateLabel, cx, cy + 3, labelW, m.S(18), TRUE);
    MoveWindow(vehicleRateEdit, cx + labelW, cy, editW, rowH, TRUE);
    cy += rowGap;
    MoveWindow(pedTempoLabel, cx, cy + 3, labelW, m.S(18), TRUE);
    MoveWindow(pedTempoEdit, cx + labelW, cy, editW, rowH, TRUE);
    cy += rowGap;
    MoveWindow(timeMulLabel, cx, cy + 3, labelW, m.S(18), TRUE);
    MoveWindow(timeMulEdit, cx + labelW, cy, editW, rowH, TRUE);
    MoveWindow(applyTempoButton, cx + labelW + editW + m.S(10), cy,
               std::min(m.S(140), cw - labelW - editW - m.S(10)), rowH, TRUE);
    cy += m.S(30);

    MoveWindow(blockSectionLabel, cx, cy, cw, m.S(18), TRUE);
    cy += m.S(20);
    const int fieldW = std::max(m.S(48), (cw - m.S(16)) / 3);
    MoveWindow(blockRoadLabel, cx, cy + 2, fieldW - 4, m.S(16), TRUE);
    MoveWindow(blockLaneLabel, cx + fieldW, cy + 2, fieldW - 4, m.S(16), TRUE);
    MoveWindow(blockDurLabel, cx + fieldW * 2, cy + 2, fieldW - 4, m.S(16), TRUE);
    cy += m.S(18);
    MoveWindow(blockRoadEdit, cx, cy, fieldW - m.S(8), rowH, TRUE);
    MoveWindow(blockLaneEdit, cx + fieldW, cy, fieldW - m.S(8), rowH, TRUE);
    MoveWindow(blockDurEdit, cx + fieldW * 2, cy, fieldW - m.S(8), rowH, TRUE);
    cy += m.S(28);

    const int halfBtn = std::max(m.S(100), (cw - m.S(8)) / 2);
    MoveWindow(addBlockButton, cx, cy, halfBtn, rowH, TRUE);
    MoveWindow(clearBlocksButton, cx + halfBtn + m.S(8), cy, halfBtn, rowH, TRUE);
    cy += m.S(30);

    MoveWindow(randomSectionLabel, cx, cy, cw, m.S(18), TRUE);
    cy += m.S(20);
    MoveWindow(randomEnableCheck, cx, cy, cw, m.S(20), TRUE);
    cy += m.S(22);
    MoveWindow(randomIntervalLabel, cx, cy + 3, m.S(52), m.S(16), TRUE);
    MoveWindow(randomIntervalEdit, cx + m.S(54), cy, m.S(48), rowH, TRUE);
    MoveWindow(randomDurationLabel, cx + m.S(110), cy + 3, m.S(52), m.S(16), TRUE);
    MoveWindow(randomDurationEdit, cx + m.S(164), cy, m.S(48), rowH, TRUE);
    cy += rowGap;
    MoveWindow(randomSeedLabel, cx, cy + 3, m.S(40), m.S(16), TRUE);
    MoveWindow(randomSeedEdit, cx + m.S(42), cy, std::max(m.S(64), cw - m.S(42)), rowH, TRUE);
    cy += rowGap;
    MoveWindow(applyRandomButton, cx, cy, std::min(m.S(150), cw), rowH, TRUE);

    y += m.controlsHeight + m.gap;
    MoveWindow(eventLogHandle, rightX, y, rightW, m.logHeight, TRUE);

    // Action bar — single clear row of primary controls.
    const int btnY = (m.actionBarHeight - m.S(28)) / 2;
    const int btnH = m.S(28);
    int bx = m.pad;
    auto placeBtn = [&](HWND hwnd, int width) {
        MoveWindow(hwnd, bx, btnY, width, btnH, TRUE);
        bx += width + m.S(6);
    };
    placeBtn(startButtonHandle, m.S(68));
    placeBtn(pauseButtonHandle, m.S(68));
    placeBtn(resumeButtonHandle, m.S(68));
    placeBtn(stopButtonHandle, m.S(64));
    placeBtn(resetButtonHandle, m.S(68));
    bx += m.S(12);
    placeBtn(loadOsmButtonHandle, m.S(112));
    placeBtn(searchPlaceButtonHandle, m.S(128));
    placeBtn(demoCityButtonHandle, m.S(100));
    placeBtn(fitMapButtonHandle, m.S(108));

    Viewport viewport;
    viewport.x = 0;
    viewport.y = 0;
    viewport.width = m.mapWidth;
    viewport.height = m.mapHeight;
    mapView.setViewport(viewport);
}

void MainWindow::updateControlEnabledState() {
    const bool isRunning = controlPanel.isRunning();
    const bool isPaused = controlPanel.isPaused();

    enableWindow(startButtonHandle, !isRunning || isPaused);
    enableWindow(pauseButtonHandle, isRunning && !isPaused);
    enableWindow(resumeButtonHandle, isRunning && isPaused);
    enableWindow(stopButtonHandle, isRunning);
    enableWindow(resetButtonHandle, true);
    enableWindow(loadOsmButtonHandle, true);
    enableWindow(searchPlaceButtonHandle, true);
    enableWindow(demoCityButtonHandle, true);
    enableWindow(fitMapButtonHandle, true);
}

std::wstring MainWindow::mapLabel() const {
    switch (session.getLoadedScenario()) {
        case SessionScenario::Osm: {
            const std::wstring file = basenameWide(session.getOsmSourcePath());
            if (!file.empty()) {
                return L"OSM: " + file;
            }
            return L"OSM";
        }
        case SessionScenario::IntersectionTest:
            return L"Intersection Test";
        case SessionScenario::Demo:
        default:
            return L"Miasto demo";
    }
}

std::wstring MainWindow::appStateLabel() const {
    if (!controlPanel.isRunning()) {
        return L"STOP";
    }
    if (controlPanel.isPaused()) {
        return L"PAUZA";
    }
    return L"DZIAŁA";
}

void MainWindow::loadDemoScenario() {
    session.loadDemoScenario();
    KillTimer(windowHandle, renderTimerId);
    syncParameterEditsFromSession();
    fitMapToView();
    refreshPanels();
    setStatusMessage(L"Załadowano Miasto demo.");
}

void MainWindow::loadIntersectionTestScenario() {
    session.loadIntersectionTestScenario();
    KillTimer(windowHandle, renderTimerId);
    syncParameterEditsFromSession();
    fitMapToView();
    refreshPanels();
    setStatusMessage(L"Załadowano scenariusz intersection-test.");
}

void MainWindow::fitMapToView() {
    RenderSnapshot snapshot = session.buildDisplaySnapshot();
    mapView.fitToSnapshot(snapshot);
    InvalidateRect(mapViewHandle, nullptr, FALSE);
}

void MainWindow::showImportSummary(const OsmScenarioResult& result) {
    const std::wstring summary = utf8ToWide(SimulationSession::formatImportSummary(result));
    MessageBoxW(windowHandle, summary.c_str(), L"Import OSM", MB_OK | MB_ICONINFORMATION);
}

void MainWindow::importOsmPath(const std::string& path, const std::wstring& statusPrefix) {
    const OsmScenarioResult result = session.loadOsmFromFile(path);
    KillTimer(windowHandle, renderTimerId);

    if (!result.ok) {
        const std::wstring error = utf8ToWide(
            result.error.empty() ? std::string("Import OSM nie powiódł się.") : result.error
        );
        MessageBoxW(windowHandle, error.c_str(), L"Import OSM — błąd", MB_OK | MB_ICONERROR);
        setStatusMessage(L"Import OSM nieudany — poprzedni scenariusz zachowany.");
        refreshPanels();
        return;
    }

    syncParameterEditsFromSession();
    fitMapToView();
    refreshPanels();
    showImportSummary(result);

    std::wostringstream status;
    if (!statusPrefix.empty()) {
        status << statusPrefix << L" — ";
    }
    status << L"Zaimportowano OSM — "
           << result.report.builtRoadCount << L" dróg, "
           << result.attachedGeneratorCount << L" generatorów. Start uruchamia symulację.";
    if (!result.report.warnings.empty()) {
        status << L" Ostrzeżenia: " << result.report.warnings.size() << L".";
    }
    setStatusMessage(status.str());
}

void MainWindow::onLoadOsmClicked() {
    wchar_t fileBuffer[MAX_PATH] = L"";
    OPENFILENAMEW ofn {};
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = windowHandle;
    ofn.lpstrFilter =
        L"Overpass JSON (*.json;*.overpass.json)\0*.json;*.overpass.json\0"
        L"All files (*.*)\0*.*\0";
    ofn.lpstrFile = fileBuffer;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = L"Wczytaj OSM Overpass JSON";
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER;

    if (GetOpenFileNameW(&ofn) != TRUE) {
        return;
    }

    const std::string path = wideToUtf8(fileBuffer);
    if (path.empty()) {
        MessageBoxW(windowHandle, L"Nie można odczytać wybranej ścieżki.", L"Import OSM", MB_OK | MB_ICONERROR);
        return;
    }

    importOsmPath(path, L"");
}

void MainWindow::onSearchPlaceClicked() {
    const MapDownloadDialogResult downloaded = MapDownloadDialog::run(windowHandle);
    if (!downloaded.accepted || downloaded.cachedOverpassPath.empty()) {
        return;
    }

    std::wstring prefix = L"Pobrano";
    if (!downloaded.placeLabel.empty()) {
        prefix += L" („" + utf8ToWide(downloaded.placeLabel) + L"”)";
    }
    importOsmPath(downloaded.cachedOverpassPath, prefix);
}

void MainWindow::onDemoCityClicked() {
    loadDemoScenario();
    setStatusMessage(L"Powrót do Miasta demo — Start uruchamia symulację.");
}

void MainWindow::setStatusMessage(const std::wstring& message) {
    lastUserStatus = message;
    refreshStatusBar();
}

void MainWindow::refreshStatusBar() {
    std::wostringstream stream;
    stream << L"Mapa: " << mapLabel()
           << L"  ·  Stan: " << appStateLabel()
           << L"  ·  " << lastUserStatus;
    SetWindowTextW(statusBarHandle, stream.str().c_str());
}

bool MainWindow::readDoubleFromEdit(HWND edit, double& outValue) const {
    wchar_t buffer[64] {};
    GetWindowTextW(edit, buffer, 64);
    try {
        outValue = std::stod(buffer);
    } catch (...) {
        return false;
    }
    return std::isfinite(outValue);
}

bool MainWindow::readIntFromEdit(HWND edit, int& outValue) const {
    wchar_t buffer[64] {};
    GetWindowTextW(edit, buffer, 64);
    try {
        outValue = std::stoi(buffer);
    } catch (...) {
        return false;
    }
    return true;
}

void MainWindow::applyTempoParameters() {
    double vehicleRate = 0.0;
    double pedTempo = 0.0;
    double timeMul = 0.0;
    if (!readDoubleFromEdit(vehicleRateEdit, vehicleRate) ||
        !readDoubleFromEdit(pedTempoEdit, pedTempo) ||
        !readDoubleFromEdit(timeMulEdit, timeMul)) {
        setStatusMessage(L"Błędne wartości tempa (wymagane liczby).");
        return;
    }

    if (!controlPanel.setVehicleSpawnRate(vehicleRate) ||
        !controlPanel.setPedestrianRequestTempo(pedTempo) ||
        !controlPanel.setTimeMultiplier(timeMul)) {
        setStatusMessage(L"Tempo poza zakresem [0.1, 10].");
        return;
    }

    syncParameterEditsFromSession();
    setStatusMessage(L"Zastosowano tempo pojazdów / pieszych / czasu.");
    refreshPanels();
}

void MainWindow::applyManualBlockage() {
    int roadId = 0;
    int laneIndex = 0;
    double duration = 0.0;
    if (!readIntFromEdit(blockRoadEdit, roadId) ||
        !readIntFromEdit(blockLaneEdit, laneIndex) ||
        !readDoubleFromEdit(blockDurEdit, duration)) {
        setStatusMessage(L"Błędne dane blokady (id drogi, pas, czas s).");
        return;
    }

    const int id = controlPanel.addManualLaneBlockage(roadId, laneIndex, duration);
    if (id < 0) {
        setStatusMessage(L"Blokada odrzucona (geometria, nakładanie lub zakres).");
        return;
    }

    std::wostringstream stream;
    stream << L"Dodano blokadę id=" << id << L" droga " << roadId << L" pas " << laneIndex
           << L" na " << std::fixed << std::setprecision(1) << duration << L" s";
    setStatusMessage(stream.str());
    refreshPanels();
}

void MainWindow::applyRandomBlockage() {
    const bool enabled = SendMessageW(randomEnableCheck, BM_GETCHECK, 0, 0) == BST_CHECKED;
    double interval = 30.0;
    double duration = 12.0;
    int seed = 20261013;
    if (!readDoubleFromEdit(randomIntervalEdit, interval) ||
        !readDoubleFromEdit(randomDurationEdit, duration) ||
        !readIntFromEdit(randomSeedEdit, seed)) {
        setStatusMessage(L"Błędne parametry losowych blokad (int. s, dur. s, seed).");
        return;
    }

    if (!controlPanel.setRandomBlockages(enabled, interval, duration, static_cast<uint32_t>(seed))) {
        setStatusMessage(L"Parametry losowych blokad poza zakresem.");
        return;
    }

    setStatusMessage(enabled
        ? L"Losowe blokady włączone."
        : L"Losowe blokady wyłączone.");
    refreshPanels();
}

void MainWindow::refreshStatisticsText() {
    const StatisticsSnapshot& snapshot = statisticsPanel.getSnapshot();
    const auto& blocks = session.getSimulation().getLaneBlockages();
    int activeBlocks = 0;
    for (const auto& block : blocks) {
        if (block.isActive(session.getSimulation().getTime())) {
            ++activeBlocks;
        }
    }

    std::wostringstream stream;
    stream << L"Statystyki\r\n";
    stream << L"──────────\r\n";
    stream << std::fixed << std::setprecision(1);
    stream << L"Mapa: " << mapLabel() << L"\r\n";
    stream << L"Czas sim: " << snapshot.time << L" s\r\n";
    stream << L"Stan: " << appStateLabel() << L"\r\n";
    stream << L"Czas ×: " << controlPanel.getTimeMultiplier() << L"\r\n\r\n";

    stream << L"Aktywne: " << snapshot.activeVehicles << L"\r\n";
    stream << L"Wygenerowane: " << snapshot.totalVehiclesSeen << L"\r\n";
    stream << L"Ukończone: " << snapshot.finishedVehicles << L"\r\n";
    stream << L"Jadące: " << snapshot.movingVehicles << L"\r\n";
    stream << L"Stojące: " << snapshot.stoppedVehicles << L"\r\n\r\n";

    stream << std::setprecision(2);
    stream << L"Śr. prędkość: " << snapshot.averageSpeed << L" m/s\r\n";
    stream << L"  (" << (snapshot.averageSpeed * 3.6) << L" km/h)\r\n";
    stream << L"Śr. jadących: " << snapshot.averageMovingSpeed << L" m/s\r\n";
    stream << L"Czas postoju: " << snapshot.totalStoppedTime << L" s\r\n";
    stream << L"Max aktywnych: " << snapshot.maxActiveVehicles << L"\r\n";
    stream << L"Blokady pasów: " << activeBlocks << L" aktywne\r\n";
    stream << L"Tempo poj. ×: " << controlPanel.getVehicleSpawnRate() << L"\r\n";
    stream << L"Tempo pieszych ×: " << controlPanel.getPedestrianRequestTempo() << L"\r\n";

    SetWindowTextW(statisticsHandle, stream.str().c_str());
}

void MainWindow::refreshEventLogText() {
    std::wostringstream stream;
    stream << L"Raport importu\r\n";
    stream << L"──────────────\r\n";
    if (session.getLoadedScenario() == SessionScenario::Osm && !session.getLastImportSummary().empty()) {
        const std::wstring summary = utf8ToWide(session.getLastImportSummary());
        std::wstring normalized;
        normalized.reserve(summary.size() * 2);
        for (size_t i = 0; i < summary.size(); ++i) {
            if (summary[i] == L'\n') {
                if (i == 0 || summary[i - 1] != L'\r') {
                    normalized += L"\r\n";
                } else {
                    normalized += L'\n';
                }
            } else {
                normalized += summary[i];
            }
        }
        stream << normalized;
        if (!normalized.empty() && normalized.back() != L'\n') {
            stream << L"\r\n";
        }
    } else {
        stream << L"(brak importu OSM — aktywne Miasto demo)\r\n";
    }

    stream << L"\r\nDziennik zdarzeń\r\n";
    stream << L"────────────────\r\n";

    const auto& events = eventLogPanel.getEvents();
    int startIndex = std::max(0, static_cast<int>(events.size()) - 30);

    for (int i = startIndex; i < static_cast<int>(events.size()); i++) {
        const auto& event = events[i];
        stream << L"[" << std::fixed << std::setprecision(1) << event.time << L"s] ";

        std::wstring type = utf8ToWide(event.type);
        stream << type;

        if (event.vehicleId >= 0) {
            stream << L" v" << event.vehicleId;
        }

        stream << L"\r\n  ";
        const std::string labeled = EventLogPanel::formatReasonLabel(event.reason);
        stream << utf8ToWide(labeled);
        if (labeled != event.reason) {
            stream << L" (" << utf8ToWide(event.reason) << L")";
        }
        stream << L"\r\n";
    }

    if (events.empty()) {
        stream << L"Brak zdarzeń.\r\n";
    }

    SetWindowTextW(eventLogHandle, stream.str().c_str());
}

void MainWindow::refreshPanels() {
    updateControlEnabledState();
    refreshStatisticsText();
    refreshEventLogText();
    refreshStatusBar();
    InvalidateRect(mapViewHandle, nullptr, FALSE);
}

void MainWindow::updateFrame() {
    session.update(SimulationSession::renderDt);
    InvalidateRect(mapViewHandle, nullptr, FALSE);

    statsRefreshCounter++;
    if (statsRefreshCounter >= 10) {
        statsRefreshCounter = 0;
        refreshPanels();
    }
}

void MainWindow::onCommand(int commandId) {
    switch (commandId) {
        case IDC_START:
            if (!controlPanel.isRunning()) {
                controlPanel.start();
                SetTimer(windowHandle, renderTimerId, static_cast<UINT>(1000.0 / SimulationSession::renderHz), nullptr);
                setStatusMessage(L"Symulacja uruchomiona.");
            } else if (controlPanel.isPaused()) {
                controlPanel.resume();
                setStatusMessage(L"Wznowiono.");
            }
            refreshPanels();
            break;
        case IDC_PAUSE:
            if (controlPanel.isRunning() && !controlPanel.isPaused()) {
                controlPanel.pause();
                setStatusMessage(L"Pauza.");
            }
            refreshPanels();
            break;
        case IDC_RESUME:
            if (controlPanel.isRunning() && controlPanel.isPaused()) {
                controlPanel.resume();
                setStatusMessage(L"Wznowiono.");
            }
            refreshPanels();
            break;
        case IDC_STOP:
            controlPanel.stop();
            KillTimer(windowHandle, renderTimerId);
            setStatusMessage(L"Zatrzymano (scenariusz w pamięci).");
            refreshPanels();
            break;
        case IDC_RESET:
            controlPanel.reset();
            KillTimer(windowHandle, renderTimerId);
            syncParameterEditsFromSession();
            fitMapToView();
            if (session.getLoadedScenario() == SessionScenario::Osm) {
                setStatusMessage(L"Reset — mapa OSM zachowana; ruch wyczyszczony.");
            } else {
                setStatusMessage(L"Reset — scenariusz przeładowany.");
            }
            refreshPanels();
            break;
        case IDC_LOAD_OSM:
            onLoadOsmClicked();
            break;
        case IDC_SEARCH_PLACE:
            onSearchPlaceClicked();
            break;
        case IDC_DEMO_CITY:
            onDemoCityClicked();
            break;
        case IDC_FIT_MAP:
            fitMapToView();
            setStatusMessage(L"Mapa dopasowana do widoku.");
            break;
        case IDC_APPLY_TEMPO:
            applyTempoParameters();
            break;
        case IDC_ADD_BLOCK:
            applyManualBlockage();
            break;
        case IDC_CLEAR_BLOCKS:
            controlPanel.clearAllLaneBlockages();
            setStatusMessage(L"Wyczyszczono blokady pasów.");
            refreshPanels();
            break;
        case IDC_APPLY_RANDOM:
            applyRandomBlockage();
            break;
        default:
            break;
    }
}

LRESULT CALLBACK MainWindow::windowProcedure(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    if (message == WM_NCCREATE) {
        auto* createStruct = reinterpret_cast<CREATESTRUCTW*>(lParam);
        setWindowInstance(hwnd, reinterpret_cast<MainWindow*>(createStruct->lpCreateParams));
        return TRUE;
    }

    MainWindow* instance = getWindowInstance(hwnd);
    if (instance != nullptr) {
        return instance->handleMessage(hwnd, message, wParam, lParam);
    }

    return DefWindowProcW(hwnd, message, wParam, lParam);
}

LRESULT CALLBACK MainWindow::mapViewProcedure(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    MainWindow* instance = getWindowInstance(hwnd);
    if (instance != nullptr) {
        return instance->handleMapViewMessage(hwnd, message, wParam, lParam);
    }

    return DefWindowProcW(hwnd, message, wParam, lParam);
}

LRESULT MainWindow::handleMapViewMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_ERASEBKGND:
            return 1;

        case WM_PAINT: {
            PAINTSTRUCT paintStruct;
            HDC deviceContext = BeginPaint(hwnd, &paintStruct);
            RenderSnapshot snapshot = session.buildDisplaySnapshot();
            mapView.paint(hwnd, deviceContext, snapshot);
            EndPaint(hwnd, &paintStruct);
            return 0;
        }

        case WM_MOUSEACTIVATE:
            SetFocus(hwnd);
            return MA_ACTIVATE;

        case WM_LBUTTONDOWN: {
            SetFocus(hwnd);
            SetCapture(hwnd);
            mapView.beginPan(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
            return 0;
        }

        case WM_MOUSEMOVE: {
            if (mapView.isPanning() && (wParam & MK_LBUTTON)) {
                mapView.updatePan(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            return 0;
        }

        case WM_LBUTTONUP: {
            if (mapView.isPanning()) {
                mapView.updatePan(GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam));
                mapView.endPan();
                InvalidateRect(hwnd, nullptr, FALSE);
            }
            if (GetCapture() == hwnd) {
                ReleaseCapture();
            }
            return 0;
        }

        case WM_CAPTURECHANGED:
            mapView.endPan();
            return 0;

        case WM_MOUSEWHEEL: {
            POINT pt {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
            ScreenToClient(hwnd, &pt);
            const int delta = GET_WHEEL_DELTA_WPARAM(wParam);
            const double factor = (delta > 0) ? kDefaultZoomFactor : (1.0 / kDefaultZoomFactor);
            mapView.zoomAt(pt.x, pt.y, factor);
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        }

        default:
            break;
    }

    return DefWindowProcW(hwnd, message, wParam, lParam);
}

LRESULT MainWindow::handleMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam) {
    switch (message) {
        case WM_ERASEBKGND:
            return 1;
        case WM_PAINT: {
            PAINTSTRUCT ps {};
            HDC hdc = BeginPaint(hwnd, &ps);
            paintChrome(hwnd, hdc);
            EndPaint(hwnd, &ps);
            return 0;
        }
        case WM_GETMINMAXINFO: {
            auto* info = reinterpret_cast<MINMAXINFO*>(lParam);
            info->ptMinTrackSize.x = static_cast<LONG>(
                std::lround(GuiLayoutMetrics::kMinClientWidth * dpiScale));
            info->ptMinTrackSize.y = static_cast<LONG>(
                std::lround(GuiLayoutMetrics::kMinClientHeight * dpiScale));
            return 0;
        }
        case WM_SIZE: {
            const int clientWidth = LOWORD(lParam);
            const int clientHeight = HIWORD(lParam);
            layoutChildWindows(clientWidth, clientHeight);
            refreshPanels();
            InvalidateRect(hwnd, nullptr, FALSE);
            return 0;
        }
        case WM_CTLCOLORSTATIC: {
            HDC hdc = reinterpret_cast<HDC>(wParam);
            HWND child = reinterpret_cast<HWND>(lParam);
            SetBkMode(hdc, TRANSPARENT);
            if (child == statusBarHandle) {
                SetTextColor(hdc, kTextMuted);
                return reinterpret_cast<LRESULT>(statusBrush != nullptr
                    ? statusBrush
                    : GetSysColorBrush(COLOR_3DFACE));
            }
            if (child == legendHintHandle) {
                SetTextColor(hdc, kTextMuted);
                return reinterpret_cast<LRESULT>(actionBarBrush != nullptr
                    ? actionBarBrush
                    : GetSysColorBrush(COLOR_3DFACE));
            }
            SetTextColor(hdc, kTextPrimary);
            return reinterpret_cast<LRESULT>(chromeBrush != nullptr
                ? chromeBrush
                : GetSysColorBrush(COLOR_3DFACE));
        }
        case WM_MOUSEWHEEL: {
            POINT pt {GET_X_LPARAM(lParam), GET_Y_LPARAM(lParam)};
            RECT mapRect {};
            if (mapViewHandle != nullptr && GetWindowRect(mapViewHandle, &mapRect)
                && PtInRect(&mapRect, pt)) {
                return SendMessageW(mapViewHandle, WM_MOUSEWHEEL, wParam, lParam);
            }
            break;
        }
        case WM_COMMAND:
            onCommand(LOWORD(wParam));
            return 0;
        case WM_TIMER:
            if (wParam == renderTimerId) {
                updateFrame();
            }
            return 0;
        case WM_DESTROY:
            if (controlPanel.isRunning()) {
                controlPanel.stop();
            }
            session.exportResults();
            KillTimer(hwnd, renderTimerId);
            running = false;
            PostQuitMessage(0);
            return 0;
        default:
            break;
    }

    return DefWindowProcW(hwnd, message, wParam, lParam);
}

int MainWindow::run() {
    MSG message {};
    while (running && GetMessageW(&message, nullptr, 0, 0) > 0) {
        TranslateMessage(&message);
        DispatchMessageW(&message);
    }

    return static_cast<int>(message.wParam);
}

} // namespace gui
