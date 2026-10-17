#pragma once

#ifndef UNICODE
#define UNICODE
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include <string>
#include "ui/SimulationSession.h"
#include "ui/ControlPanel.h"
#include "ui/StatisticsPanel.h"
#include "ui/EventLogPanel.h"
#include "gui/MapView.h"
#include "gui/GuiLayout.h"

namespace gui {

class MainWindow {
public:
    MainWindow();
    ~MainWindow();

    bool initialize(HINSTANCE instanceHandle);
    void loadDemoScenario();
    void loadIntersectionTestScenario();
    int run();

private:
    static constexpr int windowWidth = 1440;
    static constexpr int windowHeight = 900;
    static constexpr UINT_PTR renderTimerId = 1;

    SimulationSession session;
    ::ControlPanel controlPanel;
    ::StatisticsPanel statisticsPanel;
    ::EventLogPanel eventLogPanel;
    MapView mapView;
    GuiLayoutMetrics layoutMetrics;

    HINSTANCE instanceHandle;
    HWND windowHandle;
    HWND mapViewHandle;
    HWND statusBarHandle;
    HWND legendHintHandle;
    HWND statisticsHandle;
    HWND controlsGroupHandle;
    HWND eventLogHandle;

    HWND startButtonHandle;
    HWND pauseButtonHandle;
    HWND resumeButtonHandle;
    HWND stopButtonHandle;
    HWND resetButtonHandle;
    HWND loadOsmButtonHandle;
    HWND searchPlaceButtonHandle;
    HWND demoCityButtonHandle;
    HWND fitMapButtonHandle;

    HWND tempoSectionLabel;
    HWND vehicleRateLabel;
    HWND vehicleRateEdit;
    HWND pedTempoLabel;
    HWND pedTempoEdit;
    HWND timeMulLabel;
    HWND timeMulEdit;
    HWND applyTempoButton;

    HWND blockSectionLabel;
    HWND blockRoadLabel;
    HWND blockRoadEdit;
    HWND blockLaneLabel;
    HWND blockLaneEdit;
    HWND blockDurLabel;
    HWND blockDurEdit;
    HWND addBlockButton;
    HWND clearBlocksButton;

    HWND randomSectionLabel;
    HWND randomEnableCheck;
    HWND randomIntervalLabel;
    HWND randomIntervalEdit;
    HWND randomDurationLabel;
    HWND randomDurationEdit;
    HWND randomSeedLabel;
    HWND randomSeedEdit;
    HWND applyRandomButton;

    HFONT uiFont;
    HFONT uiFontBold;
    HBRUSH chromeBrush;
    HBRUSH actionBarBrush;
    HBRUSH statusBrush;
    double dpiScale;
    bool running;
    int statsRefreshCounter;
    std::wstring lastUserStatus;

    static LRESULT CALLBACK windowProcedure(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
    static LRESULT CALLBACK mapViewProcedure(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

    LRESULT handleMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
    LRESULT handleMapViewMessage(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);

    void createChildWindows();
    void layoutChildWindows(int clientWidth, int clientHeight);
    void applyFontToChildren();
    void paintChrome(HWND hwnd, HDC hdc);
    void updateControlEnabledState();
    void updateFrame();
    void refreshPanels();
    void refreshStatisticsText();
    void refreshEventLogText();
    void refreshStatusBar();
    void syncParameterEditsFromSession();
    void setStatusMessage(const std::wstring& message);
    void onCommand(int commandId);

    bool readDoubleFromEdit(HWND edit, double& outValue) const;
    bool readIntFromEdit(HWND edit, int& outValue) const;
    void applyTempoParameters();
    void applyManualBlockage();
    void applyRandomBlockage();
    void onLoadOsmClicked();
    void onSearchPlaceClicked();
    void onDemoCityClicked();
    void showImportSummary(const OsmScenarioResult& result);
    void importOsmPath(const std::string& path, const std::wstring& statusPrefix);
    void fitMapToView();
    std::wstring mapLabel() const;
    std::wstring appStateLabel() const;
};

} // namespace gui
