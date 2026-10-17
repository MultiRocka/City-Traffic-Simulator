#ifndef UNICODE
#define UNICODE
#endif

#ifndef NOMINMAX
#define NOMINMAX
#endif

#include <windows.h>
#include "gui/MainWindow.h"

namespace {

void enableDpiAwareness() {
    // Prefer per-monitor v2 when available (Win10 1703+); fall back gracefully.
    using SetDpiAwarenessContextFn = BOOL(WINAPI*)(DPI_AWARENESS_CONTEXT);
    HMODULE user32 = GetModuleHandleW(L"user32.dll");
    if (user32 != nullptr) {
        auto setContext = reinterpret_cast<SetDpiAwarenessContextFn>(
            GetProcAddress(user32, "SetProcessDpiAwarenessContext")
        );
        if (setContext != nullptr) {
            if (setContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE_V2)) {
                return;
            }
            if (setContext(DPI_AWARENESS_CONTEXT_PER_MONITOR_AWARE)) {
                return;
            }
        }
    }
    SetProcessDPIAware();
}

} // namespace

int WINAPI wWinMain(HINSTANCE instanceHandle, HINSTANCE, PWSTR, int) {
    enableDpiAwareness();

    gui::MainWindow mainWindow;

    if (!mainWindow.initialize(instanceHandle)) {
        MessageBoxW(nullptr, L"Nie udało się zainicjalizować GUI Easy Rider.", L"Easy Rider", MB_ICONERROR);
        return 1;
    }

    mainWindow.loadDemoScenario();
    return mainWindow.run();
}
