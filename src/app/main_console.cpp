#include <thread>
#include <chrono>
#include <iostream>

#include "ui/ControlPanel.h"
#include "app/ConsolePreview.h"

using namespace std;

int main() {
    SimulationSession session;
    ControlPanel controlPanel(session);

    session.loadDemoScenario();
    controlPanel.start();

    for (int i = 0; i < 40; i++) {
        controlPanel.stepOnce();
        renderConsolePreview(session.buildRenderSnapshot());
        this_thread::sleep_for(chrono::milliseconds(500));
    }

    controlPanel.stop();
    session.exportResults();

    cout << endl;
    cout << "Exported: simulation_timeline.csv" << endl;
    cout << "Exported: simulation_summary.csv" << endl;
    cout << "Exported: event_log.json" << endl;

    return 0;
}
