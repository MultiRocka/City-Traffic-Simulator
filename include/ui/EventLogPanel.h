#pragma once

#include <string>
#include <vector>
#include "core/EventLog.h"
#include "ui/SimulationSession.h"

using namespace std;

class EventLogPanel {
public:
    explicit EventLogPanel(const SimulationSession& session);

    const vector<SimulationEvent>& getEvents() const;

    // Maps core stop/event reason codes to a short display label.
    // Unknown codes are returned unchanged (never invents a random reason).
    static string formatReasonLabel(const string& reason);

private:
    const SimulationSession& session;
};
