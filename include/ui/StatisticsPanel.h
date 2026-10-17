#pragma once

#include "core/StatisticsCollector.h"
#include "ui/SimulationSession.h"

class StatisticsPanel {
public:
    explicit StatisticsPanel(const SimulationSession& session);

    const StatisticsSnapshot& getSnapshot() const;

private:
    const SimulationSession& session;
};
