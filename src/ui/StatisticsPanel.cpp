#include "ui/StatisticsPanel.h"

StatisticsPanel::StatisticsPanel(const SimulationSession& session) : session(session) {
}

const StatisticsSnapshot& StatisticsPanel::getSnapshot() const {
    return session.getSimulation().getStatisticsCollector().getLastSnapshot();
}
