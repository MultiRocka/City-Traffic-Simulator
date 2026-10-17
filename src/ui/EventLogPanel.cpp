#include "ui/EventLogPanel.h"

EventLogPanel::EventLogPanel(const SimulationSession& session) : session(session) {
}

const vector<SimulationEvent>& EventLogPanel::getEvents() const {
    return session.getSimulation().getEventLog().getEvents();
}

string EventLogPanel::formatReasonLabel(const string& reason) {
    // Only labels for reasons already produced by core. Unknown → unchanged.
    if (reason == "red_light") {
        return "red";
    }
    if (reason == "vehicle_ahead") {
        return "vehicle ahead";
    }
    if (reason == "pedestrian_crossing" || reason == "pedestrian_on_road") {
        return "pedestrian";
    }
    if (reason == "yield_priority") {
        return "priority/yield";
    }
    if (reason == "intersection_conflict") {
        return "intersection conflict";
    }
    if (reason == "blocked_next_road") {
        return "blocked next road";
    }
    if (reason == "lane_blockage") {
        return "lane blockage";
    }
    if (reason == "zipper_wait") {
        return "waiting lane change";
    }
    if (reason == "destination_reached") {
        return "destination";
    }
    if (reason == "route_invalid") {
        return "invalid route";
    }
    return reason;
}
