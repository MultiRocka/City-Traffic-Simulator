#include "core/EventLog.h"
#include <fstream>
#include <iomanip>

using namespace std;

static string escapeJsonString(const string& value) {
    string escaped;
    escaped.reserve(value.size());

    for (char character : value) {
        switch (character) {
            case '\\':
                escaped += "\\\\";
                break;
            case '"':
                escaped += "\\\"";
                break;
            case '\n':
                escaped += "\\n";
                break;
            case '\r':
                escaped += "\\r";
                break;
            case '\t':
                escaped += "\\t";
                break;
            default:
                escaped += character;
                break;
        }
    }

    return escaped;
}

EventLog::EventLog()
    : outputFilePath("event_log.json"),
      maxEvents(defaultMaxEvents),
      droppedEvents(0) {
}

void EventLog::setOutputFile(const string& filePath) {
    outputFilePath = filePath;
}

void EventLog::clear() {
    events.clear();
    droppedEvents = 0;
}

void EventLog::setMaxEvents(size_t limit) {
    maxEvents = limit;
    if (maxEvents > 0 && events.size() > maxEvents) {
        const size_t overflow = events.size() - maxEvents;
        events.erase(events.begin(), events.begin() + static_cast<long>(overflow));
        droppedEvents += overflow;
    }
}

size_t EventLog::getMaxEvents() const {
    return maxEvents;
}

size_t EventLog::getDroppedEventCount() const {
    return droppedEvents;
}

void EventLog::addEvent(double time, const string& type, int vehicleId, int roadId, const string& reason, const string& description) {
    if (maxEvents > 0 && events.size() >= maxEvents) {
        // Hard cap: stop accepting rather than unbounded growth on continuous runs.
        ++droppedEvents;
        return;
    }

    SimulationEvent event;
    event.time = time;
    event.type = type;
    event.vehicleId = vehicleId;
    event.roadId = roadId;
    event.reason = reason;
    event.description = description;

    events.push_back(event);
}

void EventLog::exportToJson() const {
    ofstream file(outputFilePath);

    file << "[\n";

    for (int i = 0; i < static_cast<int>(events.size()); i++) {
        const auto& event = events[i];

        file << "  {\n";
        file << "    \"time\": " << fixed << setprecision(2) << event.time << ",\n";
        file << "    \"type\": \"" << escapeJsonString(event.type) << "\",\n";
        file << "    \"vehicleId\": " << event.vehicleId << ",\n";
        file << "    \"roadId\": " << event.roadId << ",\n";
        file << "    \"reason\": \"" << escapeJsonString(event.reason) << "\",\n";
        file << "    \"description\": \"" << escapeJsonString(event.description) << "\"\n";
        file << "  }";

        if (i + 1 < static_cast<int>(events.size())) {
            file << ",";
        }

        file << "\n";
    }

    file << "]\n";

    file.close();
}

const vector<SimulationEvent>& EventLog::getEvents() const {
    return events;
}
