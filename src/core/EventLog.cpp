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

EventLog::EventLog() : outputFilePath("event_log.json") {
}

void EventLog::setOutputFile(const string& filePath) {
    outputFilePath = filePath;
}

void EventLog::clear() {
    events.clear();
}

void EventLog::addEvent(double time, const string& type, int vehicleId, int roadId, const string& reason, const string& description) {
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
