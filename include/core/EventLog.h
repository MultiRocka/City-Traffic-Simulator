#pragma once

#include <string>
#include <vector>

using namespace std;

struct SimulationEvent {
    double time;
    string type;
    int vehicleId;
    int roadId;
    string reason;
    string description;
};

class EventLog {
public:
    // Soft cap so long continuous runs do not grow unbounded. 0 = unlimited.
    static constexpr size_t defaultMaxEvents = 50000;

    EventLog();

    void setOutputFile(const string& filePath);
    void clear();

    void setMaxEvents(size_t maxEvents);
    size_t getMaxEvents() const;
    size_t getDroppedEventCount() const;

    void addEvent(double time, const string& type, int vehicleId, int roadId, const string& reason, const string& description);
    void exportToJson() const;

    const vector<SimulationEvent>& getEvents() const;

private:
    string outputFilePath;
    vector<SimulationEvent> events;
    size_t maxEvents;
    size_t droppedEvents;
};
