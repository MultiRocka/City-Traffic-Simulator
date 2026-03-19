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
    EventLog();

    void setOutputFile(const string& filePath);
    void clear();

    void addEvent(double time, const string& type, int vehicleId, int roadId, const string& reason, const string& description);
    void exportToJson() const;

    const vector<SimulationEvent>& getEvents() const;

private:
    string outputFilePath;
    vector<SimulationEvent> events;
};
