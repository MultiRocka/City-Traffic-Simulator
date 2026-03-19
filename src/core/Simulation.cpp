#include "core/Simulation.h"

using namespace std;

Simulation::Simulation() : running(false), currentTime(0.0) {
}

void Simulation::start() {
    running = true;
}

void Simulation::stop() {
    running = false;
}

void Simulation::update(double dt) {
    if (!running) {
        return;
    }

    currentTime += dt;
}

bool Simulation::isRunning() const {
    return running;
}

double Simulation::getTime() const {
    return currentTime;
}
