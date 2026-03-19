#pragma once

class Simulation {
public:
    Simulation();

    void start();
    void stop();
    void update(double dt);

    bool isRunning() const;
    double getTime() const;

private:
    bool running;
    double currentTime;
};
