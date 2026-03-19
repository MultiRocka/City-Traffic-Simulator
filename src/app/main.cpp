#include <iostream>
#include "core/Simulation.h"

using namespace std;

int main() {
    Simulation sim;

    sim.start();

    for (int i = 0; i < 10; i++) {
        sim.update(0.1);
        cout << "Czas symulacji: " << sim.getTime() << " s" << endl;
    }

    sim.stop();

    return 0;
}
