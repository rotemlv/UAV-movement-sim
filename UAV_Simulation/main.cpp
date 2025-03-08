// memory leak detection
#define _CRTDBG_MAP_ALLOC
#include<crtdbg.h>
#include "Simulation.h"

int main()
try {
    // checking for memory leaks while avoiding false positives from static objects in some libraries
    _CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

    Simulation sim("SimParams.ini", 
        "SimCmds.txt");

    // print all simulation details
    if(_VERBOSE)
        sim.showSimulationPrep();

    sim.run();

    return 0;
}
catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 1;
}

