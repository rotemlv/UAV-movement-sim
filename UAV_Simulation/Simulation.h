#ifndef SIMULATION_H
#define SIMULATION_H


#include "project_headers.h"
#include "UAV.h"
#include "SimConfig.h"
#include "Command.h"

class Simulation {
private:
    std::vector<UAV> uavs;
    const SimConfig config;
    std::vector<Command> commands;


    std::vector<Command> readCommandsFromFile(const std::string& filename);
    std::vector<Command> loadCommandsVectorFromFileSorted(const std::string& filename);
    // file cleanup functions
    const std::string trim(const std::string& s);
    const double readdouble(const std::string& s);
    const int readint(const std::string& s);

    // load config
    SimConfig loadConfig(std::string filename);

    // show info
    void verboseShowRunInfo();

    const std::vector<UAV> initializeUAVs(const SimConfig& config);

public:

    void run();

    // constructor
    Simulation(const std::string configFile, const std::string commandsFile);

    // show data
    void showSimulationPrep();
};

#endif

