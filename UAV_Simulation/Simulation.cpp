#include "Simulation.h"


std::vector<Command> Simulation::readCommandsFromFile(const std::string& filename) {
    std::vector<Command> commands;
    commands.reserve(10); // avoid the extra initial allocations
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        //std::cout << " Line read: " << line << "\n";

        std::istringstream iss(line);
        double time, x, y;
        int uavNum;

        if (!(iss >> time >> uavNum >> x >> y)) {
            throw std::runtime_error("Invalid line format in file");
        }

        Command cmd(x, y, time, uavNum);
        commands.push_back(cmd);
    }

    file.close();
    return commands;
}

std::vector<Command> Simulation::loadCommandsVectorFromFileSorted(const std::string& filename) {
    std::vector<Command> commands = readCommandsFromFile(filename);
    std::sort(commands.begin(), commands.end());
    return commands;

}

// Function to trim whitespace from a string
const std::string Simulation::trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t");
    size_t end = s.find_last_not_of(" \t");
    return (start == std::string::npos || end == std::string::npos) ? "" : s.substr(start, end - start + 1);
}

// Function to read a double value from a string
const double Simulation::readdouble(const std::string& s) {
    std::istringstream iss(trim(s));
    double value;
    iss >> value;
    return value;
}

// Function to read an int value from a string
const int Simulation::readint(const std::string& s) {
    std::istringstream iss(trim(s));
    int value;
    iss >> value;
    return value;
}

SimConfig Simulation::loadConfig(std::string filename) {
    // default values
    double x = 0.0, y = 0.0, z = 0.0, dt = 0.0, timeLimit = 0.0, radius = 0.0, velocity = 0.0, azimuth = 0.0;
    size_t nUavs = 0;
    std::ifstream configFile(filename);

    if (!configFile.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    std::string line;
    bool allFieldsFound = true;

    while (std::getline(configFile, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        size_t equalsPos = line.find('=');
        if (equalsPos == std::string::npos) continue;

        const std::string key = trim(line.substr(0, equalsPos));
        const std::string value = trim(line.substr(equalsPos + 1));

        try {
            if (key == "Dt") dt = readdouble(value);
            else if (key == "N_uav") nUavs = readint(value);
            else if (key == "R") radius = readdouble(value);
            else if (key == "X0") x = readdouble(value);
            else if (key == "Y0") y = readdouble(value);
            else if (key == "Z0") z = readdouble(value);
            else if (key == "V0") velocity = readdouble(value);
            else if (key == "Az") azimuth = readdouble(value);
            else if (key == "TimeLim") timeLimit = readdouble(value);
            else {
                // Unknown key
                throw std::exception("Invalid key!");
            }
        }
        catch (const std::exception&) {
            std::cerr << "Warning: Failed to parse value for '" << key << "'. Skipping." << '\n';
            allFieldsFound = false;
        }
    }

    configFile.close();

    // Check if all required fields were found
    if (!allFieldsFound) {
        throw std::runtime_error("Not all required configuration fields were found or parsed correctly.");
    }

    // init config object
    return SimConfig(x,y,z,velocity, radius, azimuth * M_PI / 180., timeLimit, dt, nUavs);

}


void Simulation::verboseShowRunInfo()
{
    for (const auto& c : commands) {
        std::cout << "command params (x,y,time,uav): " << c.getX() << ", " << c.getY() << ", " << c.getTime() << ", " << c.getUavNum() << "\n";
    }
    int n = 0; // dummy variable for storing UAV number
    // Print loaded configuration
    config.showConfig();

    // Print initial UAV states
    std::cout << "\nInitial UAV States:" << '\n';

    // print location and current angle of each UAV, numbered.
    for (const auto& uav : uavs) {
        uav.showUAV();
    }
}

const std::vector<UAV> Simulation::initializeUAVs(const SimConfig& config) {
    std::vector<UAV> uavs;
    //uavs.reserve(config.getTotalUavs());
    for (size_t i = 0; i < config.getTotalUavs(); ++i) {
        uavs.emplace_back(i, config.getX(), config.getY(), config.getAngleRad(), config.getV0(), config.getR0(), config.getDt());
    }
    return uavs;
}


void Simulation::run() {

    std::vector<std::ofstream> streams(config.getTotalUavs());
    // initialize file streams and open them
    for (size_t i = 0; i < config.getTotalUavs(); i++) {
        std::string fileName = "UAV" + std::to_string(i) + ".txt";
        streams[i].open(fileName.c_str());
    }
    if (_VERBOSE)
        std::cout << "\n - - - Simulation begins - - - \n";
    for (double currentTime = 0.; currentTime < config.getTimeLimit(); currentTime += config.getDt()) {
        // before performing each tick, fetch commands
        // store last command to avoid calling duplicates (in case these exist in "queue")
        Command lastCommand = { -1,-1,-1,0 };
        // check if there are commands to perform, if yes, check if time for first command has arrived.
        while (commands.size() && currentTime >= commands.back().getTime()) {
            // pop command
            const Command command = commands.back();
            const size_t uavNum = command.getUavNum();
            commands.pop_back();  // assuming there are thousands of commands, we remove them during simulation to maintain low memory footprint
            if (command == lastCommand)
                continue;   // ignore duplicate commands
            if (_VERBOSE) {
                // print command
                std::cout << "Executing command: " << command.getTime() << ", x,y:" << command.getX() << ", " <<
                    command.getY() << ". Command for UAV num: " << command.getUavNum() << "\n";
            }
            // turning logic here - begins here and then goes through stages as described below
            uavs[uavNum].acceptCommand(command);
            // store current command
            lastCommand = command;
        }
        // perform tick logic for each UAV
        for (auto& uav : uavs) {
            // while is used since we don't know for a fact there is always just one command per UAV per unit of time.
            uav.flightStep(currentTime);
            // Write current stats to file (we only need degrees here, so we convert here)
            streams[uav.getUavNum()] << std::fixed << std::setprecision(2) <<
                currentTime << " " << uav.getX() << " " << uav.getY() << " " << (uav.getAngleRad() * 180. / M_PI) << '\n';
        }

    }
    // close files
    for (auto& s : streams) {
        if (s.is_open()) {
            s.close();
        }
    }
}

// constructor - loads config and commands from files and creates UAVs for simulation
Simulation::Simulation(const std::string configFile, const std::string commandsFile)
    : config(loadConfig(configFile)), commands(loadCommandsVectorFromFileSorted(commandsFile))
{
    this->uavs = initializeUAVs(config);
}

// show data
void Simulation::showSimulationPrep() {
    std::cout << "Showing config: \n";
    config.showConfig();
    std::cout << "Showing commands: \n";
    for (const auto &c : commands) {
        std::cout << "Command time: " << c.getTime() << "x,y: " << c.getX() << "," << c.getY() << " for UAV number: " << c.getUavNum() << "\n";

    }
    std::cout << "Showing UAVs:\n";
    for (const auto &u : uavs) {
        u.showUAV();
    }
}

