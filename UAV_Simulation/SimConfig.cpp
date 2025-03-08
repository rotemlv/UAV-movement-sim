#include "SimConfig.h"

SimConfig::SimConfig(double x, double y, double z, double v0, double r0, double initialAngleRadians, double timeLimit, double dt, const size_t& totalUavs)
	: x(x), y(y), z(z), v0(v0), r0(r0), initialAngleRadians(initialAngleRadians), timeLimit(timeLimit), dt(dt), totalUavs(totalUavs)
{
}

void SimConfig::showConfig() const {
	std::cout << "Loaded Configuration:" << '\n';
	std::cout << "Number of UAVs: " << this->totalUavs << '\n';
	std::cout << "Turn Radius: " << this->r0 << '\n';
	std::cout << "Initial Position: (" << this->x << ", " << this->y << ", " << this->z << ")" << '\n';
	std::cout << "Velocity: " << this->v0 << '\n';
	std::cout << "Initial Azimuth: " << (this->initialAngleRadians * 180. / M_PI) << " degrees" << '\n';
	std::cout << "Simulation Delta: " << this->dt << '\n';
	std::cout << "Time Limit: " << this->timeLimit << '\n';
}