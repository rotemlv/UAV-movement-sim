#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H
#include "project_headers.h"

// defines the configuration object for the current simulation
class SimConfig {
private:
	double x, y, z, v0, r0, initialAngleRadians, timeLimit, dt;
	size_t totalUavs;


public:

	SimConfig(double x, double y, double z, double v0, double r0, double initialAngleRadians, double timeLimit, double dt, const size_t& totalUavs);

	double getX() { return x; }
	double getX() const { return x; }

	double getY() { return y; }
	double getY() const { return y; }

	double getZ() { return z; }
	double getZ() const { return z; }

	double getV0() { return v0; }
	double getV0() const { return v0; }

	double getR0() { return r0; }
	double getR0() const { return r0; }

	double getAngleRad() { return initialAngleRadians; }
	double getAngleRad() const { return initialAngleRadians; }


	double getTimeLimit() { return timeLimit; }
	double getTimeLimit() const { return timeLimit; }

	double getDt() { return dt; }
	double getDt() const { return dt; }

	size_t getTotalUavs() { return totalUavs; }
	size_t getTotalUavs() const { return totalUavs; }



	SimConfig() = default;

	void showConfig() const;
};

#endif