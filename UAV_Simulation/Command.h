#ifndef COMMAND_H
#define COMMAND_H

#include "project_headers.h"

// container for each command - used by the UAV class and the Simulation class
class Command {
private:
	double x, y, time;
	size_t uavNum;
public:

	Command(double x, double y, double time, const size_t& uavNum)
		: x(x), y(y), time(time), uavNum(uavNum)
	{

	}

	bool operator<(const Command& other) const;


	// avoid duplicate commands
	bool operator==(const Command& other) const;

	double getX() { return x; }
	double getX() const { return x; }
	double getY() { return y; }
	double getY() const { return y; }
	double getTime() { return time; }
	double getTime() const { return time; }
	size_t getUavNum() { return uavNum; }
	size_t getUavNum() const { return uavNum; }



	Command() = default;

	void showCommand() const;
};

#endif