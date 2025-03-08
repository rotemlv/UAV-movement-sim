#ifndef UAV_H
#define UAV_H

#include "project_headers.h"
#include "Command.h"
#include "uav_utilities.h"

// An object representation of a UAV for the simulation, with navigation component
// according to the stated requirements
class UAV {
private:
	enum State {
		CRUISE, HAS_DEST, PREP_TURN, TURN, ROTATE
	};
	size_t uavNum;
	double x, y; // z is irrelevant for our purpose, though it is stored in the config object
	double radianAngle;
	double destX, destY;
	double velocity, turnRadius;
	double omega; // omega is our angular speed, defined according to physics rules
	double dt;
	bool clockwise;

	State state;

	bool rotatingClockwise();

	// methods for handling flight logic
	void confirmArrival();

	void applyAngleChange();

	void turnLogic();

	bool turnIsPossible();

public:
	UAV(const size_t& uavNum, double x, double y, double radianAngle, double velocity, double turnRadius, double dt);

	void setDest(const double x, const double y);

	void acceptCommand(const Command& command);

	void handleTurnPreperation();

	void flightStep(const double currentTime);

	size_t getUavNum() { return uavNum; };
	const size_t getUavNum() const { return uavNum; };


	double getX() { return x; };
	const double getX() const { return x; };


	double getY() { return y; };
	const double getY() const { return y; };


	double getAngleRad() { return radianAngle; };
	const double getAngleRad() const { return radianAngle; };


	double getDestX() { return destX; };
	const double getDestX() const { return destX; };


	double getDestY() { return destY; };
	const double getDestY() const { return destY; };

	double getVelocity() { return velocity; };
	const double getVelocity() const { return velocity; };

	double getTurnRadius() { return turnRadius; };
	const double getTurnRadius() const { return turnRadius; };

	State getState() { return state; };
	const State getState() const { return state; };

	// setter (for state only)
	void setState(const State state) {
		this->state = state;
	}

	void showUAV() const;

};

#endif