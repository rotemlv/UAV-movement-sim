#include "UAV.h"


// check if we are rotating around the destination clock-wise
bool UAV::rotatingClockwise() {
	const double cx = x - destX, cy = y - destY;
	const double meX = cos(radianAngle), meY = sin(radianAngle);
	return (meX * cy - meY * cx) > 0;
}

void UAV::confirmArrival() {
	// project next step's distance
	double nextX, nextY;
	double nextAngle = radianAngle;
	double currDist = vec2DDist(x, y, destX, destY);
	if (currDist >= (1.4 + dt)*turnRadius) // safety distance from centre, we only check in detail when close enough
		return;
	if (getState() == UAV::State::TURN) {
		// apply turn logic to next values
		nextAngle = (clockwise) ? (nextAngle - dt * omega) : (nextAngle + dt * omega);
	}
	bool rightAngle = equals_epsilon(normalizedDotProduct2D(cos(radianAngle), sin(radianAngle), destX - x, destY - y), 0., dt * velocity / turnRadius);

	nextX = x + dt * velocity * cos(nextAngle);
	nextY = y + dt * velocity * sin(nextAngle);
	// if we are at (roughly) 90 degrees angle with vector to center, and we are closest we will be to it, we have arrived
	if (rightAngle && (currDist < vec2DDist(nextX, nextY, destX, destY))) {
		setState(UAV::State::ROTATE);
		clockwise = true; // we are going to rotate clock-wise
		if(_VERBOSE)
			std::cout << "Arrived at tangent!\n";
	}
}

void UAV::applyAngleChange()
{
	// apply new angle and correct it from going overboard
	radianAngle = (clockwise) ? radianAngle - omega * dt : radianAngle + omega * dt;
	// solution for clamping without explicit ifs - from SO 
	radianAngle -= (2 * M_PI) * floor(radianAngle / (2 * M_PI));
}

// in this version we check if we are on the correct angle using law of sines.
// no course correction here
void UAV::turnLogic() {
	// get angle
	double sineRatio = sqrt((destX - x) * (destX - x) + (destY - y) * (destY - y)); // sin(90) = 1
	// (using sine theorem)
	// distToDest / sin(90) = R / sin(theta) -> theta = arcsin(R / distToDest)
	double theta = asin(turnRadius / sineRatio);
	double proposedAngle = (getAngleBetweenTwoVectors(1, 0, destX - x, destY - y) * M_PI / 180.) + theta;
	if (fabs(radianAngle - proposedAngle) <= (dt * velocity / turnRadius)) {
		setState(HAS_DEST);
		return;
	}
	applyAngleChange();
}

// Did not manage to prove this is the most efficient way to determine if we can turn,
// it is for turning directly into a point.
bool UAV::turnIsPossible() {
	// if we are further than 2R from dest, anything is possible (any turn)
	if (vec2DDist(destX, destY, x, y) >= 2 * turnRadius)
		return true;
	// else, a more complicated calculation is required - using the circle equation for our potential turn:
	const double angleToCircleCenter = (clockwise) ? -M_PI_2 : M_PI_2;  // depends on turn direction, pre-calculated
	const double cX = x + turnRadius * cos(angleToCircleCenter + radianAngle);
	const double cY = y + turnRadius * sin(angleToCircleCenter + radianAngle);
	// now, circle equation for the circular motion the UAV can perform is:
	// (x - cX)^2 + (y - cY)^2 = R
	// we want to see that, when plugging destX, destY, we get something at least as large as R
	// this means that we will be able to reach the destination, were we to start turning now
	const double deltaX = destX - cX;
	const double deltaY = destY - cY;
	return (deltaX * deltaX + deltaY * deltaY) >= turnRadius;
}

// Public methods:

UAV::UAV(const size_t& uavNum, double x, double y, double radianAngle, double velocity, double turnRadius, double dt)
	: uavNum(uavNum), x(x), y(y), radianAngle(radianAngle), destX(0.), destY(0.),
	velocity(velocity), turnRadius(turnRadius), omega(velocity / turnRadius), dt(dt), clockwise(false),
	state(UAV::State::CRUISE)
{
}

void UAV::setDest(const double x,const double y) {
	this->destX = x;
	this->destY = y;
}

void UAV::acceptCommand(const Command& command) {
	setDest(command.getX(), command.getY());
	setState(PREP_TURN);
	// four arguments: unit vector with current azimuth, vector between curr point and dest
	// order here is important as angle is defined relative to "me" (the UAV).
	
	// TODO: use rule-of-sines to adjust this from dest-directed turn to dest-tangent turn
	const double angleBetweenVectors =
		getAngleBetweenTwoVectors(cos(radianAngle), sin(radianAngle), destX - x, destY - y);
	
	// if "my" (UAV -> dest turn) angle is greater than 180, turn clockwise.
	clockwise = (angleBetweenVectors - radianAngle > 180);
	if(_VERBOSE)
		std::cout << "UAV#" << uavNum << " received command to move to : " << destX << ", " << destY << "\n";
}

void UAV::handleTurnPreperation()
{
	// this check is used for edge case of being tangent both last and current destination
	confirmArrival();
	if (getState() != UAV::State::ROTATE && turnIsPossible())
		setState(UAV::State::TURN);
}

void UAV::flightStep(const double currentTime) { // current time is used for debug (verbose printing)
	// "state machine" - UAV:
	switch (getState()) {
	case UAV::State::CRUISE:   // cruising without destination (at first)
		break;
	case UAV::State::PREP_TURN: // check if we can turn (not too close to objective from wrong direction)
		handleTurnPreperation();
		// allow jumping to the correct path if we can start turning now
		if(getState() == UAV::State::PREP_TURN)
			break;
	case UAV::State::HAS_DEST:  // cruising to destination (no turn yes dest)
		confirmArrival();
		break;
	case UAV::State::TURN:	// turning to a destination
		turnLogic();
		break;
	case UAV::State::ROTATE: // rotating around the destination
		applyAngleChange();
		break;
	default:
		throw std::exception("UAV state not-implemented");
	}

	x = x + dt * velocity * cos(radianAngle);
	y = y + dt * velocity * sin(radianAngle);

		
}

void UAV::showUAV() const {
	// this only prints the important stuff
	std::cout << "UAV number " << uavNum << ": ";
	std::cout << "Coordinates (x,y,z): (" << x << ", " << y << ") Azimuth: " << (radianAngle * 180. / M_PI) << '\n';
}

