#include "Command.h"

// compare commands by time, from high to low
bool Command::operator<(const Command& other) const
{
	return this->getTime() > other.getTime();
}


// used to avoid duplicate commands
bool Command::operator==(const Command& other) const
{
	return x == other.x && y == other.y && time == other.time && uavNum == other.uavNum;
}

void Command::showCommand() const {
	std::cout << "Command time: " << getTime() << "x,y: " << getX() << "," << getY() << " for UAV number: " << getUavNum() << "\n";

}