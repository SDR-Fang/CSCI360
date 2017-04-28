#include "Project3.h"
#include <iostream>
#include <cmath>

// Get utility of actual action
float
Project3::get_utility (RobotAction actionResult, int x, int y){
	switch (actionResult) {
		case (RobotAction::MOVE_UP):
			if (x <= 0) // Cannot go up
				return utilityGrid[x][y];
			else
				return utilityGrid[x-1][y];
		break;
		case (RobotAction::MOVE_DOWN):
			if (x >= height - 1) // Cannot go down
				return utilityGrid[x][y];
			else
				return utilityGrid[x+1][y];
		break;
		case (RobotAction::MOVE_LEFT):
			if (y <= 0) // cannot go left
				return utilityGrid[x][y];
			else
				return utilityGrid[x][y-1];
		break;
		case (RobotAction::MOVE_RIGHT):
			if (y >= width - 1) // cannot go right
				return utilityGrid[x][y];
			else
				return utilityGrid[x][y+1];
		break;
		default:
			std::cout << "Warning: default action in get_utility()\n";
			return 0.0;
		break;
	}
	std::cout << "Warning: why you are here? (get_utility)\n";
	return 0.0;
}

// Update utility of state(x,y). Return false if changed value > error
bool
Project3::value_iteration(int x, int y){
	float ini_utility = utilityGrid[x][y];
	float uTemp, uMax; // temp and max utility
	// Special case: goal and obstacle
	if (rewardGrid[x][y] >= 1.0 | rewardGrid[x][y] <= -1.0)
		utilityGrid[x][y] = rewardGrid[x][y] + discount * 1.0 * utilityGrid[x][y];
	else {
		uMax = 0.0;
		// Up.
		uMax += pc * get_utility(RobotAction::MOVE_UP, x, y);
		uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, x, y);
		uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, x, y);
		uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, x, y);
		// Down
		uTemp = 0.0;
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, x, y);
		uTemp += pc * get_utility(RobotAction::MOVE_DOWN, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, x, y);
		if (uTemp > uMax) uMax = uTemp;
		// Left
		uTemp = 0.0;
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, x, y);
		uTemp += pc * get_utility(RobotAction::MOVE_LEFT, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, x, y);
		if (uTemp > uMax) uMax = uTemp;
		// Right
		uTemp = 0.0;
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, x, y);
		uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, x, y);
		uTemp += pc * get_utility(RobotAction::MOVE_RIGHT, x, y);
		if (uTemp > uMax) uMax = uTemp;
		utilityGrid[x][y] =  rewardGrid[x][y] + discount * uMax;
	}
	return std::abs(utilityGrid[x][y] - ini_utility) <= error;
}

/**
* @brief default constructor
*/
Project3::Project3(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
	width = sim1->getWidth();
	height = sim1->getHeight();
	target = sim1->getTarget();
	// Initialize grids and set reward value
	for (int i=0; i<height; ++i){
		std::vector<float> rewardRow;
		std::vector<float> utilityRow;
		std::vector<RobotAction> actionRow;
		for (int j=0; j<width; ++j){
			rewardRow.push_back(-0.05);
			utilityRow.push_back(0.0);
		}
		rewardGrid.push_back(rewardRow);
		utilityGrid.push_back(utilityRow);
	}
	rewardGrid[target.x][target.y] = 1.0;
	std::vector<Point2D> obstacles = sim1->getKnownObstacleLocations();
	for (int i=0; i<obstacles.size(); ++i)
		rewardGrid[obstacles[i].x][obstacles[i].y] = -1.0;

	bool noError = false;
	while (!noError) {
		noError = true;
		for (int i=0; i<height; ++i)
			for (int j=0; j<width; ++j)
				if (!value_iteration(i,j))
					noError = false;
	}

	// Debug
	std::cout.precision(1);
	std::cout << std::fixed;
	for (int i=0; i<height; ++i){
		for (int j=0; j<width; ++j){
			if (j!=0)
				std::cout << ' ';
			std::cout << utilityGrid[i][j];
		}
		std::cout << std::endl;
	}
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */
RobotAction Project3::getOptimalAction(Simulator* sim1, Robot* r1) {
	// Here, you should find the next step of the robot.
	// The robot should always follow a shortest path (wrt the known and sensed obstacles) to the goal.
	Vector2D pos = r1->getPosition();
	RobotAction result = MOVE_UP;
	// Get optimal by expcted utility
	float uMax = 0.0;
	// Up.
	uMax += pc * get_utility(RobotAction::MOVE_UP, pos.x, pos.y);
	uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, pos.x, pos.y);
	uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, pos.x, pos.y);
	uMax += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, pos.x, pos.y);
	// Down
	float uTemp = 0.0;
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, pos.x, pos.y);
	uTemp += pc * get_utility(RobotAction::MOVE_DOWN, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, pos.x, pos.y);
	if (uTemp > uMax){
		uMax = uTemp;
		result = MOVE_DOWN;
	}
	// Left
	uTemp = 0.0;
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, pos.x, pos.y);
	uTemp += pc * get_utility(RobotAction::MOVE_LEFT, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_RIGHT, pos.x, pos.y);
	if (uTemp > uMax){
		uMax = uTemp;
		result = MOVE_LEFT;
	}
	// Right
	uTemp = 0.0;
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_UP, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_DOWN, pos.x, pos.y);
	uTemp += (1 - pc) / 3.0 * get_utility(RobotAction::MOVE_LEFT, pos.x, pos.y);
	uTemp += pc * get_utility(RobotAction::MOVE_RIGHT, pos.x, pos.y);
	if (uTemp > uMax){
		uMax = uTemp;
		result = MOVE_RIGHT;
	}

	return result;
}