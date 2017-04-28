#include "Project3.h"

/**
* @brief default constructor
*/
Project3::Project3(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
	width = sim1->getWidth();
	height = sim1->getHeight();
	target = sim1->getTarget();
	// Initialize grids
	for (int i=0; i<height; ++i){
		std::vector<float> rewardRow;
		std::vector<float> utilityRow;
		std::vector<float> actionRow;
		for (int j=0; j<idth; ++j){
			rewardRow.push_back(-0.05);
			utilityRow.push_back(0.0);
			actionRow.push_back(RobotAction::MOVE_RIGHT);
		}
		rewardGrid.push_back(rewardRow);
		utilityGrid.push_back(utilityRow);
		actionGrid.push_back(actionRow);
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
    return actionGrid[pos.X][pos.Y];
}
