#include "Project3.h"

/**
* @brief default constructor
*/
Project3::Project3(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
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
    return (RobotAction)(MOVE_RIGHT);
}
