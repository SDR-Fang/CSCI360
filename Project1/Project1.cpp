#include "Project1.h"
#include "Robot.h"

/**
* @posX current X position
* @posY current Y position
* @tarX target X position
* @tarY target Y position
*/
int Heuristic (int posX, int posY, int tarX, int tarY){
	// Simply use distance as Heuristic
	return (posX - tarX) * (posX - tarX) + (posY - tarY) * (posY - tarY);
}

/**
* @brief default constructor
*/
Project1::Project1(Simulator* sim1) {
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */
RobotAction Project1::getOptimalAction(Simulator* sim1, Robot* r1) {
    // Here, you should find the next step of the robot.
	int next_heuristic = 0;
	RobotAction action = STOP;
	// check actions
	
    return action;
}
