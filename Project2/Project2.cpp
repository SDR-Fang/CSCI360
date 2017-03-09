#include <stdlib.h>
#include "Project2.h"

/**
* @posX current X position
* @posY current Y position
* @tarX target X position
* @tarY target Y position
*/
int Heuristic (int posX, int posY, int tarX, int tarY){
	// Simply use distance as Heuristic
	diffX = abs(posX - tarX);
	diffY = abs(posY - tarY);
	return  diffX > diffY ? diffY*0.5+diffX : diffX*0.5+diffY;
}

/**
* @brief default constructor
*/
Project2::Project2(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
}

/**
 * @brief get optimal action
 * @param sim simulator pointer
 * @param r robot pointer
 * @return optimal action
 */
RobotAction Project2::getOptimalAction(Simulator* sim1, Robot* r1) {
	// Here, you should find the next step of the robot.
	// The robot should always follow a shortest path (wrt the known and sensed obstacles) to the goal.
    return (RobotAction)(rand()%8);
}
