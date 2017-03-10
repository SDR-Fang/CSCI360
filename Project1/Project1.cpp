#include <iostream>
#include "Project1.h"
#include "Robot.h"

/**
* @posX current X position
* @posY current Y position
* @tarX target X position
* @tarY target Y position
*/
float Heuristic (int posX, int posY, int tarX, int tarY){
	// Simply use distance as Heuristic
	int diffX = abs(posX - tarX);
	int diffY = abs(posY - tarY);
	float result = diffX > diffY ? diffY*0.5+diffX : diffX*0.5+diffY;
		// std::cout << "Heuristic: (" << posX << ',' << posY << ") to ("
		// 	<< tarX << ',' << tarY << ")=" << result << "\n";
	return result;
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
	float next_heuristic = 0;
	float temp_heuristic = 0;
	RobotAction action = STOP;
	Point2D target = sim1->getTarget();
	// check actions
	// MOVE_UP
	next_heuristic = Heuristic(r1->X - 1, r1->Y, target.x, target.y);
	action = MOVE_UP;
	// std::cout << "MOVE_UP:" << next_heuristic << std::endl;
	// MOVE_DOWN
	temp_heuristic = Heuristic(r1->X + 1, r1->Y, target.x, target.y);
	// std::cout << "MOVE_DOWN:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_DOWN;
	}
	// MOVE_LEFT
	temp_heuristic = Heuristic(r1->X, r1->Y - 1, target.x, target.y);
	// std::cout << "MOVE_LEFT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_LEFT;
	}
	// MOVE_RIGHT
	temp_heuristic = Heuristic(r1->X, r1->Y + 1, target.x, target.y);
	// std::cout << "MOVE_RIGHT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_RIGHT;
	}
	// MOVE_UP_RIGHT
	temp_heuristic = Heuristic(r1->X - 1, r1->Y + 1, target.x, target.y);
	// std::cout << "MOVE_UP_RIGHT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_UP_RIGHT;
	}
	// MOVE_UP_LEFT
	temp_heuristic = Heuristic(r1->X - 1, r1->Y - 1, target.x, target.y);
	// std::cout << "MOVE_UP_LEFT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_UP_LEFT;
	}
	// MOVE_DOWN_RIGHT
	temp_heuristic = Heuristic(r1->X + 1, r1->Y + 1, target.x, target.y);
	// std::cout << "MOVE_DOWN_RIGHT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_DOWN_RIGHT;
	}
 	// MOVE_DOWN_LEFT
	temp_heuristic = Heuristic(r1->X + 1, r1->Y - 1, target.x, target.y);
	// std::cout << "MOVE_DOWN_LEFT:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = MOVE_DOWN_LEFT;
	}
 	// STOP
	temp_heuristic = Heuristic(r1->X, r1->Y, target.x, target.y);
	// std::cout << "STOP:" << temp_heuristic << std::endl;
	if (temp_heuristic < next_heuristic){
		next_heuristic = temp_heuristic;
		action = STOP;
	}
	
    return action;
}
