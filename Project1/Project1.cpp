#include "Project1.h"

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
    return (RobotAction)(rand()%8);
}
