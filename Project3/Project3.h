#ifndef Project3_hpp
#define Project3_hpp

#include <stdio.h>
#include <vector>
#include "Robot.h"
#include "Vector2D.h"
#include "Simulator.h"

class Project3 {
private:
	// Probability of Correct action
	const float pc = 1.0;
	const float discount = 0.9;
	std::vector<std::vector<float>> rewardGrid;
	std::vector<std::vector<float>> utilityGrid;
	std::vector<std::vector<RobotAction>> actionGrid;
public:
    /**
     * @brief default constructor
     */
    Project3(Simulator* sim1);

    /**
     * @brief get optimal action
     * @param sim1 simulator pointer
     * @param r robot pointer
     * @return optimal action
     */
    RobotAction getOptimalAction(Simulator* sim1, Robot* r1);
};

#endif
