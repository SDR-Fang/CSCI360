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
	const float pc = 0.8;
	const float discount = 0.9;
    const float error = 0.05;
	std::vector<std::vector<float> > rewardGrid;
	std::vector<std::vector<float> > utilityGrid;

    int width, height;
    Vector2D target;

    float get_utility(RobotAction, int, int);
    bool value_iteration(int, int);
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
