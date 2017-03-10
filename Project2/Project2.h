#ifndef Project2_hpp
#define Project2_hpp

#include <stdio.h>
#include <vector>
#include "Robot.h"
#include "Vector2D.h"
#include "Simulator.h"

struct node{
    RobotAction next = RobotAction::STOP;
    // position
    float x, y;
    // Cost spend from start to this node
    float g = 0.0f;
    // Calculated heuristic
    float h = 0.0f;
    // If this node is blocked
    bool blocked = false;
};

class Project2 {
private:
    int width, height;
    Vector2D target;
    std::vector<std::vector<node>> nodeMap;
    bool tryFindPath(float, float);
    bool first_move = true;
public:
    /**
     * @brief default constructor
     */
    Project2(Simulator* sim1);

    /**
     * @brief get optimal action
     * @param sim1 simulator pointer
     * @param r robot pointer
     * @return optimal action
     */
    RobotAction getOptimalAction(Simulator* sim1, Robot* r1);
};

#endif
