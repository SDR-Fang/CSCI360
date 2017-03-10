#include <iostream>
#include <stdlib.h>
#include <unordered_set>
#include "Project2.h"

/**
* @posX current X position
* @posY current Y position
* @tarX target X position
* @tarY target Y position
*/
float Heuristic (float posX, float posY, float tarX, float tarY){
	int diffX = abs(posX - tarX);
	int diffY = abs(posY - tarY);
	return diffX > diffY ? diffY*0.5f+diffX : diffX*0.5f+diffY;
}

// Return a robot action could move from a to b
RobotAction
getActon (node* a, node* b){
	if (a->x < b->x){
		if (a->y < b->y)
			return RobotAction::MOVE_DOWN_RIGHT;
		if (a->y > b->y)
			return RobotAction::MOVE_DOWN_LEFT;
		return RobotAction::MOVE_DOWN;
	}

	if (a->x > b->x){
		if (a->y < b->y)
			return RobotAction::MOVE_UP_RIGHT;
		if (a->y > b->y)
			return RobotAction::MOVE_UP_LEFT;
		return RobotAction::MOVE_UP;
	}

	if (a->y < b->y)
		return RobotAction::MOVE_RIGHT;
	if (a->y > b->y)
		return RobotAction::MOVE_LEFT;
}

/**
* @brief default constructor
*/
Project2::Project2(Simulator* sim1) {
	// Here, you should initialize the grid with all the known obstacles.
	width = sim1->getWidth();
	height = sim1->getHeight();
	target = sim1->getTarget();
	// Initialize node map
	for (int i=0; i<height; ++i){
		// Create row node vector
		std::vector<node> row_vec;
		for (int j=0; j<width; ++j){
			// Create new node and calculate heuristic
			node temp_node;
			temp_node.x = i;
			temp_node.y = j;
			row_vec.push_back(temp_node);
		}
		nodeMap.push_back(row_vec);
	}
	// Record known obstacles
	std::vector<Point2D> obstacles = sim1->getKnownObstacleLocations();
	for (int i=0; i<obstacles.size(); ++i){
		nodeMap[obstacles[i].x][obstacles[i].y].blocked = true;
		std::cout << "Obstacle @" << obstacles[i].x << "," << obstacles[i].y
			<< " is " << (nodeMap[obstacles[i].x][obstacles[i].y].blocked?
			"blocked\n":"ERROR:NOT BLOCKED\n");
	}
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
    bool rebuild_path = first_move;
    if (first_move = true)
    	first_move = false;
    // Checked whether there is unrecored hidden obstacles. If so, rebuild path
    std::vector<Point2D> hiddenObstacles = r1->getLocalObstacleLocations();
    for (int i=0; i<hiddenObstacles.size(); ++i){
    	// std::cout << "Local obstacle detected @"
    	// 	<< hiddenObstacles[i].x << ','
    	// 	<< hiddenObstacles[i].y << std::endl;
    	// std::cout << "Blocked: " << (nodeMap[hiddenObstacles[i].x][hiddenObstacles[i].y].blocked?
    	// 	"true":"false") << std::endl;
    	if (nodeMap[hiddenObstacles[i].x][hiddenObstacles[i].y].blocked == false){
    		// std::cout << "Detected hidden obstacles!\n";
    		nodeMap[hiddenObstacles[i].x][hiddenObstacles[i].y].blocked = true;
    		rebuild_path = true;
    	}
    }
    Vector2D pos = r1->getPosition();
    if (rebuild_path)
    	if(!tryFindPath(pos.x, pos.y)){
    		std::cout << "Failed to find path. Program exited\n";
    		exit(-1);
    	}
    // std::cout << "Optimal action = " << nodeMap[pos.x][pos.y].next
    // 	<< " @" << pos.x << "," << pos.y << std::endl;
    return nodeMap[pos.x][pos.y].next;
    // return RobotAction::MOVE_UP;
}

bool
Project2::tryFindPath(float posX, float posY){
	// Do algorithm in reverse
	node* start = &nodeMap[target.x][target.y];
	node* current = start;
	node* goal = &nodeMap[posX][posY];
	// std::cout << "Try find path: from start = " << target.x << "," << target.y
	// 	<< " to " << posX << " " << posY << std::endl;
	// std::cout << "Start @" << start << " to @" << goal << std::endl;
	// Set start cost = 0
	current->g = 0.0f;
	std::unordered_set<node*> openSet;
	std::unordered_set<node*> closedSet;
	// Add start to current
	openSet.emplace(current);
	while (current != goal){
		// std::cout << "Loop @" << current->x << "," << current->y
		// 	<< " @" << current << std::endl;
		// Expand current node
		std::vector<node*> adNode;
		if (current->x > 0){
			adNode.push_back(&nodeMap[current->x - 1][current->y]);
			if (current->y > 0)
				adNode.push_back(&nodeMap[current->x - 1][current->y - 1]);
			if (current->y < width - 1)
				adNode.push_back(&nodeMap[current->x - 1][current->y + 1]);
		}
		if (current->x < height - 1){
			adNode.push_back(&nodeMap[current->x + 1][current->y]);
			if (current->y > 0)
				adNode.push_back(&nodeMap[current->x + 1][current->y - 1]);
			if (current->y < width - 1)
				adNode.push_back(&nodeMap[current->x + 1][current->y + 1]);
		}
		if (current->y > 0)
			adNode.push_back(&nodeMap[current->x][current->y - 1]);
		if (current->y < width - 1)
			adNode.push_back(&nodeMap[current->x][current->y + 1]);
		// std::cout << "Loop:adhere nodes generated\n";
		// Loop through nodes
		for (node* node : adNode){
			// check closed set, skip if inside
			if (closedSet.find(node) != closedSet.end())
				continue;
			// check open set, if better g(cost) then update g
			if (openSet.find(node) != openSet.end()){
				float cost = abs(current->x - node->x) + abs(current->y - node->y) == 1?
					1.0f:1.5f;
				float new_g = current->g + cost;
				if (node->g > new_g){
					node->g = new_g;
					node->next = getActon(node, current);
				}
			} else {
				if (node->blocked){
					// Add to closed set
					closedSet.emplace(node);
					continue;
				}
				node->next = getActon(node, current);
				float cost = abs(current->x - node->x) + abs(current->y - node->y) == 1?
					1.0f:1.5f;
				node->g = current->g + cost;
				// Calculate heuristic
				node->h = Heuristic(node->x, node->y, goal->x, goal->y);
				openSet.emplace(node);
			}
		}
		// Move current from open to close
		openSet.erase(current);
		closedSet.emplace(current);
		// If no node in open set, return false
		if (openSet.empty())
			return false;
		// Pick up next node with the least g+h value
		float min = width + height;
		for (node* node : openSet){
			if (node->g + node->h < min){
				current = node;
				min = node->g + node->h;
			}
		}
	}
	return true;
}