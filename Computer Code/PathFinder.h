#pragma once
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <fstream>
#include <cstdlib>

#include <boost/thread.hpp>

#include "ImageProcessor.h"
#include "RobotMover.h"
#include "GridNode.h"

#define x_coord 48
#define y_coord 48


#define max 100

class PathFinder
{
public:
	PathFinder( RobotMover * robo_p );
	~PathFinder( void );

	void updateData( ImageProcessor * processor );
private:
	RobotMover * robo;

	vector<FieldNode> * inputBalls;
	vector<FieldNode> * inputObstacles;
	vector<FieldNode> * ourRobot;
	vector<FieldNode> * enemyRobot;

	boost::thread findPath_Thread;
	boost::mutex pathData_Mutex;
	boost::mutex updated_Mutex;
	void findPath( void );

	void send_instructions(int node_xA, int node_xB, int node_yA, int node_yB, double robotAngle, bool pickup);
	
	bool start_at_top;
	string PathFinder::pathFind( const int & xStart, const int & yStart, const int & xFinish, const int & yFinish );
	void plot_obstacles();
	bool check_up(int , int);
	bool check_down(int , int);
	bool check_left(int , int); 
	bool check_right(int , int); 
	int change_state(int, int, int);
	bool path_check( int xA, int xB, int yA, int yB);
	int line_func(float slope, int x_pos, int y_intpt);
	int angle_check(int , int, int , int);
	void determine_ball(int, int, int);

	int counter_k;									// a counter that keeps track of the number of designated obstaclrs
	int ball_counter;								// a counter that keeps track of all balls
	int smallest ;									// compare and sort which of the balls are the smallest
	int path_counter;
	int x_goal;
	int y_goal;

	int sorted_ball [max][2];						// converted data from coordinates to grid 
	int sorted_enemy[1][2];							// converted data 
	int sorted_robot [2][2];						// converted data from coordinates to robot
	int sorted_direction[max][2];					// converted data from direction to direction
	int sorted_obstacles [2000][2];					// converted a single point to the multiple obstacles as coordinates to evade
	double unsorted_obstacles [max][3];
	double unsorted_enemy[2][2];
	int shortest[2000][5];

	int map[x_coord][y_coord];
	int closed_nodes_map[x_coord][y_coord]; // map of closed (tried-out) nodes
	int open_nodes_map[x_coord][y_coord]; // map of open (not-yet-tried) nodes
	int dir_map[x_coord][y_coord]; // map of directions
	
	static const int dir=4; // number of possible directions to go at any position

	int dx[4];	//={1, 0, -1, 0};
	int dy[4];
	
	int x_scale;
	int y_scale;

};

