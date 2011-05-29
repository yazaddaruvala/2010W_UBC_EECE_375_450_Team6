#include "PathFinder.h"
#include <vector>
#include <math.h>

using namespace std;

#define PI 3.14159265

inline static long double square(int a)
{
	return a * a;
}

PathFinder::PathFinder( RobotMover * robo_p )
{
	robo = robo_p;

	counter_k = 0;
	ball_counter=0;		
	smallest = 100;
	dx[0]=1;
	dx[1]=0;
	dx[2]=-1;
	dx[3]=0;
	dy[0]=0;
	dy[1] = 1;
	dy[2] = 0;
	dy[3] = -1;

	inputBalls = new vector<FieldNode>;
	inputObstacles = new vector<FieldNode>;
	ourRobot = new vector<FieldNode>;
	enemyRobot = new vector<FieldNode>;
	updated_Mutex.lock();
	findPath_Thread = thread(&PathFinder::findPath, this);
}


PathFinder::~PathFinder(void)
{
	findPath_Thread.interrupt();
	findPath_Thread.join();
}

void PathFinder::updateData( ImageProcessor * processor ){
	pathData_Mutex.lock();
		x_scale = processor->getSize().width;
		y_scale = processor->getSize().height;
	
		start_at_top = !processor->getHomeLocation();
		inputBalls->clear();
		inputObstacles->clear();
		ourRobot->clear();
		enemyRobot->clear();
		processor->updateData( inputBalls, inputObstacles, ourRobot, enemyRobot );
		updated_Mutex.unlock();
	pathData_Mutex.unlock();
}

void PathFinder::findPath( void ){
	while(!findPath_Thread.interruption_requested()){
		updated_Mutex.lock();

		int balls_needed;
		int balls_picked_up = 0;
		int path_number= 0;
		int z = 0;
		int balls_possible = 0;
		int balls_possible_highest = 0;
		
		int node_x[50]={-1}; // array of GridNodes
		int node_y[50]={-1};
		int node_d[50]={-1};
		int node_count=0;


		bool path_status = false;  //sets default path status as false
		bool pickup = false;

		int ball_start = 0;
		int ball_number = 0;							// which ball is the closest

		x_goal = x_coord/2;
		y_goal = 6;
		if(start_at_top)
			y_goal = y_coord-6;

		int x_pos=0; //current x position 
		int y_pos=0; //current y position
		int curr_state=0;	//position of current character
		int prev_state=0;	// previous state of the bot
		double initial_state=0;	//the angle of the initial state of the bot

		int xA=0, yA=0, xB=0, yB=0;

		double unsorted_ball [max][3];					// the x and y and the radius coordinates for the balls
		double unsorted_front_robot [1][2];				// the x and y coordinates for the front strip of robot
		double unsorted_back_robot [1][2];				// the x and y coordinates for the back strip of the robot
		double unsorted_robot[1][3];

		int short_d[10][3];

		bool impos_route = false;

		ball_counter = 0;
		counter_k = 0;

		pathData_Mutex.lock();

		unsorted_back_robot[0][0] = ourRobot->back().getxPos();
		unsorted_back_robot[0][1] = ourRobot->back().getyPos();
		unsorted_front_robot[0][0] = ourRobot->front().getxPos();
		unsorted_front_robot[0][1] = ourRobot->front().getyPos();

		
		unsorted_enemy[0][0] = enemyRobot->back().getxPos();
		unsorted_enemy[0][1] = enemyRobot->back().getyPos();
		unsorted_enemy[1][0] = enemyRobot->front().getxPos();
		unsorted_enemy[1][1] = enemyRobot->front().getyPos();
		
		if(true){
			int i = 0;
			for( vector<FieldNode>::iterator ballIterator = inputBalls->begin();  ballIterator != inputBalls->end(); ++ballIterator, i++){
				unsorted_ball[i][0]= ballIterator->getxPos();
				unsorted_ball[i][1]= ballIterator->getyPos();
			}
			unsorted_ball[i][0]= -1;
			unsorted_ball[i][1]= -1;
		}

		if(true){
			int i = 0;
			for( vector<FieldNode>::iterator obstacleiterator = inputObstacles->begin(); obstacleiterator != inputObstacles->end(); ++obstacleiterator, i++){
				unsorted_obstacles[i][0]= obstacleiterator->getxPos();
				unsorted_obstacles[i][1]= obstacleiterator->getyPos();
				unsorted_obstacles[i][2]= obstacleiterator->getRadius();
			}
			unsorted_obstacles[i][0]= -1;
			unsorted_obstacles[i][1]= -1;
		}
		pathData_Mutex.unlock();


		// take the back of the robot and the front of the robot and put it in the robot
		unsorted_robot[0][0] = (unsorted_front_robot[0][0]+unsorted_back_robot[0][0])/2;
		unsorted_robot[0][1] = (unsorted_front_robot[0][1]+unsorted_back_robot[0][1])/2;


		// determine the direction the robot is facing
		unsorted_robot[0][2] = angle_check( (int) unsorted_back_robot[0][0], (int)unsorted_front_robot[0][0], (int)unsorted_back_robot[0][1], (int) unsorted_front_robot[0][1]);

		// convert robot from coordinates to grid
		sorted_robot[0][0]=(int) unsorted_robot[0][0]*x_coord/x_scale;
		sorted_robot[0][1]=(int) unsorted_robot[0][1]*y_coord/y_scale;

		// convert enemy from coordinates to grid
		sorted_enemy[0][0]=(int) (((unsorted_enemy[0][0] + unsorted_enemy[1][0])/2) *x_coord/x_scale);
		sorted_enemy[0][1]=(int) (((unsorted_enemy[0][1] + unsorted_enemy[1][1])/2) *y_coord/y_scale);

		if( unsorted_back_robot[0][0] <= 1 && unsorted_back_robot[0][1] <= 1 &&  unsorted_front_robot[0][0] <= 1 && unsorted_front_robot[0][1] <= 1){
			continue;
		}
		else if( unsorted_front_robot[0][0] <= 1 && unsorted_front_robot[0][1] <= 1){
			robo->goBackward(10);
			continue;
		}
		else if( unsorted_back_robot[0][0] <= 1 && unsorted_back_robot[0][1] <= 1){
			robo->goForward(10);
			continue;
		}

		for (int i= 0; unsorted_ball[i][0] !=-1; i++)
		{

			sorted_ball[ball_counter][0] =(int) unsorted_ball[i][0]*x_coord/x_scale;
			sorted_ball[ball_counter][1] =(int) unsorted_ball[i][1]*y_coord/y_scale;
			if (sorted_ball[ball_counter][0] <= 16 && sorted_ball[ball_counter][0] > 0)
				sorted_ball[ball_counter][0]--;
			if (sorted_ball[ball_counter][0] >= 32 && sorted_ball[ball_counter][0] < 48)
				sorted_ball[ball_counter][0]++;
			if (sorted_ball[ball_counter][0]<= (sorted_robot[0][0] + 3) && sorted_ball[ball_counter][0] >= (sorted_robot[0][0] -3) && sorted_ball[ball_counter][1]<= (sorted_robot[0][1]+3) && sorted_ball[ball_counter][1] >= (sorted_robot[0][1] -3))
			{	
				sorted_ball[ball_counter][0] = 1;
				sorted_ball[ball_counter][1] = 1;
				ball_counter --;
			}
			ball_counter++;
		}

		for(int y=0;y<y_coord;y++)
		{

			for(int x=0;x<x_coord;x++) map[x][y]=0;
		}

		plot_obstacles();

/*		determine_ball(0, ball_counter, ball_counter);
	
		if (ball_counter <=4)
			balls_needed = ball_counter;
		else if (ball_counter >4)
			balls_needed = ball_counter;

		while(!impos_route && ball_counter >0)
		{
			z++;
			impos_route = true;
			balls_possible = 0;
			for ( int i = 1; i <= (balls_needed); i++ )
			{
				if(i == 1)
				{

					xB = sorted_ball[shortest[z][i]][0];
					yB = sorted_ball[shortest[z][i]][1];
				}
				else
				{

					xB = sorted_ball[shortest[z][i]][0];
					yB = sorted_ball[shortest[z][i]][1];
				}
				xA = sorted_robot[0][0];
				yA = sorted_robot[0][0];
				string route = pathFind(xA, yA, xB, yB);
				balls_possible++;
				if (route == "")
				{
					balls_possible--;
					impos_route = false;
				}
			}
			if (balls_possible > balls_possible_highest)
				balls_possible_highest = balls_possible;

			if (z >= path_counter && balls_possible_highest<=0)
			{
				ball_counter = 0;
				break;
			}
			else if (z>= path_counter && balls_possible_highest>0)
			{
				z = 0;
				determine_ball(0, balls_possible_highest, ball_counter);
				while(!impos_route)
				{
					z++;
					impos_route = true;
					balls_possible = 0;
					for ( int i = 0; i < balls_possible_highest; i++ )
					{
						if (z>path_counter)
							//return;
							continue;
						if(i == 0)
						{
							xA = sorted_robot[0][0];
							yA = sorted_robot[0][1];
							xB = sorted_ball[shortest[z][1]][0];
							yB = sorted_ball[shortest[z][1]][1];
						}
						else
						{
							xA = sorted_ball[shortest[z][i]][0];
							yA = sorted_ball[shortest[z][i]][1];
							xB = sorted_ball[shortest[z][i+1]][0];
							yB = sorted_ball[shortest[z][i+1]][1];
						}
						string route = pathFind(xA, yA, xB, yB);					
						if (route == "")
						{
							impos_route = false;
						}

					}
				}
				break;
			}
		}
*/

		for(int i = 0; i < ball_counter; i++)
		{
			short_d[i][0] = abs(sorted_robot[0][0] - sorted_ball[i][0]) + abs(sorted_robot[0][1] - sorted_ball[i][1]);
			short_d[i][1] = sorted_ball[i][0];
			short_d[i][2] = sorted_ball[i][1];
		}

		int tmp;
		for (int b = 0; b< ball_counter; b++)
		{
			for( int c = 1; c< ball_counter; c++)
			{
				if(short_d[c-1][0]> short_d[c][0])
				{
					for(int a = 0; a< 3; a++)
					{
						tmp = short_d[c][a];
						short_d[c][a] = short_d[c-1][a];
						short_d[c-1][a] = tmp;
					}
				}
			}
		}

		balls_possible = ball_counter;

		for (balls_picked_up = 0; balls_picked_up < (ball_counter+1); balls_picked_up++)
		{
			node_count = 0;

			if (ball_counter == 0|| balls_possible == 0)
			{
				xB = x_goal;
				yB = y_goal;
			}
			else
			{
				xB = short_d[balls_picked_up][1];
				yB = short_d[balls_picked_up][2];
			}
			if ((ball_counter == 0 || balls_possible == 0) && sorted_robot[0][0] <= x_goal+2 && sorted_robot[0][0] >= x_goal-2 &&
				sorted_robot[0][1] <= y_goal+2 && sorted_robot[0][1] >= y_goal-2 )
				{
					if(start_at_top && unsorted_robot[0][2] < 280 && unsorted_robot[0][2] > 260)
					{
						robo->releaseBalls();
						cout << "releasing ball" << endl;
					}
					else if(!start_at_top && unsorted_robot[0][2] < 100 && unsorted_robot[0][2] > 80)
					{
						robo->releaseBalls();
						cout << "releasing ball" << endl;
					}
				}
			xA = sorted_robot[0][0];
			yA = sorted_robot[0][1];
	//		cout << "xA : " << xA << " yA : " << yA <<endl;
			x_pos = xA;
			y_pos = yA;

			cout<<"Map Size (X,Y): "<<x_coord<<","<<y_coord<<endl;
			cout<<"Start: "<<xA<<","<<yA<<endl;
			cout<<"Finish: "<<xB<<","<<yB<<endl;
			// get the route
			clock_t start = clock();
			string route=pathFind(xA, yA, xB, yB);

			if( route == "" && ball_counter != 0)
			{
				balls_possible_highest = 0;
				balls_possible--;
			}
			else if(route == "" && balls_possible == 0 && !(sorted_robot[0][0] <= x_goal+2 && sorted_robot[0][0] >= x_goal-2 && sorted_robot[0][1] <= y_goal+3 && sorted_robot[0][1] >= y_goal-3))
			{
				robo->goBackward(10);
				continue;
			}
			else if(route.length()>0)
			{
				balls_possible_highest = 1;
				clock_t end = clock();
				double time_elapsed = double(end - start);
				cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
				cout<<"Route:"<<endl;
				cout<<route<<endl<<endl;
				int j; char c;
				int x=xA;
				int y=yA;
				map[x][y]=2;
				for(int i=0;i<route.length();i++)
				{
					c =route.at(i);
					j=atoi(&c); 
					x=x+dx[j];
					y=y+dy[j];
					map[x][y]=3;
				}
				map[x][y]=4;

				//this section of code does an initial check to locate
				//the direction of movement from the very first GridNode
				//initialization with respect to 'S' pixel, runs only once
				node_x[node_count]=x_pos; //saving position as a GridNode
				node_y[node_count]=y_pos;
				node_count++;
				if( check_up(x_pos,y_pos)== true)  
				{curr_state=3; y_pos--;
				} 
				else if( check_down(x_pos,y_pos) == true)
				{curr_state=1; y_pos++;
				}
				else if( check_left(x_pos,y_pos) == true)
				{curr_state=2; x_pos--;
				}
				else if( check_right(x_pos,y_pos) == true)
				{curr_state=4; x_pos++;
				} 
                                                                                                                                             				while (1)
				{
					if (map[x_pos][y_pos] == 4)
						{
							node_x[node_count]=x_pos;
							node_y[node_count]=y_pos;
							node_count++;
							break;
						}
					if( (check_up(x_pos,y_pos)== true) && ( curr_state == 3 ) ){y_pos--;} 
					else if( (check_down(x_pos,y_pos) == true ) && ( curr_state == 1) ){y_pos++;}
					else if( (check_left(x_pos,y_pos) == true ) && ( curr_state == 2) ){x_pos--;} 
					else if( (check_right(x_pos,y_pos) == true ) && ( curr_state == 4) ){x_pos++;} 
					else{ 
						//direction changes here, next path position gets updated
						node_x[node_count]=x_pos; //saving position as a GridNode
						node_y[node_count]=y_pos;
						node_count++;
						prev_state=curr_state;
						curr_state=change_state(x_pos,y_pos,prev_state);

						switch(curr_state){
						case 1:
							y_pos++;
							break;

						case 2:
							x_pos--;
							break;

						case 3:
							y_pos--;
							break;

						case 4:
							x_pos++;
							break;
						}
					}
				}
				// displays balls
				for(int i =0; i< ball_counter; i++)
				{
					if (sorted_ball[i][0]>=0 && sorted_ball[i][0] <=60 && sorted_ball[i][1]>=0 && sorted_ball[i][1] <= 48)
					map[sorted_ball[i][0]][sorted_ball[i][1]] = 6;
				}

				// display the map with the route
	/*			for(int y=0;y<y_coord;y++)
				{
					for(int x=0;x<x_coord;x++)
					if(map[x][y]==0)
						cout<<".";
						else if(map[x][y]==1)
						cout<<"O"; //obstacle
						else if(map[x][y]==2)
						cout<<"S"; //start
						else if(map[x][y]==3)
						cout<<"R"; //route
						else if(map[x][y]==4)
						cout<<"F"; //finish
						else if(map[x][y]==5)
						cout<<"N"; // GridNode
						else if(map[x][y]==6)
						cout<<"B"; // ball
						cout<<endl;
				}
	*/			
				if ( node_count > 2 && !( balls_possible == 0 && sorted_robot[0][0] <= x_goal+2 && sorted_robot[0][0] >= x_goal-2 && sorted_robot[0][1] <= y_goal+2 && sorted_robot[0][1] >= y_goal-2))
				{
					pickup = false;
					if (balls_possible_highest == 0)
					{
						if(path_check (node_x[0], node_x[2], node_y[0], node_y[2]))
							send_instructions(node_x[0],node_x[2], node_y[0], node_y[2], unsorted_robot[0][2], false);
						else 
							send_instructions(node_x[0],node_x[1], node_y[0], node_y[1], unsorted_robot[0][1], false);
					}
					else
					{
						for ( int i = 0; i < 1; i++)
						{
							if (path_check (node_x[0], node_x[2], node_y[0], node_y[2]))
							{
								if((i+2) >= (node_count-2))
									pickup = true;
								send_instructions(node_x[i], node_x[i+2], node_y[i], node_y[i+2], unsorted_robot[0][2], pickup);
							}
								// find angle from current node to that next node
							else
							{
								send_instructions(node_x[i], node_x[i+1], node_y[i], node_y[i+1], unsorted_robot[0][2], pickup);
							}						
						}
					}
				}
				else if (node_count <=2 && !( balls_possible == 0 && sorted_robot[0][0] <= x_goal+3 && sorted_robot[0][0] >= x_goal-3 && sorted_robot[0][1] <= y_goal+3 && sorted_robot[0][1] >= y_goal-3))
				{
					if (balls_possible_highest != 0)
						pickup = true;
					send_instructions(sorted_robot[0][0], node_x[1], sorted_robot[0][1], node_y[1], unsorted_robot[0][2], pickup);
				}
				break;
			}
		}
		if( (balls_possible == 0 || ball_counter == 0) && sorted_robot[0][0] <= x_goal+2 && sorted_robot[0][0] >= x_goal-2 && sorted_robot[0][1] <= y_goal+2 && sorted_robot[0][1] >= y_goal-2)
		{
			if (start_at_top)
			{
				send_instructions(sorted_robot[0][0],sorted_robot[0][0], sorted_robot[0][1], (sorted_robot[0][1]-1), unsorted_robot[0][2], false);
				cout << "Turning to dump" << endl;
	/*			if (unsorted_robot[0][2] < 280 && unsorted_robot[0][2] > 260)
				{
					robo->releaseBalls();
					cout << "releasing ball" << endl;
				}
				*/
			}
			else
			{
				send_instructions(sorted_robot[0][0],sorted_robot[0][0], sorted_robot[0][1], (sorted_robot[0][1]+1), unsorted_robot[0][2], false);
//				if (unsorted_robot[0][2] < 100 && unsorted_robot[0][2] > 80)
//				{
//					robo->releaseBalls();
//					cout << "releasing ball" << endl;
//				}
			}
//				cout<< angle_check(sorted_robot[0][0], sorted_robot[0][0], sorted_robot[0][1], (sorted_robot[0][1]- 4)) << endl;
		}


	}
}


// A-star algorithm.
// taken and altered from ahttp://code.activestate.com/recipes/577457-a-star-shortest-path-algorithm/
// The route returned is a string of direction digits.
string PathFinder::pathFind( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )

{

    static priority_queue<GridNode> pq[2]; // list of open (not-yet-tried) GridNodes
    static int pqi; // pq index
    static GridNode* n0;
    static GridNode* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // reset the GridNode maps
    for(y=0;y<y_coord;y++)
    {
        for(x=0;x<x_coord;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start GridNode and push into list of open GridNodes
    n0=new GridNode(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open GridNodes map

    // A* search
    while(!pq[pqi].empty())
    {
        // get the current GridNode w/ the highest priority
        // from the list of open GridNodes
        n0=new GridNode( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), 
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remove the GridNode from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed GridNodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish) 
        {
            // generate the path from finish to start
            // by following the directions
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // garbage collection
            delete n0;
            // empty the leftover GridNodes
            while(!pq[pqi].empty()) pq[pqi].pop();           
            return path;
        }

        // generate moves (child GridNodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>x_coord-1 || ydy<0 || ydy>y_coord-1 || map[xdx][ydy]==1 
                || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child GridNode
                m0=new GridNode( xdx, ydy, n0->getLevel(), 
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent GridNode direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the GridNode
                    // by emptying one pq to the other one
                    // except the GridNode to be replaced will be ignored
                    // and the new GridNode will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx && 
                           pq[pqi].top().getyPos()==ydy))
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pq[pqi].pop(); // remove the wanted GridNode
                    
                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {                
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();       
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better GridNode instead
                }
                else delete m0; // garbage collection
            }
        }
        delete n0; // garbage collection
    }
    return ""; // no route found
}
bool PathFinder::check_up(int x_pos, int y_pos)// int map[x_coord][y_coord])
{
	int x_new= x_pos;
	int y_new= y_pos-1;

	if (y_new < 0) return false;

	if(map[x_new][y_new]==3 || map[x_new][y_new]==4){return true;}
	else {return false;}
}
bool PathFinder::check_down(int x_pos, int y_pos)//, int map[x_coord][y_coord])
{
	int x_new= x_pos;
	int y_new= y_pos+1;

	if (y_new > 47) return false;

	if((map[x_new][y_new]==3) || (map[x_new][y_new]==4)){return true;}
	else {return false;}
}
bool PathFinder::check_left(int x_pos, int y_pos) //, int map[x_coord][y_coord])
{
	int x_new= x_pos-1;
	int y_new= y_pos;

	if (x_new < 0) return false;

	if(map[x_new][y_new]==3 || map[x_new][y_new]==4){return true;}
	else {return false;}
}
bool PathFinder::check_right(int x_pos, int y_pos) //, int map[x_coord][y_coord])
{
	int x_new= x_pos+1;
	int y_new= y_pos;

	if (x_new > 47) return false;

	if(map[x_new][y_new]==3 || map[x_new][y_new]==4){return true;}
	else {return false;}
}
int PathFinder::change_state(int x_pos, int y_pos, int prev_state)
{
	if( check_up(x_pos,y_pos)== true && prev_state != 1)  
	{return 3;} 
	else if( check_down(x_pos,y_pos) == true && prev_state != 3)
	{return 1;}
	else if( check_left(x_pos,y_pos) == true && prev_state != 4)
	{return 2;} 
	else if( check_right(x_pos,y_pos) == true && prev_state != 2)
	{return 4;} 

}
bool PathFinder::path_check( int xA, int xB, int yA, int yB){

int angle=angle_check(xA, xB, yA, yB);  //checking the angle between the 2 nodes 
	//finding the slope between the 2 nodes
	if(xA == xB || yA == yB)
		return true;
	float slope = (float)(yB - yA) / ((xB - xA)); 
	int x_pos = xA;  //x_1
	int y_pos_end = yB; //y_2
	int y_incpt = yA; //y_1 - intercept for the slope function
	int y_pos = yA; //y_1

	if( angle >= 0 && angle <90)
	{
	
		while( y_pos < y_pos_end || x_pos < xB ){
			if (y_pos < y_pos_end)
				y_pos++;
			if (x_pos < xB)
				x_pos++;
			// y_pos = line_func(slope,x_pos,y_incpt);  //get y value from line function
			if( map[x_pos][y_pos]==1){return false;} //return false if obstacle detected
		//	x_pos++; //inc x_pos to continue
		}

		return true;

		//loop till end y_coord is reached
		//plug current x_pos into line_func to get y_pos
		//check corresponding position in map[x_pos][y_pos] for '1'
		//return true or false
	
	}
	else if ( angle >= 90 && angle <180)
	{
		while( y_pos < y_pos_end || x_pos > xB ){
			if (y_pos < y_pos_end)
				y_pos++;
			if (x_pos > xB)
				x_pos--;
		//y_pos = line_func(slope,x_pos,y_incpt);  //get y value from line function
			if( map[x_pos][y_pos]==1){return false;} //return false if obstacle detected
		//	x_pos--; //inc x_pos to continue
		}

		return true;
		
	}
	else if ( angle >= 180 && angle <270)
	{
		while( y_pos > y_pos_end || x_pos > xB ){
			if (y_pos > y_pos_end)
				y_pos--;
			if (x_pos > xB)
				x_pos--;
			//y_pos = line_func(slope,x_pos,y_incpt);  //get y value from line function
			if( map[x_pos][y_pos]==1){return false;} //return false if obstacle detected
			//x_pos--; //dec x_pos to continue
		}

		return true;
	
	}
	else if ( angle >= 270 && angle <360)
	{
		while( y_pos > y_pos_end || x_pos < xB ){
			if (y_pos > y_pos_end)
				y_pos--;
			if (x_pos < xB)
				x_pos++;
			//y_pos = line_func(slope,x_pos,y_incpt);  //get y value from line function
			if( map[x_pos][y_pos]==1){return false;} //return false if obstacle detected
			//x_pos++; //inc x_pos to continue
		}

		return true;
		
	}
}
int PathFinder::line_func(float slope, int x_pos, int y_intpt){
	
	float y_pos = (slope*x_pos)+y_intpt; //y=m*x+b
	int y_pos_int = (int)y_pos; //cast to int 
	return y_pos_int;

}
int PathFinder::angle_check(int x_1, int x_2, int y_1, int y_2){

	if ((x_2-x_1) == 0 && y_2> y_1)
		return 90;
	else if ( (x_2-x_1) == 0 && y_1> y_2)
		return 270;
	double angle = 0.0;
	if (x_2-x_1 != 0)
		 angle = atan(((double)abs(y_2-y_1))/(double)(abs(x_2-x_1))) * 180.0 /PI;
	else angle = 0;
	int int_angle = (int) angle;
	if (y_2>=y_1 && x_2>x_1)
		return int_angle;
	else if ( y_2>= y_1 && x_2< x_1)
		return 180 - int_angle;
	else if (y_2 <= y_1 && x_2< x_1)
		return int_angle + 180;
	else if (y_2 <= y_1 && x_2> x_1)
		return 360 -int_angle;

}
void PathFinder::send_instructions(int node_xA, int node_xB, int node_yA, int node_yB, double robotAngle, bool pickup)
{
	int distance;
	double distance_x, distance_y;
	int int_angle = (int)(angle_check(node_xA, node_xB, node_yA,node_yB) - robotAngle);
	if ((int_angle <0 && int_angle >(-180)) || int_angle >=180 )
	{
		if(int_angle >= 180)
		{
			int_angle = 360- int_angle;
		}
		int_angle = abs(int_angle);
		cout<< "turn left: " << int_angle << endl;
		robo->turnLeft(int_angle);
	}
	else if (int_angle> 0 || int_angle <= -180)
	{
		if(int_angle <= -180)
		{
			int_angle = 360 - abs(int_angle);
		}
		cout<< "turn right: " << int_angle << endl;
		robo->turnRight(int_angle);
	}

	if( int_angle < 10)
	{
		distance_x = (node_xB - node_xA)* 2 *2.54;
		distance_y = (node_yB - node_yA)* 2 *2.54;
		distance = (int)sqrt(((distance_x * distance_x)+(distance_y*distance_y)))-7;
		if(!pickup)
		{
			cout<< "go straight: " << distance << endl;
			robo->goForward(abs(distance));		// node_x[i+2], node_y[i+2]);
			if(abs(distance) < 5)
				robo->goForward(5);
		}
		else if(pickup)
		{
			cout<< "pick up ball: " << distance << endl;
			robo->goForward(abs(distance));
			if(abs(distance) < 5)
				robo->goForward(5);
		}

	}
}

void PathFinder::plot_obstacles()
{
	// this for loop creates a box around the box
	for (int i= 0; unsorted_obstacles[i][0] !=-1; i++)
	{
		for (int k = 0; k< 5; k++)
		{
			if (k == 0){		// centre box
				sorted_obstacles[counter_k+k][0]=(int)unsorted_obstacles[i][0] * x_coord/x_scale; 
				sorted_obstacles[counter_k+k][1]=(int)unsorted_obstacles[i][1] * y_coord/y_scale;
				if (sorted_obstacles[counter_k+k][0] < 18 && sorted_obstacles[counter_k+k][0] > 0)
					sorted_obstacles[counter_k+k][0]++;
				if (sorted_obstacles[counter_k+k][0] < 48 && sorted_obstacles[counter_k+k][0] > 30)
					sorted_obstacles[counter_k+k][0]--;
				if (sorted_obstacles[counter_k+k][1] < 18 && sorted_obstacles[counter_k+k][1] > 0)
					sorted_obstacles[counter_k+k][1]++;
				if (sorted_obstacles[counter_k+k][1] < 48 && sorted_obstacles[counter_k+k][1] > 30)
					sorted_obstacles[counter_k+k][1]--;
			}
			else if (k==1){		// left box creates a barricade of 9 boxes
				sorted_obstacles[counter_k+k][0]=(int) (((unsorted_obstacles[i][0]-(unsorted_obstacles[i][2]/(sqrt(2.0)))) * x_coord/x_scale)-3); 
				sorted_obstacles[counter_k+k][1]=(int) (((unsorted_obstacles[i][1]+(unsorted_obstacles[i][2]/(sqrt(2.0))))*y_coord/y_scale)+3);
				for(int j = 1; j<9 ; j++)
				{
					sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]);
					sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]-1);		
				}
				counter_k+=8;
			}
			else if(k==2){
				sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]+1);
				sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]);
				for(int j=1; j<8; j++)
				{
					sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]+1);
					sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]);
				}
				counter_k+=7;
			}
			else if(k==3){
				sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]);
				sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]+1);
				for(int j=1; j<8; j++)
				{
					sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]);
					sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]+1);
				}
				counter_k+=7;
			}
			else if(k==4){
				sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]-1);
				sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]);
				for(int j=1; j<8; j++)
				{
					sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]-1);
					sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]);
				}
				counter_k+=11;
			}
		}
	}
	for (int k = 0; k< 5; k++)
	{
		if (k == 0){		// centre box
			sorted_obstacles[counter_k+k][0]=sorted_enemy[0][0]; 
			sorted_obstacles[counter_k+k][1]=sorted_enemy[0][1];
		}
		else if (k==1){		// left box creates a barricade of 9 boxes
			sorted_obstacles[counter_k+k][0]=(int) (sorted_enemy[0][0]-6); 
			sorted_obstacles[counter_k+k][1]=(int) (sorted_enemy[0][1]+6);
			for(int j = 1; j<13 ; j++)
			{
				sorted_obstacles[counter_k+k+j][0]=(sorted_enemy[0][0]-6);
				sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]-1);		
			}
			counter_k+=12;
		}
		else if(k==2){
			sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]+1);
			sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]);
			for(int j=1; j<12; j++)
			{
				sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]+1);
				sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]);
			}
			counter_k+=11;
		}
		else if(k==3){
			sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]);
			sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]+1);
			for(int j=1; j<12; j++)
			{
				sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]);
				sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]+1);
			}
			counter_k+=11;
		}
		else if(k==4){
			sorted_obstacles[counter_k+k][0]=(sorted_obstacles[counter_k+k-1][0]-1);
			sorted_obstacles[counter_k+k][1]=(sorted_obstacles[counter_k+k-1][1]);
			for(int j=1; j<12; j++)
			{
				sorted_obstacles[counter_k+k+j][0]=(sorted_obstacles[counter_k+k+j-1][0]-1);
				sorted_obstacles[counter_k+k+j][1]=(sorted_obstacles[counter_k+k+j-1][1]);
			}
			counter_k+=15;
		}
	}

	for(int i =0; i< counter_k; i++)
	{
		if (sorted_obstacles[i][0]>=0 && sorted_obstacles[i][0] <=60 && sorted_obstacles[i][1]>=0 && sorted_obstacles[i][1] <= 48)
		map[sorted_obstacles[i][0]][sorted_obstacles[i][1]] = 1;
	}

	for(int i =0; i<y_coord ; i++)
	{
		map[2][i] = 1;
		map[x_coord-3][i] = 1;
		if(i % 3 == 1)
		{
			map[3][i] = 1;
			map[x_coord-4][i] = 1;
		}
	}
	for (int j =0; j<x_coord; j++)
	{
		map[j][0] = 1;
		map[j][y_coord-1] = 1;
/*		if(j % 3 == 0)
		{
			map[j][1] = 1;
			map[j][y_coord-2] = 1;
		}
*/	}
	
}

void PathFinder::determine_ball(int balls_picked_up, int balls_needed, int ball_counter)
{
	if (balls_needed >4)
	{
		balls_needed = 4;
	}
	int distance = 0;
	int array_counter = 0;
	int i=0;
	int f=0;
	int k=0;
	int l=0;


	for (i= 0; i< ball_counter; i++)
	{
		
		if ( balls_needed == 1)
		{
			shortest[array_counter][1] = i;
			if(path_check(sorted_robot[0][0], sorted_ball[i][0], sorted_robot[0][1], sorted_ball[i][1]))
				distance += (int)sqrt((double)((sorted_robot[0][0]- sorted_ball[i][0])*(sorted_robot[0][0]-sorted_ball[0][0]))+((sorted_robot[0][1]-sorted_ball[i][1])*(sorted_robot[0][1]-sorted_ball[i][1])))-6;
			else
				distance += abs(sorted_robot[0][0]-sorted_ball[i][0]) + abs(sorted_robot[0][1]-sorted_ball[i][1]);
			shortest[array_counter][0] = distance + abs(sorted_ball[i][0]- x_goal) +abs(sorted_ball[i][1] - y_goal);
			array_counter ++;
			distance = 0;
		}
		f=0;
		while(f<ball_counter && balls_needed >1)
		//for (f= 0; (f < ball_counter) && (balls_needed >1); f++);
		{		
			
			if( f != i && balls_needed == 2)
			{
				shortest[array_counter][1] = i;
				shortest[array_counter][2] = f;
				if(path_check(sorted_robot[0][0], sorted_ball[i][0], sorted_robot[0][1], sorted_ball[i][1]))
					distance += (int)sqrt((double)((sorted_robot[0][0]- sorted_ball[i][0])*(sorted_robot[0][0]-sorted_ball[i][0]))+((sorted_robot[0][1]-sorted_ball[i][1])*(sorted_robot[0][1]-sorted_ball[i][1])))-6;
				else
					distance += abs(sorted_robot[0][0]-sorted_ball[i][0]) + abs(sorted_robot[0][1]-sorted_ball[i][1]);
				if(path_check(sorted_ball[i][0], sorted_ball[f][0], sorted_ball[i][1], sorted_ball[f][1]))
					distance += (int)sqrt((double)((sorted_ball[i][0]- sorted_ball[f][0])*(sorted_ball[i][0]-sorted_ball[f][0]))+((sorted_ball[i][1]-sorted_ball[f][1])*(sorted_ball[i][1]-sorted_ball[f][1])))-6;
				else
					distance += abs(sorted_ball[i][0]-sorted_ball[f][0]) + abs(sorted_ball[i][1] - sorted_ball[f][1]);
				shortest[array_counter][0] = distance+ abs(sorted_ball[f][0]- x_goal) +abs(sorted_ball[f][1] - y_goal);
				array_counter++;
				distance = 0;
			}			
			k = 0;
			while(k<ball_counter && balls_needed>2 && f!=i)
			{
				
				if( k != i && k != f && balls_needed == 3)
				{
					shortest[array_counter][1] = i;
					shortest[array_counter][2] = f;
					shortest[array_counter][3] = k;
					if(path_check(sorted_robot[0][0], sorted_ball[i][0], sorted_robot[0][1], sorted_ball[i][1]))
						distance += (int)sqrt((double)((sorted_robot[0][0]- sorted_ball[i][0])*(sorted_robot[0][0]-sorted_ball[i][0]))+((sorted_robot[0][1]-sorted_ball[i][1])*(sorted_robot[0][1]-sorted_ball[i][1])))-6;
					else
						distance += abs(sorted_robot[0][0]-sorted_ball[i][0]) + abs(sorted_robot[0][1]-sorted_ball[i][1]);
					if(path_check(sorted_ball[i][0], sorted_ball[f][0], sorted_ball[i][1], sorted_ball[f][1]))
						distance += (int)sqrt((double)((sorted_ball[i][0]- sorted_ball[f][0])*(sorted_ball[i][0]-sorted_ball[f][0]))+((sorted_ball[i][1]-sorted_ball[f][1])*(sorted_ball[i][1]-sorted_ball[f][1])))-6;
					else
						distance += abs(sorted_ball[i][0]-sorted_ball[f][0]) + abs(sorted_ball[i][1] - sorted_ball[f][1]);
					if(path_check(sorted_ball[f][0], sorted_ball[k][0], sorted_ball[f][1], sorted_ball[k][1]))
						distance += (int)sqrt((double)((sorted_ball[f][0]- sorted_ball[k][0])*(sorted_ball[f][0]-sorted_ball[k][0]))+((sorted_ball[f][1]-sorted_ball[k][1])*(sorted_ball[f][1]-sorted_ball[k][1])))-6;
					else
						distance += abs(sorted_ball[f][0] - sorted_ball[k][0]) + abs(sorted_ball[f][1] - sorted_ball[k][1]);
					shortest[array_counter][0] = distance + abs(sorted_ball[k][0]- x_goal) +abs(sorted_ball[k][1] - y_goal);
					array_counter ++;
					distance = 0;
				}		
				l=0;
				while(l < ball_counter && balls_needed >3 && k!=i && k!=f) 
				{
					if( l != k && l != f && l!=i && balls_needed ==4)
					{
						shortest[array_counter][1] = i;
						shortest[array_counter][2] = f;
						shortest[array_counter][3] = k;
						shortest[array_counter][4] = l;
						if(path_check(sorted_robot[0][0], sorted_ball[i][0], sorted_robot[0][1], sorted_ball[i][1]))
							distance += (int)sqrt((double)((sorted_robot[0][0]- sorted_ball[i][0])*(sorted_robot[0][0]-sorted_ball[0][0]))+((sorted_robot[0][1]-sorted_ball[i][1])*(sorted_robot[0][1]-sorted_ball[i][1])))-6;
						else
							distance += abs(sorted_robot[0][0]-sorted_ball[i][0]) + abs(sorted_robot[0][1]-sorted_ball[i][1]);
						if(path_check(sorted_ball[i][0], sorted_ball[f][0], sorted_ball[i][1], sorted_ball[f][1]))
							distance += (int)sqrt((double)((sorted_ball[i][0]- sorted_ball[f][0])*(sorted_ball[i][0]-sorted_ball[f][0]))+((sorted_ball[i][1]-sorted_ball[f][1])*(sorted_ball[i][1]-sorted_ball[f][1])))-6;
						else
							distance += abs(sorted_ball[i][0]-sorted_ball[f][0]) + abs(sorted_ball[i][1] - sorted_ball[f][1]);
						if(path_check(sorted_ball[f][0], sorted_ball[k][0], sorted_ball[f][1], sorted_ball[k][1]))
							distance += (int)sqrt((double)((sorted_ball[f][0]- sorted_ball[k][0])*(sorted_ball[f][0]-sorted_ball[k][0]))+((sorted_ball[f][1]-sorted_ball[k][1])*(sorted_ball[f][1]-sorted_ball[k][1])))-6;
						else
							distance += abs(sorted_ball[f][0] - sorted_ball[k][0]) + abs(sorted_ball[f][1] - sorted_ball[k][1]);
						if(path_check(sorted_ball[k][0], sorted_ball[l][0], sorted_ball[k][1], sorted_ball[l][1]))
							distance += (int)sqrt((double)((sorted_ball[k][0]- sorted_ball[l][0])*(sorted_ball[k][0]-sorted_ball[l][0]))+((sorted_ball[k][1]-sorted_ball[l][1])*(sorted_ball[k][1]-sorted_ball[l][1])))-6;
						else
							distance += abs(sorted_ball[l][0]-sorted_ball[k][0]) + abs(sorted_ball[l][1] - sorted_ball[k][1]);
						shortest[array_counter][0] = distance + abs(sorted_ball[l][0]- x_goal) +abs(sorted_ball[l][1] - y_goal);
						array_counter ++;
						distance = 0;
					}
					l++;
				}
				k++;
			}
			f++;
		}
	}
	int tmp;
	path_counter = array_counter;
	for (int b = 0; b< array_counter; b++)
	{
		for( int c = 0; c< array_counter-b; c++)
		{
			if(shortest[c][0] <2)
				shortest[c][0] = 1000;
			if(shortest[c+1][0]<shortest[c][0])
			{
				for(int a = 0; a< 5; a++)
				{
					tmp = shortest[c+1][a];
					shortest[c+1][a] = shortest[c][a];
					shortest[c][a] = tmp;
				}
			}
		}
	}

}
