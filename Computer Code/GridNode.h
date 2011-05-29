#pragma once
#include "Node.h"

class GridNode :
	public Node
{
public:
	GridNode( const int xp, const int yp, const int d, const int p );
	~GridNode(void);

    int getLevel() const;
    int getPriority() const;

    void updatePriority(const int & xDest, const int & yDest);
    // give better priority to going strait instead of diagonally
    void nextLevel(const int & i); // i: direction
    // Estimation function for the remaining distance to the goal.
    const int & estimate(const int & xDest, const int & yDest) const;

	// Determine priority (in the priority queue)
	friend bool operator < ( const GridNode & lhs,  const GridNode & rhs )
	{
		return (lhs.getPriority() > rhs.getPriority());
	}
private:
	// total distance already travelled to reach the node
	int level;

    // priority = level + remaining distance estimate
    int priority;  // smaller: higher priority
};

