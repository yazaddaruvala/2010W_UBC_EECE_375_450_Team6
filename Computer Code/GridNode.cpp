#include "GridNode.h"
#include <math.h>

GridNode::GridNode( const int xp, const int yp, const int d, const int p ): Node( xp, yp )
{
	level=d;
	priority=p;
}

/*GridNode( const FieldNode node, const int d, const int p ): Node( node.getxPos(), node.getyPos() )
{
	level=d;
	priority=p;
}
*/

GridNode::~GridNode(void)
{
}

int GridNode::getLevel() const
{
	return level;
}

int GridNode::getPriority() const
{
	return priority;
}

void GridNode::updatePriority(const int & xDest, const int & yDest)
{
     priority = level + 10*estimate(xDest, yDest); //A*
}

// give better priority to going strait instead of diagonally
void GridNode::nextLevel(const int & i) // i: direction
{
	int dir = 4;
     level+=(dir==4?(i%2==0?1:2000):180);
}

// Estimation function for the remaining distance to the goal.
const int & GridNode::estimate(const int & xDest, const int & yDest) const
{
	static int xd, yd, d;
	xd=xDest-getxPos();
	yd=yDest-getyPos();

	// Euclidian Distance
	d=static_cast<int>(sqrt(xd*xd+yd*yd*1.0));

	// Manhattan distance
	//d=abs(xd)+abs(yd);

	// Chebyshev distance
	//d=max(abs(xd), abs(yd));

	return(d);
}