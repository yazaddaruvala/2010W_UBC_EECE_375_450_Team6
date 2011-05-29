/*
* Written by Yazad Daruvala
*/
#include "Node.h"


Node::Node( const int xp, const int yp )
{
	xPos = xp;
	yPos = yp;
}


Node::~Node( void )
{
}

int Node::getxPos( void ) const
{
	return xPos;
}

int Node::getyPos( void ) const
{
	return yPos;
}

void Node::setxPos( int x )
{
	xPos = x;
}

void Node::setyPos( int y )
{
	yPos = y;
}
