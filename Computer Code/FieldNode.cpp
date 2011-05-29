#include "FieldNode.h"

FieldNode::FieldNode( const int xp, const int yp, const int rad ): Node(xp, yp)
{
	radius = rad;
}


FieldNode::~FieldNode(void)
{
}

int FieldNode::getRadius( void ) const{
	return radius;
}

void FieldNode::setRadius( const int rad ){
	radius = rad;
}