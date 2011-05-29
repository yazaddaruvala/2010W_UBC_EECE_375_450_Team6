/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_FIELDNODE_H
#define EECE375_FIELDNODE_H

#include "Node.h"

class FieldNode :
	public Node
{
public:
	FieldNode( const int xp, const int yp, const int rad );
	~FieldNode(void);

	int getRadius( void ) const;
	void setRadius( const int rad );

private:
	int radius;

};

#endif