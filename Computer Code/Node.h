/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_NODE_H
#define EECE375_NODE_H

class Node
{
public:
	Node( const int xp, const int yp );
	~Node( void );

	int getxPos( void ) const;
	int getyPos( void ) const;
	void setxPos( int x );
	void setyPos( int y );

private:
	int xPos;
	int yPos;
};

#endif