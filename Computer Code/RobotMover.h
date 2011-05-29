/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_ROBOTMOVER_H
#define EECE375_ROBOTMOVER_H

#include "SerialCom.h"

class RobotMover{
public:
	RobotMover( char* com_p );
	~RobotMover();

	void turnLeft( const int degrees );
	void turnRight( const int degrees );
	void goForward( const int distance );
	void goBackward( const int distance );
	void releaseBalls( void );
	void captureBalls( void );

private:
	Serial * comHandler;
	char sendBuffer[4];

};

#endif