#ifndef EECE375_MOTORCONTROLER_H
#define EECE375_MOTORCONTROLER_H

#include <stdlib.h>
#include <WProgram.h>

class Motor{
public:
	Motor( const int Power_p,  const int Select_p, const int Speed_p );
	~Motor( void );
  
	void run( bool dir );
	void stopRun( void );
	void resumeRun( void );
	void pauseRun( void );

	int getSpeed( void );

private:
	int Power;
	int Select;
	int Speed;
	bool dir;
};

class MotorControler{
public:
	MotorControler( Motor * Right_p, Motor * Left_p );
	~MotorControler( void );
  
	void turnRight( const int dist );
	void turnLeft( const int dist );
	void goForward( const int dist );
	void goBackward( const int dist );
	void resume( void );
	void pause( void );
	void slightRight( void );
	void slightLeft( void );
	void stopRun( void );

	void updateRightEncoder( void );
	void updateLeftEncoder( void );

	void allignWheels( void );
	int checkState( void );

private:
	int movementFlag;
	Motor * Right;
	Motor * Left;
	volatile long int rightEncoder;
	volatile long int leftEncoder;
};

#endif
