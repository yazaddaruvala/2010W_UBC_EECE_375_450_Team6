#include "RobotMover.h"
#include "cv.h"
#include "highgui.h"
#define GO_FORWARD_CMD 'w'
#define TURN_LEFT_CMD 'a'
#define TURN_RIGHT_CMD 'd'
#define GO_BACKWARD_CMD 's'
#define RELEASE_BALL_CMD 'q'
#define CAPTURE_BALL_CMD 'c'

RobotMover::RobotMover( char* com_p ){
	comHandler = new Serial( com_p );
	sprintf(sendBuffer, "    ");
}

RobotMover::~RobotMover(){
	comHandler->~Serial();
}

void RobotMover::turnLeft( const int degrees ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );

	if (degrees < 4)
		sprintf(sendBuffer, "%c%.3d", TURN_LEFT_CMD, 4);
	else
		sprintf(sendBuffer, "%c%.3d", TURN_LEFT_CMD, degrees);
	comHandler->WriteData( sendBuffer, 4);
	cvWaitKey(1000 + 12*degrees);
	if (degrees>150)
		cvWaitKey(5*degrees);
}

void RobotMover::turnRight( const int degrees ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );
	
	if (degrees < 4)
		sprintf(sendBuffer, "%c%.3d", TURN_RIGHT_CMD, 4);
	else
		sprintf(sendBuffer, "%c%.3d", TURN_RIGHT_CMD, degrees);
	comHandler->WriteData( sendBuffer, 4);
	cvWaitKey(1000 + 12*degrees);
	if (degrees>150)
		cvWaitKey(5*degrees);
	
}

void RobotMover::goForward( const int distance ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );
	if (distance > 40){
		sprintf(sendBuffer, "%c%.3d", GO_FORWARD_CMD, 40);
		comHandler->WriteData( sendBuffer, 4);
		cvWaitKey(25*50+ 1000);
	}
	else{
		sprintf(sendBuffer, "%c%.3d", GO_FORWARD_CMD, distance);
		comHandler->WriteData( sendBuffer, 4);
		cvWaitKey(25*distance+ 1000);
	}

}

void RobotMover::goBackward( const int distance ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );

	sprintf(sendBuffer, "%c%.3d", GO_BACKWARD_CMD, distance);
	puts(sendBuffer);
	comHandler->WriteData( sendBuffer, 4);
	cvWaitKey(20*distance+1000);
}

void RobotMover::releaseBalls( void ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );

	sprintf(sendBuffer, "%c%.3d", RELEASE_BALL_CMD, 0);
	comHandler->WriteData( sendBuffer, 4);
	cvWaitKey(2000);
	sprintf(sendBuffer, "%c%.3d", RELEASE_BALL_CMD, 255);
	comHandler->WriteData( sendBuffer, 4);
	

}

void RobotMover::captureBalls( void ){
	if ( !comHandler->IsConnected() )
		printf( "Com with the robot is not availible\n" );

	sprintf(sendBuffer, "%c%.3d", CAPTURE_BALL_CMD, 0);
	comHandler->WriteData( sendBuffer, 4);
}