/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_ROBOTPROCESSOR_H
#define EECE375_ROBOTPROCESSOR_H
#include <cv.h>
#include <highgui.h>
#include <boost/thread.hpp>

#include "FieldNode.h"
#include "RobotPartProcessor.h"

class RobotProcessor
{
public:
	RobotProcessor( int type_p = 0 );
	~RobotProcessor( void );

	void process( vector<IplImage *> *input );
	void configure( vector<IplImage *> *input );
	void overlayImage( IplImage * overlayImg,  IplImage * overlayMask,  const int shiftX = 0, const int shiftY = 0 );
	void updateData( vector<FieldNode> *ourRobot );

private:
	RobotPartProcessor *frontProcessor;
	RobotPartProcessor *backProcessor;
	int type;

	int Xfront, Yfront;
	int Xback, Yback;
	boost::mutex data_Mutex;

};

#endif