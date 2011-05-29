/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_BALLPROCESSOR_H
#define EECE375_BALLPROCESSOR_H
#include <cv.h>
#include <highgui.h>
#include <boost/thread.hpp>

#include "Processor.h"
class BallProcessor: public Processor
{
public:
	BallProcessor( void );
	~BallProcessor( void );

	void process( vector<IplImage *> *input );
	void configure( vector<IplImage *> *input );
	
private:
	int hueLower, hueUpper;
	int satLower, satUpper;
	int valLower, valUpper;
	int minRadius, maxRadius;
	
	boost::mutex bounds_Mutex;

};

#endif
