/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_ROBOTPARTPROCESSOR_H
#define EECE375_ROBOTPARTPROCESSOR_H
#include <cv.h>
#include <highgui.h>
#include <string>
#include <boost/thread.hpp>

#include "Processor.h"

class RobotPartProcessor: public Processor
{
public:
	RobotPartProcessor( char* name_p, CvScalar color_p, const int hl, const int sl, const int vl, const int hh, const int sh, const int vh );
	~RobotPartProcessor( void );

	void process( vector<IplImage *> *input );
	void configure( vector<IplImage *> *input );
	
private:
	int hueLower, hueUpper;
	int satLower, satUpper;
	int valLower, valUpper;
	int minRadius, maxRadius;

	char* name;

	boost::mutex bounds_Mutex;
};

#endif
