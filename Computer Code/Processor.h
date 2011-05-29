/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_PROCESSOR_H
#define EECE375_PROCESSOR_H
#include <cv.h>
#include <highgui.h>

#include <boost/thread.hpp>

#include <vector>
using namespace std;

#include "FieldNode.h"
#include "SegmentFinder.h"

class Processor
{
public:
	Processor( void );
	~Processor( void );
	
	CvScalar getColor( void ) const;
	void overlayImage( IplImage * overlayImg,  IplImage * overlayImgBinary, const int shiftX = 0, const int shiftY = 0 );
	void updateData( vector<FieldNode> * input );
	vector<FieldNode> *getData( void );

protected:
	SegmentFinder *contourFinder;
	CvScalar color;
	vector<FieldNode> *data;
	boost::mutex data_Mutex;
	
	static IplImage * arrayToThreshed( vector<IplImage *> *input, CvScalar lowerBound, CvScalar upperBound );
};

#endif
