/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_COLORPROCESSOR_H
#define EECE375_COLORPROCESSOR_H
#include <cv.h>
#include <highgui.h>
#include <boost/thread.hpp>

using namespace boost;

#include "FieldNode.h"
class ColorProcessor
{
public:
	ColorProcessor( CvSize size_p, CvScalar lowerBound_p = cvScalar(0,0,0), CvScalar upperBound_p  = cvScalar(0,0,0) );
	~ColorProcessor( void );

	void process( IplImage * input );
	void configure( IplImage * imp_p );
	vector<FieldNode> * getData( void );
	CvScalar getColor( void ) const;

private:
	CvScalar lowerBound, upperBound, color;
	CvMemStorage *storage;
	IplImage * dataImg;
	vector<FieldNode> *data;
	mutex data_Mutex;
};

#endif