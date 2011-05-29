/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_FRAMECAPTURER_H
#define EECE375_FRAMECAPTURER_H
#include <highgui.h>
#include <boost/thread.hpp>

using namespace cv;
using namespace boost;

class FrameCapturer{
public:
	FrameCapturer( const char *filename, const unsigned int fps = 1000 );
	~FrameCapturer( void );
	
	void setFPS( const unsigned int fps );
	IplImage * getFrame( void );
	
	unsigned int getFPS( void ) const;
	CvSize getFrameSize( void ) const;

private:
	CvCapture *InputVideo;
	CvSize videoFrameSize;
	unsigned int delay;

	void GrabFrames( void );
	IplImage * tempImg;

	thread grabFrames_thread;
	mutex mutexQueryFrame;
};

#endif