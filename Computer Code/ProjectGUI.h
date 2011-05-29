/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_GUI_H
#define EECE375_GUI_H
#include <vector>
using namespace std;

#include <cv.h>
#include <highgui.h>
using namespace cv;

#include <boost/thread.hpp>
using namespace boost;

#include "FrameCapturer.h"
#include "ImageProcessor.h"

class ProjectGUI{
public:
	ProjectGUI( FrameCapturer * cap_p );
	~ProjectGUI( void );
	void start( void );
	void stop( void );

	void updateOverlay( ImageProcessor * processor );

private:
	FrameCapturer * guiFrameCapturer;
	IplImage * displayImg;
	IplImage * overlayImg;
	IplImage * overlayMask;
	IplImage * guiBackground;

	thread ShowImg_Thread;
	thread OverlayImg_Thread;
	mutex overlayImg_Mutex;
	mutex showImg_Mutex;
	void showImage( void );
	void overlayImage( void );
	void displayImage(IplImage * img_p);
};

#endif