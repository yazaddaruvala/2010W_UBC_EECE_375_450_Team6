/*
* Written by Yazad Daruvala
*/
#include "FrameCapturer.h"
#define videoFrameROI cvRect(165, 0, 415, 480)

FrameCapturer::FrameCapturer( const char *filename, const unsigned int fps ){
	InputVideo = cvCreateFileCapture( filename );
	if ( InputVideo == NULL )
	{
		fprintf(stderr, "Error: Couldn't allocate video capture. Out of memory?\n");
		exit(-1);
	}
	tempImg = cvQueryFrame( InputVideo );
	videoFrameSize = cvGetSize( tempImg );
	
	setFPS(fps);
	grabFrames_thread = thread( &FrameCapturer::GrabFrames, this );
}

FrameCapturer::~FrameCapturer(){
	grabFrames_thread.interrupt();
	grabFrames_thread.join();
	cvReleaseCapture(&InputVideo);
}

IplImage * FrameCapturer::getFrame( void ){
	mutexQueryFrame.lock();
	tempImg = cvRetrieveFrame( InputVideo );
	mutexQueryFrame.unlock();
	return tempImg;
}

void FrameCapturer::GrabFrames(){
	while(!grabFrames_thread.interruption_requested()){
		mutexQueryFrame.lock();
		if (!cvGrabFrame( InputVideo )){
			fprintf(stderr, "Error: Couldn't grab frame. Out of memory?\n");
			exit(-1);
		}
		mutexQueryFrame.unlock();
		cvWaitKey(delay);
	}
	return;
}

void FrameCapturer::setFPS(const unsigned int fps){
	if (fps > 1000)
		delay = 1;
	else
		delay = (1000/fps);
	
}

unsigned int FrameCapturer::getFPS( void ) const{
	return (1000/delay);
}

CvSize FrameCapturer::getFrameSize( void ) const{
	return videoFrameSize;
}