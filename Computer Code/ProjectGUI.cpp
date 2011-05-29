/*
* Written by Yazad Daruvala
*/
#include "ProjectGUI.h"

#define FRAME_POSITION 10, 116
#define FRAME_SIZE 700, 700

ProjectGUI::ProjectGUI( FrameCapturer * cap_p ){
	guiFrameCapturer = cap_p;
	displayImg = cvCreateImage( cap_p->getFrameSize(), 8, 3);
	overlayImg = cvCreateImage( cap_p->getFrameSize(), 8, 3); cvZero(overlayImg);
	overlayMask = cvCreateImage( cap_p->getFrameSize(), 8, 3); cvSet(overlayMask, cvScalar(255, 255, 255));

	guiBackground = cvLoadImage("guiBackground.jpeg");
	namedWindow("GUI", CV_GUI_EXPANDED);
	displayImage(guiFrameCapturer->getFrame());
}


ProjectGUI::~ProjectGUI( void ){
	stop();
}


void ProjectGUI::start( void ){
	ShowImg_Thread = thread(&ProjectGUI::showImage, this);
}


void ProjectGUI::stop( void ){
	ShowImg_Thread.interrupt();
	ShowImg_Thread.join();
}

void ProjectGUI::updateOverlay( ImageProcessor * processor ){
	overlayImg_Mutex.lock();
	cvZero(overlayImg);
	cvSet(overlayMask, cvScalar(255, 255, 255));
	processor->overlayImage( overlayImg, overlayMask );
	overlayImg_Mutex.unlock();
	return;
}

void ProjectGUI::showImage( void ){
	while(!ShowImg_Thread.interruption_requested()){
		cvCopy(guiFrameCapturer->getFrame(), displayImg);
		cvResetImageROI(displayImg);
		overlayImg_Mutex.lock();
		cvAnd(displayImg, overlayMask, displayImg);
		cvAdd(displayImg, overlayImg, displayImg);
		overlayImg_Mutex.unlock();
		displayImage( displayImg );
		imshow("GUI", guiBackground);
		waitKey(1);
	}
	return;
}

void ProjectGUI::displayImage(IplImage * img_p){
	cvSetImageROI(guiBackground, cvRect( FRAME_POSITION, FRAME_SIZE ));
	cvResize( img_p, guiBackground);
	cvResetImageROI(guiBackground);
}
