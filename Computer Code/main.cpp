/*
* Written by Yazad Daruvala
*/
#include <cv.h>
#include <highgui.h>
using namespace cv;

#include <boost/thread.hpp>

#include "FrameCapturer.h"
#include "RobotMover.h"
#include "ProjectGUI.h"
#include "ImageProcessor.h"
#include "PathFinder.h"


#define SERIAL_PORT "COM5"


#define BALL_LOWER_BOUND cvScalar(24, 126, 91)		//Ball Hopefully
#define BALL_UPPER_BOUND cvScalar(77, 157, 150)		//Ball Hopefully

int main(){

	FrameCapturer * capture = new FrameCapturer( "http://137.82.120.10:8008.mjpeg" );
	RobotMover * robo = new RobotMover( SERIAL_PORT );
	ProjectGUI * guiGUI = new ProjectGUI( capture );
	guiGUI->start();
	ImageProcessor * processor = new ImageProcessor( capture );
	PathFinder * pather = new PathFinder( robo );

	char key = '\0';
	cvWaitKey(0);  
	while ((key = cvWaitKey(1000/1000)) != 27){
		processor->process();
		guiGUI->updateOverlay( processor );
		pather->updateData( processor );

		switch(key) {
		case 'c':
			//configure
			processor->configure();
			break;
		default:
			break;
		}
	}
	return 0;
}
