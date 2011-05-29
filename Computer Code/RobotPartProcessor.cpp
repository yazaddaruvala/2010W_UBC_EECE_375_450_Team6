/*
* Written by Yazad Daruvala
*/
#include "RobotPartProcessor.h"

RobotPartProcessor::RobotPartProcessor( char* name_p, CvScalar color_p, const int hl, const int sl, const int vl, const int hh, const int sh, const int vh ): Processor()
{
	name = name_p;
	hueLower = hl, hueUpper = hh;
	satLower = sl, satUpper = sh;
	valLower = vl, valUpper = vh;
	minRadius = 5;
	maxRadius = 17;
	color = color_p;
}


RobotPartProcessor::~RobotPartProcessor(void)
{
}


void RobotPartProcessor::process( vector<IplImage *> *input ){
	IplImage * threshed = arrayToThreshed(input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );

	data_Mutex.lock();
	data->clear();
	data = contourFinder->process(threshed, minRadius, cvGetSize(threshed).width/4 );
	data_Mutex.unlock();

	cvReleaseImage(&threshed);
	return;
}

void RobotPartProcessor::configure( vector<IplImage *> *input ){
	
	IplImage * threshed = arrayToThreshed( input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );
	
	bounds_Mutex.lock();
	char configWindow[50];
	sprintf( configWindow, "Configure %s\0", name );
	cvCreateTrackbar("LowerHue", configWindow, &hueLower, 255, NULL);
	cvCreateTrackbar("UpperHue", configWindow, &hueUpper, 255, NULL);
	cvCreateTrackbar("LowerSat", configWindow, &satLower, 255, NULL);
	cvCreateTrackbar("UpperSat", configWindow, &satUpper, 255, NULL);
	cvCreateTrackbar("LowerVal", configWindow, &valLower, 255, NULL);
	cvCreateTrackbar("UpperVal", configWindow, &valUpper, 255, NULL);

	cvCreateTrackbar("minRadius", configWindow, &minRadius, 20, NULL);
	bounds_Mutex.unlock();
	char threshWindow[50];
	sprintf( threshWindow, "Thresholded %s\0", name );
	cv::imshow( threshWindow, threshed);
	cvReleaseImage(&threshed);
}
