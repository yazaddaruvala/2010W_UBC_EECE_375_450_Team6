/*
* Written by Yazad Daruvala
*/
#include "BallProcessor.h"

BallProcessor::BallProcessor( void ): Processor()
{
	hueLower = 65, hueUpper = 94;
	satLower = 62, satUpper = 225;
	valLower = 130, valUpper = 215;
	minRadius = 5;
	maxRadius = 17;
	color = CV_RGB(0,200,0);
}


BallProcessor::~BallProcessor(void)
{
}

void BallProcessor::process( vector<IplImage *> *input ){
	IplImage * threshed = arrayToThreshed(input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );

	data_Mutex.lock();
	data->clear();
	data = contourFinder->process(threshed, minRadius, cvGetSize(threshed).width/4 );
	data_Mutex.unlock();

	cvReleaseImage(&threshed);
	return;
}

void BallProcessor::configure( vector<IplImage *> *input ){
	
	IplImage * threshed = arrayToThreshed( input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );
	
	bounds_Mutex.lock();
	cvCreateTrackbar("LowerHue", "Configure Balls", &hueLower, 255, NULL);
	cvCreateTrackbar("UpperHue", "Configure Balls", &hueUpper, 255, NULL);
	cvCreateTrackbar("LowerSat", "Configure Balls", &satLower, 255, NULL);
	cvCreateTrackbar("UpperSat", "Configure Balls", &satUpper, 255, NULL);
	cvCreateTrackbar("LowerVal", "Configure Balls", &valLower, 255, NULL);
	cvCreateTrackbar("UpperVal", "Configure Balls", &valUpper, 255, NULL);

	cvCreateTrackbar("minRadius", "Configure Balls", &minRadius, 20, NULL);
	bounds_Mutex.unlock();

	cv::imshow("Thresholded Balls", threshed);
	cvReleaseImage(&threshed);
}
