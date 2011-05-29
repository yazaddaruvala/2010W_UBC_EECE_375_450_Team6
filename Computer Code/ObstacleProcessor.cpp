#include "ObstacleProcessor.h"

ObstacleProcessor::ObstacleProcessor( void ): Processor()
{
	hueLower = 0, hueUpper = 37;
	satLower = 93, satUpper = 201;
	valLower = 97, valUpper = 232;
	minRadius = 10;
	maxRadius = 100;
	color = CV_RGB(0,0,200);
}


ObstacleProcessor::~ObstacleProcessor(void)
{
}


void ObstacleProcessor::process( vector<IplImage *> *input ){
	IplImage * threshed = arrayToThreshed(input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );

	data_Mutex.lock();
	data->clear();
	data = contourFinder->process(threshed, minRadius, cvGetSize(threshed).width/4 );
	data_Mutex.unlock();

	cvReleaseImage(&threshed);
	return;
}

void ObstacleProcessor::configure( vector<IplImage *> *input ){
	
	IplImage * threshed = arrayToThreshed( input, cvScalar( hueLower,  satLower, valLower), cvScalar( hueUpper,  satUpper, valUpper) );
	
	bounds_Mutex.lock();
	cvCreateTrackbar("LowerHue", "Configure Obstacles", &hueLower, 255, NULL);
	cvCreateTrackbar("UpperHue", "Configure Obstacles", &hueUpper, 255, NULL);
	cvCreateTrackbar("LowerSat", "Configure Obstacles", &satLower, 255, NULL);
	cvCreateTrackbar("UpperSat", "Configure Obstacles", &satUpper, 255, NULL);
	cvCreateTrackbar("LowerVal", "Configure Obstacles", &valLower, 255, NULL);
	cvCreateTrackbar("UpperVal", "Configure Obstacles", &valUpper, 255, NULL);

	cvCreateTrackbar("minRadius", "Configure Obstacles", &minRadius, 150, NULL);
	bounds_Mutex.unlock();

	cv::imshow("Thresholded Obstacles", threshed);
	cvReleaseImage(&threshed);
}