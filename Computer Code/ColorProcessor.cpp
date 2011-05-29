#include "ColorProcessor.h"
#define RADIUS_THRESHHOLD 4

ColorProcessor::ColorProcessor( CvSize size_p, CvScalar lowerBound_p, CvScalar upperBound_p )
{
	lowerBound = lowerBound_p;
	upperBound = upperBound_p;
	dataImg = cvCreateImage( size_p, 8, 1 );
	
	storage = cvCreateMemStorage(0);

	data_Mutex.lock();
	data = new vector<FieldNode>;
	data_Mutex.unlock();
}


ColorProcessor::~ColorProcessor(void)
{
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&dataImg);
	delete [] data;
}


void ColorProcessor::process( IplImage * input ){
	cvInRangeS( input, lowerBound, upperBound, dataImg );

	CvSeq* contours;
	cvFindContours(dataImg, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	data_Mutex.lock();
	data->clear();
	while(contours != NULL){
		CvPoint2D32f center; float radius;
		cvMinEnclosingCircle(contours, &center, &radius);
		if(cvRound(radius) > RADIUS_THRESHHOLD ){
			FieldNode tempData( cvRound(center.x), cvRound(center.y), cvRound(radius));
			data->push_back(tempData);
		}
		contours = contours->h_next;
	}
	data_Mutex.unlock();

	return;
}


vector<FieldNode> * ColorProcessor::getData( void ){
	/*
	data_Mutex.lock();
	vector<FieldNode> * temp = 	(vector<FieldNode> * )malloc( sizeof( *data ) );
	memcpy( temp, data, sizeof( *data ) );
	data_Mutex.unlock();
	*/
	return data;
}

CvScalar ColorProcessor::getColor( void ) const{
	return color;
}


void ColorProcessor::configure( IplImage * img_p ){
	int hueLower = 0, hueUpper = 255;
	int satLower = 0, satUpper = 255;
	int valLower = 0, valUpper = 255;


	cvNamedWindow("Configure");
	cvCreateTrackbar("LowerHue", "Configure", &hueLower, 255, NULL);
	cvCreateTrackbar("UpperHue", "Configure", &hueUpper, 255, NULL);
	cvCreateTrackbar("LowerSat", "Configure", &satLower, 255, NULL);
	cvCreateTrackbar("UpperSat", "Configure", &satUpper, 255, NULL);
	cvCreateTrackbar("LowerVal", "Configure", &valLower, 255, NULL);
	cvCreateTrackbar("UpperVal", "Configure", &valUpper, 255, NULL);

	IplImage * img = cvCreateImage(cvGetSize(img_p), 8, 1);
	while(cvGetWindowHandle("Configure") != NULL){
		lowerBound = cvScalar( hueLower, satLower, valLower );
		upperBound = cvScalar( hueUpper, satUpper, valUpper );
		process( img_p );
		cvZero(img);
		for(vector<FieldNode>::iterator dataIterator = data->begin(); dataIterator != data->end(); ++dataIterator){
			cvCircle( img, cvPoint(dataIterator->getxPos(), dataIterator->getyPos()), dataIterator->getRadius(), cvScalar(255), dataIterator->getRadius());
		}
		cvShowImage("Configure", img);
		switch( cv::waitKey(1) ){
		case '\n':
			cvReleaseImage( &img );
			cvDestroyWindow("Configure");
			break;
		}
	}
	return;
}
