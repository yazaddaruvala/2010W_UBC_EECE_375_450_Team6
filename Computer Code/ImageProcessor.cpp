/*
* Written by Yazad Daruvala
*/
#include "ImageProcessor.h"
#define BALL_BOUNDS cvScalar(24, 126, 91), cvScalar(55,240,240)
#include <math.h>

#define videoFrameROI cvRect(155, 0, 425, 480)

ImageProcessor::ImageProcessor( FrameCapturer * cap_p )
{
	capturer = cap_p;
	HSV_Array = new vector<IplImage *>;
	ballProcessor = new BallProcessor();
	obstacleProcessor = new ObstacleProcessor();
	ourRobotProcessor = new RobotProcessor( 0 );
	enemyRobotProcessor = new RobotProcessor( 1 );
	homeLocation = 0;
	configDisplay = cvLoadImage("guiBackground.jpeg");
	filteredRGB = cvCreateImage(capturer->getFrameSize(), 8, 3);
	cvSetImageROI(filteredRGB, videoFrameROI);
	imgHSV0 = cvCreateImage( cvGetSize(filteredRGB), 8, 3);
	imgHSV1 = cvCreateImage( cvGetSize(filteredRGB), 8, 3);
	imgHSV2 = cvCreateImage( cvGetSize(filteredRGB), 8, 3);
	cvResetImageROI(filteredRGB);

}


ImageProcessor::~ImageProcessor(void)
{
	/*delete [] balls;
	delete [] objects;
	delete [] myRF;
	delete [] myRB;
	delete [] enemyRF;
	delete [] enemyRB;
	delete [] data;
	*/
}

void ImageProcessor::process() {
	
	getHSV_Array();
	//ballProcessor->process(HSV_Array);
	//obstacleProcessor->process(HSV_Array);
	//ourRobotProcessor->process(HSV_Array);
	//enemyRobotProcessor->process(HSV_Array);

	processThread[0] = thread(&BallProcessor::process, ballProcessor, HSV_Array);
	processThread[1] = thread(&ObstacleProcessor::process, obstacleProcessor, HSV_Array);
	processThread[2] = thread(&RobotProcessor::process, ourRobotProcessor, HSV_Array);
	processThread[3] = thread(&RobotProcessor::process, enemyRobotProcessor, HSV_Array);

	for (int i = 0; i < 4; i++){
		processThread[i].join();
	}
	cvResetImageROI( filteredRGB );

}


void ImageProcessor::overlayImage( IplImage * overlayImg,  IplImage * overlayMask ){
	cvSetImageROI( overlayImg, videoFrameROI);
	cvSetImageROI( overlayMask, videoFrameROI);
	
	cvRectangle( overlayImg, cvPoint(0, 0), cvPoint(getSize().width, getSize().height), CV_RGB(255,255,255), 5 );
	cvRectangle( overlayMask, cvPoint(0, 0), cvPoint(getSize().width, getSize().height), CV_RGB(0,0,0), 5 );
	if (getHomeLocation()){
		cvRectangle( overlayImg, cvPoint(160, 30), cvPoint(250, 60), CV_RGB( 226, 150, 65), 2 );
		cvRectangle( overlayMask, cvPoint(160, 30), cvPoint(250, 60), CV_RGB( 0, 0, 0), 2 );
	} else {
		cvRectangle( overlayImg, cvPoint(160, getSize().height - 30), cvPoint(250, getSize().height - 60), CV_RGB(226,150,65), 2 );
		cvRectangle( overlayMask, cvPoint(160, getSize().height - 30), cvPoint(250, getSize().height - 60), CV_RGB(0,0,0), 2 );
	}

	ballProcessor->overlayImage( overlayImg, overlayMask );
	obstacleProcessor->overlayImage( overlayImg, overlayMask );
	ourRobotProcessor->overlayImage( overlayImg, overlayMask );
	enemyRobotProcessor->overlayImage( overlayImg, overlayMask );

	cvResetImageROI(overlayMask);
	cvResetImageROI(overlayImg);
}

void ImageProcessor::updateData( vector<FieldNode> * inputBalls, vector<FieldNode> * inputObstacles, vector<FieldNode> *ourRobot, vector<FieldNode> *enemyRobot ){
	ballProcessor->updateData( inputBalls );
	obstacleProcessor->updateData( inputObstacles );
	ourRobotProcessor->updateData( ourRobot );
	enemyRobotProcessor->updateData( enemyRobot );
}

bool ImageProcessor::getHomeLocation( void ) {
	switch(homeLocation){
	case 0:
		return true;
		break;
	case 1:
		return false;
		break;
	}
}

void ImageProcessor::configure( void ) {
	cv::imshow("Configuration", configDisplay);
	cvCreateTrackbar("Home Location", "Configuration", &homeLocation, 1, NULL);
	while(cvGetWindowHandle("Configuration") != NULL){
		switch( cv::waitKey(1) ) {
		case 'b':
			cvNamedWindow("Configure Balls");
			cvResizeWindow("Configure Balls", 400, 500);
			cvNamedWindow("Thresholded Balls");
			while(cvGetWindowHandle("Configure Balls") != NULL){
				getHSV_Array();
				ballProcessor->configure( HSV_Array );
				switch( cv::waitKey(1) ){
				case '\n':
					cvDestroyWindow("Configure Balls");
					break;
				default:
					break;
				}
			}
			cvDestroyWindow("Thresholded Balls");
			break;
		case 'o':
			cvNamedWindow("Configure Obstacles");
			cvResizeWindow("Configure Obstacles", 400, 500);
			cvNamedWindow("Thresholded Obstacles");
			while(cvGetWindowHandle("Configure Obstacles") != NULL){
				getHSV_Array();
				obstacleProcessor->configure( HSV_Array );
				switch( cv::waitKey(1) ){
				case '\n':
					cvDestroyWindow("Configure Obstacles");
					break;
				default:
					break;
				}
			}
			cvDestroyWindow("Thresholded Obstacles");
			break;
		case 'r':
			cvNamedWindow("Configure OurFront");
			cvResizeWindow("Configure OurFront", 400, 500);
			cvNamedWindow("Configure OurBack");
			cvResizeWindow("Configure OurBack", 400, 500);
	
			cvNamedWindow("Thresholded OurFront");
			cvNamedWindow("Thresholded OurBack");
			while((cvGetWindowHandle("Configure OurFront") != NULL)||(cvGetWindowHandle("Configure OurBack") != NULL)){
				getHSV_Array();
				ourRobotProcessor->configure( HSV_Array );
				switch( cv::waitKey(1) ){
				case '\n':
					cvDestroyWindow("Configure OurFront");
					cvDestroyWindow("Configure OurBack");
					break;
				default:
					break;
				}
			}
			cvDestroyWindow("Thresholded OurFront");
			cvDestroyWindow("Thresholded OurBack");
			break;
		case 'e':
			cvNamedWindow("Configure EnemyFront");
			cvResizeWindow("Configure EnemyFront", 400, 500);
			cvNamedWindow("Configure EnemyBack");
			cvResizeWindow("Configure EnemyBack", 400, 500);
	
			cvNamedWindow("Thresholded EnemyFront");
			cvNamedWindow("Thresholded EnemyBack");
			while((cvGetWindowHandle("Configure EnemyFront") != NULL)||(cvGetWindowHandle("Configure EnemyBack") != NULL)){
				getHSV_Array();
				enemyRobotProcessor->configure( HSV_Array );
				switch( cv::waitKey(1) ){
				case '\n':
					cvDestroyWindow("Configure EnemyFront");
					cvDestroyWindow("Configure EnemyBack");
					break;
				default:
					break;
				}
			}
			cvDestroyWindow("Thresholded EnemyFront");
			cvDestroyWindow("Thresholded EnemyBack");
			break;
		default:
			break;
		}
	}
}

void ImageProcessor::getHSV_Array(){
	HSV_Array->clear();

	cvCopy( capturer->getFrame(), filteredRGB );
	cvSetImageROI(filteredRGB, videoFrameROI);
	filteredRGB = filterRGB(filteredRGB);
	cvCvtColor(filteredRGB, imgHSV0, CV_RGB2HSV);
	cvResetImageROI(filteredRGB);
	HSV_Array->push_back(imgHSV0);
	/*
	cvCopy( capturer->getFrame(), filteredRGB );
	cvSetImageROI(filteredRGB, videoFrameROI);
	filteredRGB = filterRGB(filteredRGB);
	cvCvtColor(filteredRGB, imgHSV1, CV_RGB2HSV);
	cvResetImageROI(filteredRGB);
	HSV_Array->push_back(imgHSV1);

	cvCopy( capturer->getFrame(), filteredRGB );
	cvSetImageROI(filteredRGB, videoFrameROI);
	filteredRGB = filterRGB(filteredRGB);
	cvCvtColor(filteredRGB, imgHSV2, CV_RGB2HSV);
	cvResetImageROI(filteredRGB);
	HSV_Array->push_back(imgHSV2);
	*/
}

CvSize ImageProcessor::getSize( void ){
	return cvSize(videoFrameROI.width, videoFrameROI.height);
}

IplImage * ImageProcessor::filterRGB(IplImage * img_p){
	cvSmooth( img_p, img_p);
	//cvDilate(img_p, img_p, NULL, 2);
	//cvErode(img_p, img_p, NULL, 2);
	return img_p;
	
}


IplImage * ImageProcessor::normalizeRGB(IplImage * img_p){
	IplImage * Bimg = cvCreateImage(cvGetSize(img_p), 8, 1);
	IplImage * Rimg = cvCreateImage(cvGetSize(img_p), 8, 1);
	IplImage * Gimg = cvCreateImage(cvGetSize(img_p), 8, 1);
	cvSplit(img_p, Bimg, Gimg, Rimg, NULL);
	
	for(int x=0;x<img_p->width;x++) {
        	for(int y=0;y<img_p->height;y++) {
			double redValue = cvGetReal2D(Rimg, y, x);
			double greenValue = cvGetReal2D(Gimg, y, x);
			double blueValue = cvGetReal2D(Bimg, y, x);
			double sum = sqrt((redValue*redValue) + (greenValue*greenValue) + (blueValue*blueValue));
			cvSetReal2D(Rimg, y, x, (redValue/sum)*255);
			cvSetReal2D(Gimg, y, x, (greenValue/sum)*255);
			cvSetReal2D(Bimg, y, x, (blueValue/sum)*255);
		}
	}
	cvMerge(Bimg, Gimg, Rimg, NULL, img_p);
	return img_p;
}
