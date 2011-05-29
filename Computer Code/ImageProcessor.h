/*
* Written by Yazad Daruvala
*/
#ifndef EECE375_IMAGEPROCESSOR_H
#define EECE375_IMAGEPROCESSOR_H
#include <highgui.h>
#include <vector>
#include <boost/thread.hpp>

using namespace std;

#include "FrameCapturer.h"
#include "BallProcessor.h"
#include "ObstacleProcessor.h"
#include "RobotProcessor.h"


class ImageProcessor
{
public:
	ImageProcessor( FrameCapturer * cap_p );
	~ImageProcessor( void );

	void process( void );
	void configure( void );
	vector<FieldNode> *getData( void );
	void overlayImage( IplImage * overlayImg,  IplImage * overlayImgBinary );
	void updateData( vector<FieldNode> * inputBalls, vector<FieldNode> * inputObstacles, vector<FieldNode> *ourRobot, vector<FieldNode> *enemyRobot );
	bool getHomeLocation( void );

	CvSize getSize( void );
private:
	FrameCapturer * capturer;
	IplImage * filteredRGB;
	IplImage *imgHSV0;
	IplImage *imgHSV1;
	IplImage *imgHSV2;

	BallProcessor *ballProcessor;
	ObstacleProcessor *obstacleProcessor;
	RobotProcessor *ourRobotProcessor;
	RobotProcessor *enemyRobotProcessor;

	int homeLocation;
	IplImage * configDisplay; //Idealy should load an image
	vector<IplImage *> *HSV_Array;
	void getHSV_Array();
	static IplImage * filterRGB(IplImage * img_p);
	static IplImage * normalizeRGB(IplImage * img_p);

	boost::thread processThread[4];
	boost::mutex data_Mutex;

};

#endif