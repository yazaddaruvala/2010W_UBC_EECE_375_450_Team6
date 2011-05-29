/*
* Written by Yazad Daruvala
*/
#include "RobotProcessor.h"
#include <iostream>
#define MY_ROBOT 0

RobotProcessor::RobotProcessor( int type_p )
{
	type = type_p;
	if (type == 0) {
		frontProcessor = new RobotPartProcessor( "OurFront", CV_RGB(200, 0, 0), 113, 40, 92, 145, 206, 235);
		backProcessor = new RobotPartProcessor( "OurBack", CV_RGB(175, 56, 225), 153, 34, 109, 182, 119, 242);
		//backProcessor = new RobotPartProcessor( "OurBack", CV_RGB(200, 200, 4), 89, 82, 205, 101, 255, 255);
	}
	else if (type == 1) {
		frontProcessor = new RobotPartProcessor( "EnemyFront", CV_RGB(0, 0, 0), 0, 0, 0, 0, 0, 0);
		backProcessor = new RobotPartProcessor( "EnemyBack", CV_RGB(255, 255, 255), 0, 0, 0, 0, 0, 0);
	}
	Xback = 0;
	Yback = 0;
	Xfront = 0;
	Yfront = 0;
}


RobotProcessor::~RobotProcessor(void)
{
	delete [] frontProcessor;
	delete [] backProcessor;

}

void RobotProcessor::process( vector<IplImage *> *input){
	
	frontProcessor->process(input);
	backProcessor->process(input);
	data_Mutex.lock();
	Xback = 0;
	Xfront = 0;
	Yback = 0;
	Yfront = 0;
	long shortestDistance = 10000000;
	if ( type == 0 ){
		for(vector<FieldNode>::iterator backIterator = backProcessor->getData()->begin(); backIterator != backProcessor->getData()->end(); ++backIterator){
			for(vector<FieldNode>::iterator backIterator2 = backProcessor->getData()->begin(); backIterator2 != backProcessor->getData()->end(); ++backIterator2){
				long distance = ( backIterator2->getxPos() - backIterator->getxPos() )*( backIterator2->getxPos() - backIterator->getxPos() )  + (backIterator2->getyPos() - backIterator->getyPos() )*(backIterator2->getyPos() - backIterator->getyPos() );
				if(  (distance < 1600) && (distance < shortestDistance) &&  (distance > 400) ){
					Xback = ( backIterator->getxPos() + backIterator2->getxPos() ) /2;
					Yback = ( backIterator->getyPos() + backIterator2->getyPos() ) /2;
					shortestDistance = distance;
				}
			}
		}

		shortestDistance = 100000000; 
		for(vector<FieldNode>::iterator frontIterator = frontProcessor->getData()->begin(); frontIterator != frontProcessor->getData()->end(); ++frontIterator){
			long distance = ( Xback - frontIterator->getxPos() )*( Xback - frontIterator->getxPos() )  + (Yback - frontIterator->getyPos() )*(Yback - frontIterator->getyPos() );
			if( (distance < 1900) && (distance < shortestDistance) && (distance > 400)){
				Xfront = frontIterator->getxPos();
				Yfront = frontIterator->getyPos();
				shortestDistance = distance;
			}
		}
	}
	else if( type == 1 ){
		shortestDistance = 100000000; 
		for(vector<FieldNode>::iterator frontIterator = frontProcessor->getData()->begin(); frontIterator != frontProcessor->getData()->end(); ++frontIterator){
			for(vector<FieldNode>::iterator backIterator = backProcessor->getData()->begin(); backIterator != backProcessor->getData()->end(); ++backIterator){
				long distance = ( backIterator->getxPos() - frontIterator->getxPos() )*( backIterator->getxPos() - frontIterator->getxPos() )  + (backIterator->getyPos() - frontIterator->getyPos() )*(backIterator->getyPos() - frontIterator->getyPos() );
				if( (distance < 1900) && (distance < shortestDistance) && (distance > 100)){
					Xfront = frontIterator->getxPos();
					Yfront = frontIterator->getyPos();
					Xback = backIterator->getxPos();
					Yback = backIterator->getyPos();
				}
			}
		}
	}

	data_Mutex.unlock();
}

void RobotProcessor::configure( vector<IplImage *> *input){
	frontProcessor->configure(input);
	backProcessor->configure(input);
}

void RobotProcessor::updateData(  vector<FieldNode> *ourRobot ){
	FieldNode frontNode( Xfront, Yfront, 0 );
	FieldNode backNode( Xback, Yback, 0 );

	ourRobot->push_back(frontNode);
	ourRobot->push_back(backNode);
}

void RobotProcessor::overlayImage( IplImage * overlayImg,  IplImage * overlayMask,  const int shiftX, const int shiftY){
	frontProcessor->overlayImage( overlayImg, overlayMask, shiftX, shiftY );
	backProcessor->overlayImage( overlayImg, overlayMask, shiftX, shiftY );
	
	data_Mutex.lock();
	if ( !(Xfront == 0 && Yfront == 0) ){
		cvCircle( overlayImg,  cvPoint( Xfront + shiftX, Yfront + shiftY), 15, frontProcessor->getColor(), 5);
		cvCircle( overlayMask, cvPoint( Xfront + shiftX, Yfront + shiftY), 15, CV_RGB(0, 0, 0), 5);
	}
	if ( !(Xback == 0 && Yback == 0) ){
		cvCircle( overlayImg,  cvPoint( Xback + shiftX, Yback + shiftY), 15, backProcessor->getColor(), 5);
		cvCircle( overlayMask, cvPoint( Xback + shiftX, Yback + shiftY), 15, CV_RGB(0, 0, 0), 5);
	}
	data_Mutex.unlock();
	
}
