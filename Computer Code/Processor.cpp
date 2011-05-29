/*
* Written by Yazad Daruvala
*/
#include "Processor.h"

Processor::Processor(void)
{
	SegmentFinder *contourFinder = new SegmentFinder();
	
	data_Mutex.lock();
	data = new vector<FieldNode>;
	data_Mutex.unlock();
}


Processor::~Processor(void)
{
	delete [] contourFinder;
	delete [] data;
}

void Processor::overlayImage( IplImage * overlayImg,  IplImage * overlayMask,  const int shiftX, const int shiftY){
	data_Mutex.lock();
	if(!data->empty()){	
		for(vector<FieldNode>::iterator dataIterator = data->begin(); dataIterator != data->end(); ++dataIterator){
			cvCircle( overlayImg,  cvPoint(dataIterator->getxPos() + shiftX, dataIterator->getyPos() + shiftY), dataIterator->getRadius(), color, dataIterator->getRadius());
			cvCircle( overlayMask, cvPoint(dataIterator->getxPos() + shiftX, dataIterator->getyPos() + shiftY), dataIterator->getRadius(), CV_RGB(0, 0, 0), 2);
		}
	}
	data_Mutex.unlock();
}

void Processor::updateData( vector<FieldNode> * input){
	data_Mutex.lock();
	if(!data->empty()){	
		for(vector<FieldNode>::iterator dataIterator = data->begin(); dataIterator != data->end(); ++dataIterator){
			FieldNode temp(dataIterator->getxPos(), dataIterator->getyPos(), dataIterator->getRadius());
			input->push_back( temp );
		}
	}
	data_Mutex.unlock();
	
}

vector<FieldNode> *Processor::getData( void ){
	return data;
}

CvScalar Processor::getColor( void ) const{
	return color;
}

IplImage * Processor::arrayToThreshed( vector<IplImage *> *input, CvScalar lowerBound, CvScalar upperBound ){
	IplImage * threshed = cvCreateImage(cvGetSize(input->front()), 8, 1);
	cvZero(threshed);
	for (vector<IplImage *>::iterator inputIterator = input->begin(); inputIterator != input->end(); ++inputIterator){
		IplImage * temp = cvCreateImage(cvGetSize((*inputIterator)), 8, 1);	
		cvInRangeS( (*inputIterator), lowerBound, upperBound, temp );
		cvOr(threshed, temp, threshed);
		cvReleaseImage(&temp);
	}
	return threshed;
}
