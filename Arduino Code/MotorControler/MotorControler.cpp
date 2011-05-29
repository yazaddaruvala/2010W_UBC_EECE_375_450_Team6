#include "MotorControler.h"

/*
*	Motor
*
*
*
*/
Motor::Motor( const int Power_p,  const int Select_p, const int Speed_p ){
	pinMode( Power_p, OUTPUT );
	pinMode( Select_p, OUTPUT );

	this->Speed = Speed_p;
	this->Power = Power_p;
	this->Select = Select_p;
	this->stopRun();
}

Motor::~Motor( void ){
}

void Motor::run( bool dir_p ){
	dir = dir_p;
	if (dir_p == true) {
		digitalWrite( this->Select, LOW );
		analogWrite( this->Power, 0 );
		digitalWrite( this->Select, HIGH );
		analogWrite( this->Power, this->Speed );
	}
	else {
		digitalWrite( this->Select, HIGH );
		analogWrite( this->Power, 0 );
		digitalWrite( this->Select, LOW );
		analogWrite( this->Power, this->Speed );
	}
}

void Motor::stopRun(){
		digitalWrite( this->Select, HIGH );
		analogWrite( this->Power, 0 );
		digitalWrite( this->Select, LOW );
		analogWrite( this->Power, 0 );
}

void Motor::resumeRun( void ){
		run( this->dir );
}

void Motor::pauseRun( void ){
		stopRun();
}

int Motor::getSpeed( void ){
	return this->Speed;
}

/*
*	MotorControler
*
*
*
*/
MotorControler::MotorControler( Motor * Right_p, Motor * Left_p ){
	Right = Right_p;
	Left = Left_p;

	rightEncoder = 0;
	leftEncoder = 0;
}

MotorControler::~MotorControler( void ){
}

void MotorControler::turnRight( const int degs ){
	rightEncoder = (leftEncoder = degs*.333333)*0.91;
	movementFlag = 3;
	Right->run( false );
	Left->run( true );
}

void MotorControler::turnLeft( const int degs ){
	rightEncoder = (leftEncoder = degs*.333333)*0.91;
	movementFlag = 4;
	Right->run( true );
	Left->run( false );
}

void MotorControler::goForward( const int dist ){
	rightEncoder = (leftEncoder = dist*1.9)*0.91;
	movementFlag = 1;
	Right->run( true );
	Left->run( true );
}

void MotorControler::goBackward( const int dist ){
	rightEncoder = (leftEncoder = dist*1.9)*0.91;
	movementFlag = 2;
	Right->run( false );
	Left->run( false );
}

void MotorControler::stopRun( void ){
	rightEncoder = (leftEncoder = 0)*0.91;
	movementFlag = 0;
	Right->stopRun();
	Left->stopRun();
}

void MotorControler::updateRightEncoder( void ){
	rightEncoder--;
}

void MotorControler::updateLeftEncoder( void ){
	leftEncoder--;
}

void MotorControler::pause( void ){
	Left->pauseRun();
	Right->pauseRun();
}

void MotorControler::resume( void ){
	Right->resumeRun();
	Left->resumeRun();
}

void MotorControler::slightRight( void ){
	stopRun();
	int tempEncoder = (rightEncoder + leftEncoder) / 2;
	turnRight( 15 );
	delay(200);
	goForward( 20 );
	delay(200);
	turnLeft( 15 );
	goForward( tempEncoder );

}

void MotorControler::slightLeft( void ){
	stopRun();
	int tempEncoder = (rightEncoder + leftEncoder) / 2;
	turnLeft( 15 );
	delay(200);
	goForward( 20 );
	delay(200);
	turnRight( 15 );
	goForward( tempEncoder );

}

void MotorControler::allignWheels( void ){
	/*
	if ( rightEncoder == 0) {
		Right->stopRun();
	}
	else if ( leftEncoder == 0) {
		Left->stopRun();
	}
	*/
	if ( (rightEncoder + leftEncoder)/2 <= 0 ) {
		stopRun();
	}
//	if ( motorBalance >= 2 ) {
//		Serial.println( motorBalance );
//		Left->pauseRun();
//	}
//	else if ( motorBalance <= -2 ) {
//		Serial.println( motorBalance );
//		Right->pauseRun();
//	}
//	else {
//		Right->resumeRun();
//		Left->resumeRun();
//	}
}

int MotorControler::checkState( void ){
	return movementFlag;
}