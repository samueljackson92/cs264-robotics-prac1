/*
	PController.cpp
	Description: Implementation of a simple P-Controller system
	for pioneer robots using Player/Stage.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	24/2/13
*/

#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>
#include <sys/time.h>


#include "pcontroller.h"

using namespace std;

PController::PController(PlayerCc::PlayerClient* robot, 
	PlayerCc::Position2dProxy* pp){

	this->robot = robot;
	this->pp = pp;
}

clock_t PController::getMilliSecs()
{
    timeval t;
    gettimeofday(&t, NULL);
    return (double) t.tv_usec / 1000;

}

void PController::Turn(double oangle) {
	using namespace PlayerCc;
	const double gain = 0.53;
	double turnrate, yaw, error;
	double integral = 0;
	double delta = 0;

	double stop, start, angle;
	
	if(oangle < 0) {
	  angle = 360 + oangle;
	} else {
	  angle = oangle;
	}
	
	do {
		//delta = 0.5;

		
		start = getMilliSecs();
		robot->Read();
		stop = getMilliSecs();
		
		cout << start << endl;
		cout << stop << endl;
		
		delta = abs((stop - start)) /1000;
		
		yaw = rtod(pp->GetYaw());
		
		error = angle - yaw;
		
		integral += error * delta;
		turnrate = error * gain + integral * 0.07;

		if (oangle < 0) {
		  turnrate *= -1;
		}
		
		pp->SetSpeed(0, dtor(turnrate));
		
		cout << "Yaw: " << yaw << endl;
		cout << "Turnrate: " << turnrate << endl;
		cout << "Angle Error: " << error <<endl;
		cout << "Integral: " << integral << endl;
		cout << "dt: " << delta << endl;

		
	} while(abs(error) > 1);

	cout << "Done Turning!" << endl;
}

void PController::Move(double x, double y) {
	using namespace PlayerCc;
	double dx, dy, angle, speed, turnrate;
	const double gain = 0.5;

	dx = x - pp->GetXPos();
	dy = y - pp->GetYPos();

	double start, stop, delta, error, integral = 0;
	angle = rtod(pp->GetYaw());
	
	do {
		start = getMilliSecs();
		robot->Read();
		stop = getMilliSecs();

		
		delta = abs((stop - start)) /1000;
		
		dx = x - pp->GetXPos();
		dy = y - pp->GetYPos();
		
		error = sqrt(pow(dx,2) + pow(dy,2));
		integral += error * delta;
		speed = error * gain + integral * 0.007;

		//correct angle while moving
		angle = atan2(dy, dx) * 180 / M_PI;
		turnrate = 0;//dtor((angle - pp->GetYaw()) * 0.5);

		
		cout << "Speed: " << speed << endl;
		cout << "Error: " << error <<endl;
		cout << "Integral: " << integral << endl;
		cout << "dt: " << delta << endl;

		pp->SetSpeed(speed, turnrate);
	} while (abs(dx) > 0.3 || abs(dy) > 0.3);

	cout << "X Error: " << dx << endl;
	cout << "Y Error: " << dy << endl;
	cout << "Finished Moving!" << endl;
}

void PController::MoveSetDistance(double distance) {
	double angle = pp->GetYaw();

	double dx = cos(angle) * distance;
	double dy = sin(angle) * distance;

	double x = pp->GetXPos() + dx;
	double y = pp->GetYPos() + dy;

	cout << dx << endl;
	cout << dy << endl;

	cout << x << endl;
	cout << y << endl;

	Move(x,y);
}

void PController::MoveToPosition(double x, double y){
	using namespace PlayerCc;
	double dx, dy, angle;

	dx = x - pp->GetXPos();
	dy = y - pp->GetYPos();

	angle = atan2(dy, dx) * 180 / M_PI;
	cout << rtod(pp->GetYaw()) - angle << endl;
	Turn(angle);
	Move(x, y);
}

PController::~PController() {
}
