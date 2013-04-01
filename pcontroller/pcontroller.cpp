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
    return (double) (t.tv_usec / 1000);

}

void PController::Turn(double angle) {
	using namespace PlayerCc;
	const double gain = 0.5;
	double turnrate=0, yaw=0, error=0;
	double integral = 0;
	double delta = 0;
	double stop=0, start=0;
	bool minus = false;
	
	
	
	if (angle < 0) {
	 minus = true;
	 angle = 360 + angle;
	 angle -= 4;
	} else {
	  angle += 4;
	}
	
	do {
		timeval t1, t2;
		double elapsedTime;

		// start timer
		gettimeofday(&t1, NULL);
		robot->Read();
		gettimeofday(&t2, NULL);
		
		// compute and print the elapsed time in millisec
		elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
		elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
		delta = elapsedTime /1000;

		
		yaw = rtod(pp->GetYaw());
		
		//yaw = (minus ? -yaw : yaw);
		if (minus) {
		  yaw = (yaw == 0 ? 360 : yaw);
		}
		
		error = (minus ? yaw - angle : angle - yaw);
		
		integral += error * delta;
		turnrate = (error * gain) + (integral * 0.05);
		
		pp->SetSpeed(0, (minus ? dtor(-turnrate) : dtor(turnrate)));
		
		cout << "Yaw: " << yaw << endl;
		cout << "Turnrate: " << turnrate << endl;
		cout << "Angle Error: " << error <<endl;
		cout << "Integral: " << integral << endl;
		cout << "dt: " << delta << endl;

		
	} while(abs(error) > 0);

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
