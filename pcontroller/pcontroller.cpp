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
#include "../mapper.h"

using namespace std;

PController::PController(PlayerCc::PlayerClient* robot, 
			PlayerCc::Position2dProxy* pp) {

	this->robot = robot;
	this->pp = pp;
}

PController::PController(PlayerCc::PlayerClient* robot, 
	PlayerCc::Position2dProxy* pp, Mapper* parent){

	this->robot = robot;
	this->pp = pp;
	this->parent = parent;
}

double PController::DoUpdate() {
	timeval t1, t2;
	double elapsedTime = 0;
	gettimeofday(&t1, NULL);
	parent->UpdateGrid();
	robot->Read();
	gettimeofday(&t2, NULL);
	
	// compute and print the elapsed time in millisec
	elapsedTime = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
	elapsedTime += (t2.tv_usec - t1.tv_usec) / 1000.0;
	return elapsedTime /1000;
}

void PController::SimTurn(double angle) {
	using namespace PlayerCc;
	const double gain = 0.5;
	double turnrate=0, yaw=0, error=0;

	robot->Read();
	yaw = rtod(pp->GetYaw());

	double target = yaw + angle;
	double diff = 0;
	int signum = (angle > 0) - (angle < 0);
	
	do {
		DoUpdate();

		yaw = rtod(pp->GetYaw());

		error = fmod(abs(target-yaw),360);

		turnrate = (error * gain);
		turnrate *= signum;

		pp->SetSpeed(0, dtor(turnrate));
		
		// cout << "Yaw: " << yaw << endl;
		// cout << "Turnrate: " << turnrate << endl;
		// cout << "Angle Error: " << error <<endl;

	} while(abs(error) > 1);

	cout << "Done Turning!" << endl;
}

void PController::Turn(double angle) {
	SimTurn(angle);
}

void PController::PioneerTurn(double angle) {
	using namespace PlayerCc;
	const double gain = 0.6;
	double turnrate=0, yaw=0, error=0;
	double integral = 0;
	double delta = 0;
	double stop=0, start=0;
	bool minus = false;
	
	if (angle < 0) {
	 minus = true;
	 angle = 360 + angle;
	// angle -= 4;
	} else {
	  //angle += 4;
	}
	
	do {
		delta = DoUpdate();

		yaw = rtod(pp->GetYaw());
		
		if (minus) {
			yaw = (yaw == 0 ? 360 : yaw);
		}
		
		error = (minus ? yaw - angle : angle - yaw);
		
		integral += error * delta;
		turnrate = (error * gain) + (integral * 0.1);
		
		pp->SetSpeed(0, (minus ? dtor(-turnrate) : dtor(turnrate)));
		
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

	double delta = 0, error =0, integral = 0;
	angle = rtod(pp->GetYaw());
	
	do {
		delta = DoUpdate();
		
		dx = x - pp->GetXPos();
		dy = y - pp->GetYPos();
		
		error = sqrt(pow(dx,2) + pow(dy,2));
		integral += error * delta;
		speed = error * gain + integral * 0.007;

		//correct angle while moving
		angle = atan2(dy, dx) * 180 / M_PI;
		turnrate = 0;//dtor((angle - pp->GetYaw()) * 0.5);

		
		// cout << "Speed: " << speed << endl;
		// cout << "Error: " << error <<endl;
		// cout << "Integral: " << integral << endl;
		// cout << "dt: " << delta << endl;

		pp->SetSpeed(speed, turnrate);
	} while (abs(dx) > 0.1 || abs(dy) > 0.1);

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

	dx = abs(pp->GetXPos()) - abs(x);
	dy = abs(pp->GetYPos()) - abs(y);

	//calculate angle to new point
	angle = rtod(atan2(dy, dx) - pp->GetYaw());

	//correct direction
	angle = (abs(angle) > 180) ? abs(angle) - 360 : angle;

	cout << "Oldx: " << pp->GetXPos() << " Oldy: " << pp->GetYPos()  << endl;
	cout << "x: " << x << "y: " << y << endl;
	cout << "dx: " << dx << "dy: " << dy << endl;
	cout << angle << endl;
	Turn(angle);
	Move(x, y);
}

PController::~PController() {
}
