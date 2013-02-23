/*
 * Copyright (c) 2005, Brad Kratochvil, Toby Collett, Brian Gerkey, Andrew Howard, ...
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice,
 *           this list of conditions and the following disclaimer.
 *               * Redistributions in binary form must reproduce the above copyright notice,
 *                     this list of conditions and the following disclaimer in the documentation
 *                           and/or other materials provided with the distribution.
 *                               * Neither the name of the Player Project nor the names of its contributors
 *                                     may be used to endorse or promote products derived from this software
 *                                           without specific prior written permission.
 *
 *                                           THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *                                           ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *                                           WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *                                           DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 *                                           ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *                                           (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *                                           LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *                                           ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *                                           (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *                                           SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *                                           */

#define _USE_MATH_DEFINES

#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>

using namespace std;
using namespace PlayerCc;

void turn(double angle, PlayerClient &robot, Position2dProxy &pp);
void move(double x, double y, PlayerClient &robot, Position2dProxy &pp);
void moveToPosition(double x, double y, PlayerClient &robot, Position2dProxy &pp);

int main(int argc, char *argv[])
{
	PlayerClient robot("localhost");
	RangerProxy sp(&robot,0);
	Position2dProxy pp(&robot,0);

	pp.SetMotorEnable(true);
	robot.Read();

	moveToPosition(7, -7, robot, pp);
	// for(;;)
	// {
	// 	double turnrate, speed;

	// 	//  read from the proxies
	// 	robot.Read();

	// 	//print out sonars for fun
	// 	std::cout << sp << std::endl;

	// 	//do simple collision avoidance
	// 	if((sp[0] + sp[1]) < (sp[6] + sp[7]))
	// 		turnrate = dtor(-20); // turn 20 degrees per second
	// 	else
	// 		turnrate = dtor(20);

	// 	if(sp[3] < 0.500)
	// 		speed = 0;
	// 	else
	// 		speed = 0.100;


	// 	move(-1, -7, robot, pp);
	// 	//turn(90, robot, pp); //turn 90 degrees

	// 	//command the motors
	// 	//pp.SetSpeed(speed, turnrate);

	// }
}

//turn to the given angle
void turn(double target, PlayerClient &robot, Position2dProxy &pp) {
	const double gain = 0.5;
	double turnrate, yaw, error;

	do {
		robot.Read();
		yaw = rtod(pp.GetYaw());

		error = target - yaw;
		turnrate = error * gain;

		pp.SetSpeed(0, dtor(turnrate));

	} while(abs(error) > 2);

	cout << "Angle Error: " << error << endl;
	cout << "Done Turning!" << endl;
}

void moveToPosition(double x, double y, PlayerClient &robot, Position2dProxy &pp) {
	double dx, dy, angle;

	dx = x - pp.GetXPos();
	dy = y - pp.GetYPos();

	angle = atan2(dy, dx) * 180 / M_PI;
	turn(angle, robot, pp);
	move(x, y, robot, pp);
}

//Move robot to a specific x/y position
void move(double x, double y, PlayerClient &robot, Position2dProxy &pp) {
	double dx, dy, angle, speed, turnrate;
	const double gain = 0.15;

	dx = x - pp.GetXPos();
	dy = y - pp.GetYPos();

	do {
		robot.Read();

		dx = x - pp.GetXPos();
		dy = y - pp.GetYPos();
		speed = sqrt(pow(dx,2) + pow(dy,2)) * gain;

		//correct angle while moving
		angle = atan2(dy, dx) * 180 / M_PI;
		turnrate = dtor((angle - rtod(pp.GetYaw())) * 0.5);

		pp.SetSpeed(speed, turnrate);
	} while (abs(dx) > 0.3 || abs(dy) > 0.3);

	cout << "X Error: " << dx << endl;
	cout << "Y Error: " << dy << endl;
	cout << "Finished Moving!" << endl;
}

