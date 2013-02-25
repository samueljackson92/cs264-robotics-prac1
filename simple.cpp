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
#include <signal.h>

#include "pcontroller.h"
#include "occupancygrid.h"

using namespace std;
using namespace PlayerCc;

OccupancyGrid grid(-7,-7);

//Interrupt handler.
void signal_callback_handler(int signum);

int main(int argc, char *argv[])
{
	signal(SIGINT, signal_callback_handler);

	PlayerClient robot("localhost");
	RangerProxy sp(&robot,0);
	Position2dProxy pp(&robot,0);

	//create a pcontroller for the robot
	PController pc(&robot, &pp);

	double turnrate, speed, dx, dy;
	double x, y, angle;

	robot.Read();

	x = pp.GetXPos();
	y = pp.GetYPos();

	//create a blank occupancy grid

	pp.SetMotorEnable(true);

	//pc.Turn(0);
	//pc.MoveSetDistance(5);
	for (;;) {
		robot.Read();
	 	std::cout << sp << std::endl;

		//do simple collision avoidance
		if((sp[0] + sp[1]) < (sp[6] + sp[7]))
			turnrate = dtor(-20); // turn 20 degrees per second
		else
			turnrate = dtor(20);

		if(sp[3] < 0.500)
			speed = 0;
		else
			speed = 0.100;

		//map using sensors
		x = pp.GetXPos();
		y = pp.GetYPos();
		angle = pp.GetYaw();

		//forward sensors
		grid.UpdateBotPosition(x,y);
		// grid.SensorUpdate(sp[3], angle + dtor(10));
		// grid.SensorUpdate(sp[4], angle + dtor(350));

		//side sensors
		grid.SensorUpdate(sp[0], angle + dtor(90));
		grid.SensorUpdate(sp[7], angle + dtor(270));

		//rear sensors
		// grid.SensorUpdate(sp[12], angle + dtor(170));
		// grid.SensorUpdate(sp[11], angle + dtor(190));

		grid.PrintGrid();

		//pc.MoveSetDistance(1);
		//command the motors
		pp.SetSpeed(speed, turnrate);

	}
}

void signal_callback_handler(int signum)
{
   printf("Caught signal %d\n",signum);
   // Cleanup and close up stuff here
   grid.WriteGrid("output.csv");
   // Terminate program
   exit(signum);
}

