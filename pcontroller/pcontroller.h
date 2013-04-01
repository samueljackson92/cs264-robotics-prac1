/*
	PController.h
	Description: Implementation of a simple P-Controller system
	for pioneer robots using Player/Stage.
	Author:	Samuel Jackson (slj11@aber.ac.uk)
	Date:	24/2/13
*/

#ifndef __PCONTROLLER_H_INCLUDED__
#define __PCONTROLLER_H_INCLUDED__
	
#include <iostream>
#include <libplayerc++/playerc++.h>
#include <cmath>

class Mapper;
class PController {
	PlayerCc::PlayerClient* robot;
	PlayerCc::Position2dProxy* pp;
	Mapper* parent;

	public:
		PController(PlayerCc::PlayerClient* robot, 
			PlayerCc::Position2dProxy* pp, Mapper* parent);
		PController(PlayerCc::PlayerClient* robot, 
			PlayerCc::Position2dProxy* pp);
		~PController();

		void Turn(double angle);
		void Move(double x, double y);
		void MoveSetDistance(double distance);
		void MoveToPosition(double x, double y);
		double DoUpdate();

};

#endif