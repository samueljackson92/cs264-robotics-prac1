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
	PlayerCc::SonarProxy* sp;
	//PlayerCc::RangerProxy* sp;
	Mapper* parent;

	public:
		PController(PlayerCc::PlayerClient* robot, 
			PlayerCc::Position2dProxy* pp, PlayerCc::SonarProxy* sp, Mapper* parent);
		PController(PlayerCc::PlayerClient* robot, 
			PlayerCc::Position2dProxy* pp);
		~PController();

		void SimTurn(double angle);
		void PioneerTurn(double angle);
		void Turn(double angle);
		void Move(double x, double y);
		void MoveSetDistance(double distance);
		void MoveToPosition(double x, double y);
		double DoUpdate();

};

#endif