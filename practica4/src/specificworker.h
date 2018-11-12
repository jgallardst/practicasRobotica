/*
 *    Copyright (C)2018 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/



#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <chrono>
#include <genericworker.h>
#include <random>
#include <innermodel/innermodel.h>
#include <stdlib.h> 
#include <Target.h>

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);
	void setPick(const Pick &myPick);
	void goToTarget();
	bool checkObstacle(int distance);
	void bug();
	float controlIzquierda();
	void girar();
	int checkDist();
	bool targetAtSight();
	bool aligned();
	RoboCompLaser::TLaserData trimLaser(int cut);


   

public slots:
	void compute();

private:
	RoboCompGenericBase::TBaseState bState;
  	enum class botState {IDLE, GOTO, GIRO, BUG};
	botState bs = botState::IDLE;
	RoboCompLaser::TLaserData ldata;
	target_t target;
	std::shared_ptr<InnerModel> innerModel;
	int speed = 250;
	bool chooseSide = false;
	std::thread finish;
	bool triggerSet = false;
	Target t;
	float lineDiff;
	bool performed;
	QLine2D mline;
};

#endif
