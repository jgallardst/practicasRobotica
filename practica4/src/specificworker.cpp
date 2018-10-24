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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{

}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
	
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}

	catch(std::exception e) { qFatal("Error reading config params"); }

	timer.start(Period);
	return true;
}

RoboCompLaser::TLaserData SpecificWorker::trimLaser(RoboCompLaser::TLaserData ldata, int cut){
	RoboCompLaser::TLaserData ldataTrimmed;
        for(int i = cut; i <= (100-cut); i++){
 		ldataTrimmed.push_back(ldata.at(i));
	}
	return ldataTrimmed;
}

bool SpecificWorker::checkObstacle()
{
	int distance = 300;
	RoboCompLaser::TLaserData ldata = trimLaser(laser_proxy->getLaserData(), 30);
    std::sort( ldata.begin(), ldata.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
    return ( ldata.front().dist < distance );
  
}


void SpecificWorker::goToTarget(){
	if(checkObstacle()){
		qDebug() << "GOTO -> BUG";
		bs = botState::BUG;
		return;
	}

	auto relative =  (innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world"));

	float angle = atan2(relative.x(), relative.z());
	float mod = relative.norm2();

	if(mod < 100){
		qDebug() << "GOTO -> IDLE";
		bs = botState::IDLE;
		differentialrobot_proxy->stopBase();
		t.setInactive();
		return;
	}


	float speed = mod;

	if( fabs(angle) > 0.05) speed = 0;
	if(speed > 500) speed = 500;
	try
  	{
  		differentialrobot_proxy->setSpeedBase(speed, angle); 
 	}
  	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }

}

bool SpecificWorker::targetAtSight(){
	QPolygonF poly;
	for(auto l : trimLaser(laser_proxy->getLaserData(), 35)){
		auto lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
	   	poly << QPointF(lr.x(), lr.z());
	}
	qDebug() << poly.containsPoint(QPointF(target.x, target.z), Qt::WindingFill);
	return poly.containsPoint(QPointF(target.x, target.z), Qt::WindingFill);
}

bool SpecificWorker::aligned(){
	auto l = laser_proxy->getLaserData()[99];
	return (375 < l.dist) && (l.dist < 425);
}

void SpecificWorker::bug(){
	if(targetAtSight()){
		bs = botState::GOTO;
		qDebug() << "BUG -> GOTO";
		return;
	}
	if(aligned())
  		differentialrobot_proxy->setSpeedBase(200, 0); 
	else differentialrobot_proxy->setSpeedBase(0, 0.3); 

}

void SpecificWorker::compute( )
{		
    try
    {		
		differentialrobot_proxy->getBaseState(bState);

		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

		if(std::get<0>(t.activoAndGet())){
				target = std::get<1>(t.activoAndGet());
		}


		switch(this->bs){
			case botState::IDLE:
				if(std::get<0>(t.activoAndGet())){
					bs = botState::GOTO;
					qDebug() << "IDLE -> GOTO";
				}
				break;
			case botState::GOTO:
				goToTarget();
				break;
			case botState::BUG:
				bug();
				break;
		}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}

void SpecificWorker::setPick(const Pick &myPick)
{
  qDebug() << "PRESSED ON: X: " << myPick.x << " Z: " << myPick.z;
  t.set(myPick.x, myPick.z);
}



