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
//       THE FOLLOWING IS JUST AN EXAMPLE
//
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		innermodel_path = par.value;
//		innermodel = new InnerModel(innermodel_path);
//	}
//	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);
	return true;
}

static int calculaVelocidad(int speed, float dist){
	float brake = (dist/470) - (23/(float)25);
	return speed *= brake;
}

static int calculaAngulo(float angle){
	if(angle < 0) return 1;
	else return -1;
}


static RoboCompLaser::TLaserData orderedTrimmedData(RoboCompLaser::TLaserData ldata){
	RoboCompLaser::TLaserData ldataTrimmed;
        for(int i = 10; i <= 90; i++){
 		ldataTrimmed.push_back(ldata.at(i));
	}
	return ldataTrimmed;
}

void SpecificWorker::compute( )
{
    const float threshold = 500; //millimeters
    float rot = 1.25;  //rads per second
    int izq = 1;
    try
    {
	if(!triggerSet){
		finish = std::thread(&SpecificWorker::stop, this);
		triggerSet = true;
	}
		
	RoboCompLaser::TLaserData ldataTrimmed = orderedTrimmedData(laser_proxy->getLaserData());
        std::sort( ldataTrimmed.begin(), ldataTrimmed.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	if( ldataTrimmed.front().dist < threshold)
	{
		izq = calculaAngulo(ldataTrimmed.front().angle);
 		differentialrobot_proxy->setSpeedBase(calculaVelocidad(ldataTrimmed.front().dist, speed), izq * rot);
		usleep(rand()%(400000) +  600000);  // random wait between 1.5s and 0.1sec
	}
	else
	{
		if(speed < 1000) speed = speed + 50;
		differentialrobot_proxy->setSpeedBase(speed, 0);
		usleep(2000000 / speed);
  	}
    }
    catch(const Ice::Exception &ex)
    {
        std::cout << ex << std::endl;
    }
}


