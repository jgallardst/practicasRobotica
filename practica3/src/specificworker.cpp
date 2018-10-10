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
		innermodel_path = par.value;
		innermodel = new InnerModel(innermodel_path);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }




	timer.start(Period);
	return true;
}

static RoboCompLaser::TLaserData orderedTrimmedData(RoboCompLaser::TLaserData ldata){
	RoboCompLaser::TLaserData ldataTrimmed;
        for(int i = 12; i <= 88; i++){
 		ldataTrimmed.push_back(ldata.at(i));
	}
	return ldataTrimmed;
}

void SpecificWorker::compute( )
{
    try
    {		
	RoboCompLaser::TLaserData ldataTrimmed = orderedTrimmedData(laser_proxy->getLaserData());
        std::sort( ldataTrimmed.begin(), ldataTrimmed.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	if( ldataTrimmed.front().dist < threshold)
	{
		usleep(rand()%(350000) +  300000);  // random wait between 1.5s and 0.1sec
	}
	else
	{
		chooseSide = false;
		usleep(2000000 / speed);
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



