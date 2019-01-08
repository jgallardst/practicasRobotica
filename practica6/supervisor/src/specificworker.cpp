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

	targets.push_back("target00");
	targets.push_back("target01");
	targets.push_back("target02");
	targets.push_back("target03");

	timer.start(Period);
	return true;
}

void SpecificWorker::compute()
{
	int tagHolder;
	if((tagHolder = currentTag.id()) != -1){
		currentTagID = tagHolder;
		qDebug() << "Found new tag with ID: " << currentTagID;
	}
	
	switch(this->ss){
		case supervisorStatus::SEARCH:
			// Sacamos el elemento a buscar y reencolamos al final
			target = targets.front();
			targets.pop_front(); targets.push_back(target);
			tagCords = innerModel->transform("world", target.c_str());
			qDebug() << "Objetivo en " << tagCords;
			gotopoint_proxy->go("", tagCords().x(), tagCords.z(), 0);
			ss = supervisorStatus::WAIT;
			break;
		case supervisorStatus::WAIT:
			if( gotopoint_proxy->atTarget() == true){
				if(currentTag.ID != -1){
					qDebug() << "Tag encontrada, vamos a la siguiente!";
					gotopoint_proxy->stop()
					ss = supervisorStatus::SEARCH;
				} else gotopoint_proxy->turn()
			}
			break;
	}
	
}


void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	currentTag.set(tags.front());
}


