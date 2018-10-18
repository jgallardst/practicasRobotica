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

// Gaussian distribution
static float F2(float x){
	static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - 0) / 0.2;

    return inv_sqrt_2pi / 0.2 * std::exp(-0.5f * a * a);
}

static float F1(float norm){
	if(norm > 200) return 1;
	if(norm == 0) return 0;
	else return (norm/200.0);
}

void SpecificWorker::compute( )
{		
    try
    {		
		RoboCompGenericBase::TBaseState bState;
		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);

		target_t target;
		if(std::get<0>(t.activoAndGet())){
			target = std::get<1>(t.activoAndGet());
		} else return;

		

		// RELATIVE FROM INNERMODEL: auto relative =  (innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world"));
		// 2 Factores, pendiente y gaussiana segun el angulo
		// RELATIVE USING ALGEBRA
		Rot2D rot( bState.alpha);
		auto sub = QVec::vec2(target.x - bState.x, target.z - bState.z);
		auto relative = rot.invert() * (sub);

		float angle = atan2(relative.x(), relative.y());
		float mod = relative.norm2();

		if(mod < 50)
		{
			if(std::get<0>(t.activoAndGet())) t.setInactive();
			differentialrobot_proxy->setSpeedBase(0, 0); 
			return;
		} 
		differentialrobot_proxy->setSpeedBase(1000 * F1(mod) * F2(angle), angle); 


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



