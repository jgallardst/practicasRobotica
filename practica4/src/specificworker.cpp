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

RoboCompLaser::TLaserData SpecificWorker::trimLaser(int cut){
	RoboCompLaser::TLaserData ldataTrimmed;
        for(int i = cut; i <= (100-cut); i++){
 		ldataTrimmed.push_back(ldata.at(i));
	}
	return ldataTrimmed;
}

bool SpecificWorker::checkObstacle(int distance)
{
	RoboCompLaser::TLaserData ldataTrimmed = trimLaser(15);
    std::sort( ldataTrimmed.begin(), ldataTrimmed.end(), [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;
	qDebug() << ldataTrimmed.front().dist;
    return ( ldataTrimmed.front().dist < distance );
  
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

void SpecificWorker::goToTarget(){
	if(checkObstacle(300)){
		qDebug() << "GOTO -> GIRO";
		bs = botState::GIRO;
		return;
	}

	auto relative =  (innerModel->transform("base", QVec::vec3(target.x, 0, target.z), "world"));

	float angle = atan2(relative.x(), relative.z());
	float mod = relative.norm2();

	if(mod < 200){
		qDebug() << "GOTO -> IDLE";
		bs = botState::IDLE;
		differentialrobot_proxy->stopBase();
		t.setInactive();
		return;
	}
	try
  	{
		differentialrobot_proxy->setSpeedBase(400 * F1(mod) * F2(angle), angle); 
 	}
  	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }

}

float SpecificWorker::controlIzquierda()
{
	int control = 92;
	float c = ldata[control].dist;
	for(int i=control-8; i<control+8;i++)
	{
		if (ldata[i].dist < c)
			c = ldata[i].dist;
	}
	return c;
}

bool SpecificWorker::targetAtSight(){
	QPolygonF poly;
	for(auto l : trimLaser(10)){
		auto lr = innerModel->laserTo("world", "laser", l.dist, l.angle);
	   	poly << QPoint(lr.x(), lr.z());
	}
	return (controlIzquierda() > 350 && poly.containsPoint(QPoint(target.x, target.z), Qt::WindingFill));
}

void SpecificWorker::girar(){
	if(!checkObstacle(375)){
		bs = botState::BUG;
		qDebug()  << "GIRO -> BUG";
		performed = false;
		differentialrobot_proxy->setSpeedBase(0, 0);
		return;
	}
	try
	{
		differentialrobot_proxy->setSpeedBase(0, 0.5);
	}
  catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
}

void SpecificWorker::bug(){

	if(performed)
		lineDiff = fabs(mline.perpendicularDistanceToPoint(QVec::vec3(bState.x, 0, bState.z)));
	else lineDiff = 100;

	if(checkObstacle(350)){
		bs = botState::GIRO;
		qDebug()  << "BUG -> GIRO";
		differentialrobot_proxy->setSpeedBase(0, 0);
		return;
	}

	if(targetAtSight()){
		bs = botState::GOTO;
		qDebug() << "TARGET AT SIGHT: BUG -> GOTO";
		return;
	}
	if( lineDiff < 50){
		bs = botState::GOTO;
		qDebug() << "MLINE NEAR: BUG -> GOTO";
		return;
	}

	float const alfa = log( 0.1) / log( 0.3);
  	float dist = controlIzquierda();

	float k=0.1;
  	float angle =  -((1./(1. + exp(-k*(dist - 450.))))-1./2.);
  	float speed = 350 * exp ( - ( fabs ( angle ) * alfa ) ); 
	performed = true;
	try
  	{
  		differentialrobot_proxy->setSpeedBase(speed, angle); 
 	}
  	catch ( const Ice::Exception &ex ) {  std::cout << ex << std::endl; }
}

void SpecificWorker::compute( )
{		
    try
    {		
		differentialrobot_proxy->getBaseState(bState);
		ldata = laser_proxy->getLaserData();
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
			case botState::GIRO:
				girar();
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
  mline = QLine2D(QVec::vec3(bState.x, 0, bState.z), QVec::vec3(target.x, 0, target.z));
}



