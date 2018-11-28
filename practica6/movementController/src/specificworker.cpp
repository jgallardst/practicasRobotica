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
	std::cout << std::boolalpha;   
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
		innerModel = std::make_shared<InnerModel>(par.value);
	}
	catch(std::exception e) { qFatal("Error reading config params"); }

	qDebug() << __FILE__ ;
	
	// Scene
	scene.setSceneRect(-2500, -2500, 5000, 5000);
	view.setScene(&scene);
	view.scale(1, -1);
	view.setParent(scrollArea);
	//view.setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
	view.fitInView(scene.sceneRect(), Qt::KeepAspectRatio );

	grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{0, true, false, nullptr, 0.} );
	
	for(auto &[key, value] : grid)
	{
		auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));
		tile->setPos(key.x,key.z);
		value.rect = tile;
	}

	robot = scene.addRect(QRectF(-200, -200, 400, 400), QPen(), QBrush(Qt::blue));
	noserobot = new QGraphicsEllipseItem(-50,100, 100,100, robot);
	noserobot->setBrush(Qt::magenta);
	
	//qDebug() << __FILE__ << __FUNCTION__ << "CPP " << __cplusplus;
	
	connect(buttonSave, SIGNAL(clicked()), this, SLOT(saveToFile()));
	connect(buttonRead, SIGNAL(clicked()), this, SLOT(readFromFile()));
	
	timer.start();
	// AutoLoad Map
	readFromFile();


	// Init
	RoboCompGenericBase::TBaseState bState;
 	differentialrobot_proxy->getBaseState(bState);
	currentPoint = QVec::vec3(bState.x,0,bState.z);
	return true;
}

// Gaussian distribution
static float F2(float x){
	static const float inv_sqrt_2pi = 0.3989422804014327;
    float a = (x - 0) / 0.2;

    return inv_sqrt_2pi / 0.2 * std::exp(-0.5f * a * a);
}

static float F1(float norm){
	if(norm > 50) return 1;
	if(norm == 0) return 0;
	else return (norm/50.0);
}

std::list<QVec> SpecificWorker::bezierTransform(std::list<QVec> points, float accuracy) {
	if (points.size() <= 2) return points;
	std::vector<QVec> bezierVec;
	std::list<QVec> bezier;
	while(!points.empty()){
		bezierVec.push_back(points.back());
		points.pop_back();
	}
	bezier.push_front(bezierVec[0]);
    for(float i=0.f; i<1.f; i+=1.f/accuracy){
		std::vector<QVec> temp;
        for(unsigned int j=1; j<bezierVec.size(); ++j)
            temp.push_back(QVec::vec3(interpolate(bezierVec[j-1].x(), bezierVec[j].x(), i), 0,
                                    interpolate(bezierVec[j-1].z(), bezierVec[j].z(), i)));

        while(temp.size()>1)
        {
            std::vector<QVec> temp2;

            for(unsigned int j=1; j<temp.size(); ++j)
                temp2.push_back(QVec::vec3(interpolate(temp[j-1].x(), temp[j].x(), i), 0,
                                         interpolate(temp[j-1].z(), temp[j].z(), i)));
            temp = temp2;
        }
        bezier.push_front(temp[0]);

	}
	return bezier;
}

void SpecificWorker::compute()
{
	static RoboCompGenericBase::TBaseState bState;
 	try
 	{
 		differentialrobot_proxy->getBaseState(bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();
		
		//draw robot
		robot->setPos(bState.x, bState.z);
		robot->setRotation(-180.*bState.alpha/M_PI);
		
		//updateVisitedCells(bState.x, bState.z);
		updateOccupiedCells(bState, ldata);

		if(std::get<0>(t.activoAndGet())){
				target = std::get<1>(t.activoAndGet());
				path = grid.getOptimalPath(QVec::vec3(bState.x,0,bState.z), QVec::vec3(target.x,0,target.z));
				for(auto &p: path)
					redPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::red), QBrush(Qt::red)));
				bezier = this->bezierTransform(path, (int)(1.5*path.size()));
				if(!bezier.empty()){
					for(auto &p: bezier)
						greenPath.push_back(scene.addEllipse(p.x(),p.z(), 100, 100, QPen(Qt::green), QBrush(Qt::green)));
					currentPoint = bezier.front();
					bezier.pop_front();
				}
				t.setInactive();
		}

		auto relative =  (innerModel->transform("base", QVec::vec3(currentPoint.x(), 0, currentPoint.z()), "world"));
		float angle = atan2(relative.x(), relative.z());
		float mod = relative.norm2();

		

		if(mod < 100)
		{
			if(bezier.empty())
			{
				qDebug() << "Aligned with target";
				currentPoint =  QVec::vec3(target.x,0,target.z);
				differentialrobot_proxy->setSpeedBase(0, 0); 
			} else {
				qDebug() << "Picking new point";
				currentPoint = bezier.front();
				bezier.pop_front();
			}
		}
		else {
			differentialrobot_proxy->setSpeedBase(400 * F1(mod) * F2(angle), angle); 
		}
	}
 	catch(const Ice::Exception &e)
	{	std::cout  << e << std::endl; }
	
	//Resize world widget if necessary, and render the world
	if (view.size() != scrollArea->size())
			view.setFixedSize(scrollArea->width(), scrollArea->height());
	draw();
	
}

void SpecificWorker::saveToFile()
{
	grid.saveToFile(fileName);
}

void SpecificWorker::readFromFile()
{
	std::ifstream myfile;
	myfile.open(fileName, std::ifstream::in);
	
	if(!myfile.fail())
	{
		//grid.initialize( TDim{ tilesize, -2500, 2500, -2500, 2500}, TCell{true, false, nullptr} );
		for( auto &[k,v] : grid)
			delete v.rect;
		grid.clear();
		Grid<TCell>::Key key; TCell value;
		myfile >> key >> value;
		int k=0;
		while(!myfile.eof()) 
		{
			auto tile = scene.addRect(-tilesize/2,-tilesize/2, 100,100, QPen(Qt::NoPen));;
			tile->setPos(key.x,key.z);
			value.rect = tile;
			value.id = k++;
			value.cost = 1;
			grid.insert<TCell>(key,value);
			myfile >> key >> value;
		}
		myfile.close();	
		robot->setZValue(1);
		std::cout << grid.size() << " elements read to grid " << fileName << std::endl;
	}
	else
		throw std::runtime_error("Cannot open file");
}

void SpecificWorker::updateOccupiedCells(const RoboCompGenericBase::TBaseState &bState, const RoboCompLaser::TLaserData &ldata)
{
	auto *n = innerModel->getNode<InnerModelLaser>("laser");
	for(auto l: ldata)
	{
		auto r = n->laserTo("world", l.dist, l.angle);	// r is in world reference system
		// we set the cell corresponding to r as occupied 
		auto [valid, cell] = grid.getCell(r.x(), r.z()); 
		if(valid)
			cell.free = false;
	}
}

void SpecificWorker::updateVisitedCells(int x, int z)
{
	static unsigned int cont = 0;
	auto [valid, cell] = grid.getCell(x, z); 
	if(valid)
	{
		auto &occupied = cell.visited;
		if(occupied)
		{
			occupied = false;
			cont++;
		}
		float percentOccupacy = 100. * cont / grid.size();
	}
}

void SpecificWorker::draw()
{
	for(auto &[key, value] : grid)
	{
// 		if(value.visited == false)
// 			value.rect->setBrush(Qt::lightGray);
		if(value.free == false)
			value.rect->setBrush(Qt::darkRed);
	}
	view.show();
}

/////////////// PATH PLANNING /////7




/////////////////////////////////////////////////////////77
/////////
//////////////////////////////////////////////////////////

void SpecificWorker::setPick(const Pick &myPick)
{
	qDebug() << "PRESSED ON: X: " << myPick.x << " Z: " << myPick.z;
  	t.set(myPick.x, myPick.z);
	for(auto gp: greenPath)
		delete gp;
	greenPath.clear();
	for(auto gp: redPath)
		delete gp;
	redPath.clear();
	path.clear();
	bezier.clear();
}
