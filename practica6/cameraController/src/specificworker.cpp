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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx), m_tagCodes(::AprilTags::tagCodes36h11)
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

	m_tagDetector = new ::AprilTags::TagDetector(::AprilTags::tagCodes36h11);

	qDebug() << "Creating RGBD Interface variables...";

	m_width = 640;
	m_height = 480;

	m_px = m_width/2;
	m_py = m_height/2;

	m_fx = innerModel->getNode<InnerModelRGBD>("rgbd")->getFocal();
	m_fy = innerModel->getNode<InnerModelRGBD>("rgbd")->getFocal();

	image_gray.create(m_height,m_width,CV_8UC1);
	image_color.create(m_height,m_width,CV_8UC3);

	
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:0-10");
		qDebug() << "ID:0-10" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=0;i<=10; i++)
			tagsSizeMap.insert( i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) {  qFatal("Error reading config params");}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:11-20");
		qDebug() << "ID:11-20" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=11;i<=20; i++)
			tagsSizeMap.insert(i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}

	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("ID:21-100");
		qDebug() << "ID:21-100" << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		for(int i=21;i<=100; i++)
			tagsSizeMap.insert(i, QString::fromStdString(par.value).toFloat());
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}


	// Default value for IDs not defined before
	try
	{
		RoboCompCommonBehavior::Parameter par = params.at("AprilTagsSize");
		qDebug() << QString::fromStdString(par.value);
		Q_ASSERT(par.value > 0);
		m_tagSize = QString::fromStdString(par.value).toFloat();
	}
	catch(std::exception e) { std::cout << e.what() << std::endl;}


	timer.start(10);

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static RoboCompRGBD::ColorSeq colorseq;
	try
	{
		rgbd_proxy->getRGB(colorseq, hState, bState);
		memcpy(image_color.data , &colorseq[0], m_height*m_width*3);
		cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
		searchTags(image_gray);
		usleep(200000);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading form RGBD " << e << std::endl;
	}
}

void SpecificWorker::searchTags(const cv::Mat &image_gray)
{
    cv::Mat dst = image_gray;          // dst must be a different Mat
    vector<::AprilTags::TagDetection> detections = m_tagDetector->extractTags(dst);
	if(!detections.empty()) {
		qDebug() << "Found tags: "  << detections.size();
		print_detection(detections);
	}
}

void SpecificWorker::print_detection(vector< ::AprilTags::TagDetection> detections){

	listaDeMarcas.clear();
	detections2send.clear();

	while (!detections.empty()){
		// On this way, element is not copied, so handles memory on a better way.
		::AprilTags::TagDetection detection = detections.back();
		detections.pop_back();

		qDebug() << "  Id: " << detection.id << " (Hamming: " << detection.hammingDistance << ")" << endl;

		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;

		///SIN PROBAR PERO DEBERIA IR. SI NO ENCUNETRA EL ID METE m_tagSize
		const float ss = tagsSizeMap.value(detection.id, m_tagSize);

		detection.getRelativeTranslationRotation(ss, m_fx, m_fy, m_px, m_py, translation, rotation);
		QVec T(3);
		T(0) = -translation(1);//*0.65;
		T(1) =  translation(2);//*0.65;
		T(2) =  translation(0);//*0.65;

		Eigen::Matrix3d F;
		F << 1, 0,  0,	0,  -1,  0,	0,  0,  1;
		Eigen::Matrix3d fixed_rot = F*rotation;

		double rx, ry, rz;
		rotationFromMatrix(fixed_rot, rx, ry, rz);

		qDebug() << "  distance=" << T.norm2() << ", x=" << T(0) << ", y=" << T(1) << ", z=" << T(2) << ", rx=" << rx << ", ry=" << ry << ", rz=" << rz << endl;
		sender.sendTagFound(detection.id, detection.hammingDistance, bState.x, bState.z);

		RoboCompAprilTags::tag t;
		RoboCompGetAprilTags::marca mar;

		t.id=detection.id;
		t.tx=T(0);
		t.ty=T(1);
		t.tz=T(2);
		//Change the x,y,z rotation values to match robocomp's way
		t.rx=rx;
		t.ry=ry;
		t.rz=rz;
		t.cameraId = "rgbd";

		memcpy(&mar, &t, sizeof(RoboCompGetAprilTags::marca));
		mutex->lock();
		detections2send.push_back(t);
		listaDeMarcas.push_back(mar);
		mutex->unlock();
	}

	if (!detections2send.empty())
	{
		try
		{
			apriltags_proxy->newAprilTag(detections2send);
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
		}
		try
		{
			// qDebug()<<"bState"<<bState.correctedX<<bState.correctedZ<<bState.correctedAlpha;
			apriltags_proxy->newAprilTagAndPose(detections2send,bState,hState);
		}
		catch(const Ice::Exception &ex)
		{
			std::cout << ex << std::endl;
		}
	}
}

void SpecificWorker::rotationFromMatrix(const Eigen::Matrix3d &R, double &rx, double &ry, double &rz)
{
	QMat v(3,3);
	for (uint32_t f=0; f<3; f++)
	{
		for (uint32_t c=0; c<3; c++)
		{
			v(f,c) = R(f,c);
		}
	}
	QVec ret = v.extractAnglesR_min();
	rx = ret(0)+M_PIl;
	while (rx >= M_PIl) rx -= 2.*M_PIl;
	while (rx < -M_PIl) rx += 2.*M_PIl;
	ry = -ret(1);
	rz = ret(2);

/*
	yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
	double c = cos(yaw);
	double s = sin(yaw);
	pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
	roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
*/
}



listaMarcas SpecificWorker::checkMarcas()
{
  QMutexLocker locker(mutex);
  return  listaDeMarcas;
}


