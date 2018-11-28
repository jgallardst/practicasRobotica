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

	m_tagDetector = new AprilTags::TagDetector(AprilTags::tagCodes36h11);

	qDebug() << "Creating RGBD Interface variables...";
	image_gray.create(height,width,CV_8UC1);
	image_color.create(height,width,CV_8UC3);

	timer.start(Period);

	return true;
}

void SpecificWorker::compute()
{
	QMutexLocker locker(mutex);
	static RoboCompRGBD::ColorSeq colorseq;
	try
	{
		rgbd_proxy->getRGB(colorseq, hState, bState);
		innerModel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
		memcpy(image_color.data , &colorseq[0], width*height*3);
		cv::cvtColor(image_color, image_gray, CV_RGB2GRAY);
		searchTags(image_gray);
		usleep(2000000);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << "Error reading form RGBD " << e << std::endl;
	}
}

void SpecificWorker::searchTags(const cv::Mat &image_gray)
{
    cv::Mat dst = image_gray;          // dst must be a different Mat
    cv::flip(image_gray, dst, 0);
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(dst);
	qDebug() << "Found tags:"  << detections.size();
	if(detections.size() > 0) {
		sender.sendTagFound();
	}
}


listaMarcas SpecificWorker::checkMarcas()
{
//implementCODE

}


