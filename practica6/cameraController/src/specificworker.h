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

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "TelegramHandler.h"

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	listaMarcas checkMarcas();

public slots:
	void compute();

private:
	std::shared_ptr<InnerModel> innerModel;
	// Tag detection instruments
	::AprilTags::TagDetector* m_tagDetector;
	::AprilTags::TagCodes m_tagCodes;

	RoboCompGenericBase::TBaseState bState;
	RoboCompJointMotor::MotorStateMap hState;

	QMap<int, float> tagsSizeMap;
	double m_tagSize;

	// Video settings
	int m_width;
	int m_height;

	// Img containers
	cv::Mat image_gray, image_color;

	// Searching for tags
	void searchTags(const cv::Mat &image_gray);
	void print_detection(vector< ::AprilTags::TagDetection> detections);

	// Telegram Sender
	TelegramHandler sender;

};

#endif
