/*
 *    Copyright (C)2020 by YOUR NAME HERE
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

// THIS IS AN AGENT


#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>

#include <grid.h>
#include <controller.h>
#include <navigation.h>
#include <algorithm>
#include <cppitertools/zip.hpp>

using namespace std;

#define USE_QTGUI
#ifdef USE_QTGUI
	#include "innerviewer.h"
#endif

class SpecificWorker : public GenericWorker
{

Q_OBJECT
public:

	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	void RCISMousePicker_setPick(const RoboCompRCISMousePicker::Pick &myPick);

	using InnerPtr = std::shared_ptr<InnerModel>;
	#ifdef USE_QTGUI
		using InnerViewerPtr = std::shared_ptr<InnerViewer>;
	#endif


public slots:
    void compute();
    void initialize(int period);
    void checkRobotAutoMovState();
    void moveRobot();
    void sendRobotTo();
	void forcesSliderChanged(int value = 0);

    //Specification slot methods State Machine
	void sm_compute();
    void sm_initialize();

	void sm_finalize();

//--------------------
private:
	InnerPtr innerModel;
	std::string action;
//	ParameterMap params;
    std::string robotName;


    #ifdef USE_QTGUI
        InnerViewerPtr viewer;
    #endif

    std::shared_ptr<RoboCompCommonBehavior::ParameterList> confParams;
    Navigation<Grid<>,Controller> navigation;


    RoboCompLaser::TLaserData updateLaser();




};

#endif
