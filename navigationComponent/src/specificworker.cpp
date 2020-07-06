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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{

    innerModel = std::make_shared<InnerModel>(); //Leer de fichero

}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	emit t_compute_to_finalize();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{

    confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);

    try
    {

        try{ robotName = params.at("NavigationAgent.RobotName").value;}
        catch(std::exception e) { qFatal("Error reading robotName"); }

        std::string innermodel_path;

        try{ innermodel_path = params.at("NavigationAgent.InnerModelPath").value;}
        catch(std::exception e) { qFatal("Error reading innerModel Path"); }


        qDebug()<<"InnerModelPath: " << QString::fromStdString(innermodel_path);




        innerModel = std::make_shared<InnerModel> (innermodel_path);


        //-------> LA CONFIGURACION SE ENCUENTRA EN specificmonitor.cpp <-------

        // PASAMOS LA LISTA DE PARAMETROS A navigation.h
        confParams  = std::make_shared<RoboCompCommonBehavior::ParameterList>(params);

    }
    catch(std::exception e) { qFatal("Error reading config params"); }

	defaultMachine.start();

	return true;
}

void SpecificWorker::initialize(int period)
{
    QMutexLocker lockIM(mutex);

    std::cout << "Initialize worker" << std::endl;

    connect(autoMov_checkbox, SIGNAL(clicked()),this, SLOT(checkRobotAutoMovState()));
    connect(robotMov_checkbox, SIGNAL(clicked()),this, SLOT(moveRobot()));

    connect(ki_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));
    connect(ke_slider, SIGNAL (valueChanged(int)),this,SLOT(forcesSliderChanged(int)));

    connect(send_button, SIGNAL(clicked()),this, SLOT(sendRobotTo()));


    forcesSliderChanged();
    moveRobot();

#ifdef USE_QTGUI
	viewer = std::make_shared<InnerViewer>(innerModel, "Social Navigation");  //InnerViewer copies internally innerModel so it has to be resynchronized
#endif

    navigation.initialize(innerModel, viewer, confParams, omnirobot_proxy);

    qDebug()<<"Classes initialized correctly";

    this->Period = period;
    timer.start(period);


    emit this->t_initialize_to_compute();

    std::cout << "[END] Initialize worker" << std::endl;


}

void SpecificWorker::compute()
{
    qDebug()<< __FUNCTION__;

    //actualizamos innerModel
    string RobotName = "robot";
    auto currentRobotPose = innerModel->transformS6D("world",RobotName);
    qDebug()<< __FUNCTION__<< " ---- Robot pre: "<< currentRobotPose;
    int xpos;
    int ypos;
    float angle;
    omnirobot_proxy->getBasePose(xpos, ypos, angle);
    qDebug()<< __FUNCTION__<< " ---- omnirobot_proxy.getBasePose:   x:"<< xpos << "    y:" << ypos << "    a:" << angle;

    //actualizando innerModel
    innerModel->updateTransformValuesS(RobotName, xpos, currentRobotPose.z(), ypos, currentRobotPose.rx(), angle, currentRobotPose.rz());

    currentRobotPose = innerModel->transformS6D("world", RobotName); // esta linea necesita el nombre del robot (esta en configparams)
    qDebug()<< __FUNCTION__<< " ---- Robot new: "<< currentRobotPose;

//    static QTime reloj = QTime::currentTime();

    bool needsReplaning = false;


    QMutexLocker lockIM(mutex);

    RoboCompLaser::TLaserData laserData = updateLaser();

	navigation.update( laserData, needsReplaning);

//    static QTime reloj = QTime::currentTime();

    viewer->run();
//    qDebug()<< "viewer " << reloj.restart();

}



RoboCompLaser::TLaserData  SpecificWorker::updateLaser()
{
	qDebug()<<__FUNCTION__;

	RoboCompLaser::TLaserData laserData;

    try
    {
		laserData  = laser_proxy->getLaserData();
    }

    catch(const Ice::Exception &e){ std::cout <<"Can't connect to laser --" <<e.what() << std::endl; };

    return laserData;
}



void  SpecificWorker::moveRobot()
{
    qDebug()<<__FUNCTION__;

    if(robotMov_checkbox->checkState() == Qt::CheckState(2))
    {
        autoMov_checkbox->setEnabled(true);
        navigation.moveRobot = true;
		navigation.stopMovingRobot = false;
    }

    else
    {
        if(navigation.current_target.active.load())
			navigation.stopMovingRobot = true;

        else
		{
            navigation.moveRobot = false;
			navigation.stopMovingRobot = false;
		}

        autoMov_checkbox->setEnabled(false);

    }

}


void  SpecificWorker::checkRobotAutoMovState()
{
	qDebug()<<__FUNCTION__;

	if(autoMov_checkbox->checkState() == Qt::CheckState(2))
	{
		navigation.robotAutoMov = true;
		navigation.newRandomTarget();
	}

	else
    {
        navigation.robotAutoMov = false;
    }

}


void SpecificWorker::sendRobotTo()
{
    auto x =  x_spinbox->value();
    auto z =  z_spinbox->value();

    navigation.newTarget(QPointF(x,z));

}


void SpecificWorker::
forcesSliderChanged(int value)
{

    navigation.KI = (float) ki_slider -> sliderPosition();
    navigation.KE = (float) ke_slider -> sliderPosition();

}


void SpecificWorker::sm_compute()
{
//	std::cout<<"Entered state compute"<<std::endl;
	compute();
}

void SpecificWorker::sm_initialize()
{
	std::cout<<"Entered initial state initialize"<<std::endl;
}

void SpecificWorker::sm_finalize()
{
	std::cout<<"Entered final state finalize"<<std::endl;
}

////////////////////////// SUBSCRIPTIONS /////////////////////////////////////////////

void SpecificWorker::RCISMousePicker_setPick(const RoboCompRCISMousePicker::Pick &myPick)
{
    navigation.newTarget(QPointF(myPick.x,myPick.z));
}


///////////////////////////////////////////////////////////////////////////////////////
