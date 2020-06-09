//
// Created by robolab on 17/01/20.
//

#ifndef PROJECT_NAVIGATION_H
#define PROJECT_NAVIGATION_H



#include <innermodel/innermodel.h>
#include <math.h>

#include <genericworker.h>
#include <Laser.h>
#include "collisions.h"
#include <QPolygonF>
#include <QPointF>

#include <cppitertools/chain.hpp>
#include <cppitertools/range.hpp>
#include <cppitertools/sliding_window.hpp>
#include <cppitertools/zip.hpp>
#include <cppitertools/filter.hpp>
#include <cppitertools/enumerate.hpp>
#include <cppitertools/slice.hpp>
#include <algorithm>
//#include <localPerson.h>

// Map
struct TMapDefault
{

};

struct TContDefault
{
    TContDefault(){};

};

template<typename TMap = TMapDefault, typename TController = TContDefault>
class Navigation
{
public:

    // Target
    struct Target : public std::mutex
    {
        QPointF p;
        std::atomic_bool active = false;
        std::atomic_bool blocked = true;
        std::atomic_bool humanBlock = false;
    };

    Target current_target;

    bool robotAutoMov = false;
    bool moveRobot = false;

    bool stopMovingRobot = false;

    float KE;
    float KI;

    vector<int32_t> blockIDs;
    vector<vector<int32_t>> softBlockIDs;

	//localPersonsVec totalPersons;

    //robot data
    string RobotName;
    string LaserName; 

void initialize(const std::shared_ptr<InnerModel> &innerModel_, std::shared_ptr< RoboCompCommonBehavior::ParameterList > configparams_, RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy_)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Starting initialize";

    innerModel = innerModel_;
    configparams = configparams_;

    //robot data
    RobotName = configparams->at("NavigationAgent.RobotName").value;

    if(RobotName.compare("base") == 0){
        LaserName = "laser";
    } else {
        LaserName = RobotName.back();
        LaserName.insert(0,"laser");
    }


    qDebug() << "-=-=-=-=-=-=-=-=-=-=  > " << QString::fromStdString(RobotName) << "  " << QString::fromStdString(LaserName) << endl; 


    omnirobot_proxy = omnirobot_proxy_;
    stopRobot(); //grid can't be initialized if the robot is moving

    collisions =  std::make_shared<Collisions>();

    collisions->initialize(innerModel, configparams); // mirar la funcion "checkRobotValidStateAtTargetFast()" de collision.h (fuera de src)
    grid.initialize(collisions);

    controller.initialize(innerModel,configparams);


    robotXWidth = std::stof(configparams->at("RobotXWidth").value);
    robotZLong = std::stof(configparams->at("RobotZLong").value);
    robotBottomLeft     = QVec::vec3( - robotXWidth / 2 - 100, 0, - robotZLong / 2 - 100);
    robotBottomRight    = QVec::vec3( + robotXWidth / 2 + 100, 0, - robotZLong / 2 - 100);
    robotTopRight       = QVec::vec3( + robotXWidth / 2 + 100, 0, + robotZLong / 2 + 100);
    robotTopLeft        = QVec::vec3( - robotXWidth / 2 - 100, 0, + robotZLong / 2 + 100);

    reloj.restart();

};

void updateInnerModel(const std::shared_ptr<InnerModel> &innerModel_)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): innerModel updated";

    innerModel = innerModel_;
    controller.updateInnerModel(innerModel);

};

void update(const RoboCompLaser::TLaserData &laserData_, bool needsReplaning)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Updating...";

    if (gridChanged)
    {
        updateFreeSpaceMap();
        gridChanged = false;
    }

    //totalPersons = totalPersons_;
    RoboCompLaser::TLaserData laserData;

    laserData = computeLaser(laserData_);  // esta linea necesita dentro el nombre del laser de cada oveja

    currentRobotPose = innerModel->transformS6D("world", RobotName); // esta linea necesita el nombre del robot (esta en configparams)

    updateLaserPolygon(laserData); // esta linea necesita dentro el nombre del laser de cada oveja    
    currentRobotPolygon = getRobotPolygon();
    currentRobotNose = getRobotNose();


    if(needsReplaning)
    {
        for (auto p: pathPoints)
        {
            if(std::any_of(std::begin(socialSpaces), std::end(socialSpaces),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
            or std::any_of(std::begin(personalSpaces), std::end(personalSpaces),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
            or std::any_of(std::begin(totalAffordances), std::end(totalAffordances),[p](const auto &poly) { return poly.containsPoint(p, Qt::OddEvenFill);})
           )
            {
                stopRobot();

                this->current_target.lock();
                    current_target.blocked.store(true);
                this->current_target.unlock();

                break;
            }
        }
    }

    if (checkPathState() == false)
        return;

    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Computing forces...";
    computeForces(pathPoints, laserData);

    cleanPoints();
    addPoints();



    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Updating Controller...";
    auto [blocked, active, xVel,zVel,rotVel] = controller.update(pathPoints, laserData, current_target.p, currentRobotPose);
    qDebug()<< "xVel "<<xVel << "zVel "<<zVel << "rotVel" << rotVel << "Blocked:" << blocked << "  Active:" << active << "   moveRobot:" << moveRobot;



    if (blocked)
    {
    	qDebug()<<"Navigation - "<< __FUNCTION__<<"(): [!] Robot Blocked [!]";
        stopRobot();

        this->current_target.lock();
            current_target.blocked.store(true);
        this->current_target.unlock();
    }

    if (!active)
    {
    	qDebug()<<"Navigation - "<< __FUNCTION__<<"(): [!] Robot Inactive [!]";
        stopRobot();

        this->current_target.lock();
            current_target.active.store(false);
        this->current_target.unlock();

        pathPoints.clear();

        if(stopMovingRobot) moveRobot = false;

        if(robotAutoMov) newRandomTarget();
    }

    if (!blocked and active)
    {
        if(moveRobot){
		omnirobot_proxy->setSpeedBase(xVel,zVel,rotVel);
	} else {
		qDebug()<<"Navigation - "<< __FUNCTION__<<"(): [!] Motionless Robot (use enableRobotMovement() to active robot movement ability) [!]";
	}
    }



};

void enableRobotMovement(){
	moveRobot = true;
}

void disableRobotMovement(){
	moveRobot = false;
}

bool isRobotMovementEnabled(){
	return moveRobot;
}


void stopRobot()
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Stopping robot";
    omnirobot_proxy->setSpeedBase(0,0,0);
}

bool checkPathState()
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Checking path state...";

    if (current_target.active.load())
    {
        if (current_target.blocked.load()) {

            if (!findNewPath()) {

                qDebug()<< "checkPathState - Path not found";

                if(current_target.humanBlock.load()) //if the path is blocked by human the target is not deactivated
                    return false;

                qDebug()<< "checkPathState - Deactivating current target";

                stopRobot();

                this->current_target.lock();
                current_target.active.store(false);
                this->current_target.unlock();

                pathPoints.clear();

                if(robotAutoMov) newRandomTarget();

                return false;
            }

            else{

                qDebug()<< "checkPathState - Path found";
                this->current_target.lock();
                    this->current_target.blocked.store(false);
                this->current_target.unlock();


                reloj.restart();
            }
        }

        return true;
    } else {
        qDebug()<< "checkPathState - no target set. Set one using newTarget() or newRandomTarget()";
        return false;
    }
}


void newRandomTarget()
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): New RANDOM target established";

    auto hmin = std::min(collisions->outerRegion.left(), collisions->outerRegion.right());
    auto width = std::max(collisions->outerRegion.left(), collisions->outerRegion.right()) - hmin;
    auto vmin = std::min(collisions->outerRegion.top(), collisions->outerRegion.bottom());
    auto height = std::max(collisions->outerRegion.top(), collisions->outerRegion.bottom()) - vmin;

    auto x = hmin + (double)rand() * width / (double)RAND_MAX;
    auto z = vmin + (double)rand() * height/ (double)RAND_MAX;


    this->current_target.lock();
        current_target.active.store(true);
        current_target.blocked.store(true);
    current_target.p = QPointF(x,z);

    this->current_target.unlock();

    qDebug()<<"New Random Target" << current_target.p;

}

void newTarget(QPointF newT)
{

    qDebug()<<"Navigation -"<< __FUNCTION__ <<"(): New Target established"<< newT;

    if(stopMovingRobot){
        stopRobot();
        moveRobot = false;
    }

    this->current_target.lock();
	current_target.active.store(true);
	current_target.blocked.store(true);
	current_target.humanBlock.store(false);
	current_target.p = newT;

    this->current_target.unlock();
}

void updatePersonalPolylines(vector<QPolygonF> intimateSpaces_, vector<QPolygonF> personalSpaces_, vector<QPolygonF> socialSpaces_){
    intimateSpaces = intimateSpaces_;
    personalSpaces = personalSpaces_;
    socialSpaces = socialSpaces_;

    gridChanged = true;
}

void updateAffordancesPolylines(std::map<float, vector<QPolygonF>> mapCostObjects_,
        vector<QPolygonF>totalAffordances_, vector<QPolygonF>affordancesBlocked_)
{
    totalAffordances = totalAffordances_;
    mapCostObjects = mapCostObjects_;
    affordancesBlocked = affordancesBlocked_;

    gridChanged = true;

    }



private:
    std::shared_ptr<Collisions> collisions;
    std::shared_ptr<InnerModel> innerModel;
    std::shared_ptr<RoboCompCommonBehavior::ParameterList> configparams;

    RoboCompOmniRobot::OmniRobotPrx omnirobot_proxy;

    typedef struct { float dist; float angle;} LocalPointPol;

    TMap grid;
    TController controller;

    // ElasticBand
    std::vector<QPointF> pathPoints;

    const float ROBOT_LENGTH = 400;
    const float ROAD_STEP_SEPARATION = ROBOT_LENGTH * 0.9;

    bool targetBehindRobot = false;

//Integrating time
    QTime reloj = QTime::currentTime();

    QPointF lastPointInPath, currentRobotNose;

    QPolygonF currentRobotPolygon, laser_poly;
    std::vector<QPointF> laser_cart;
    QVec currentRobotPose;

    float robotXWidth, robotZLong; //robot dimensions read from config
    QVec robotBottomLeft, robotBottomRight, robotTopRight, robotTopLeft;

    bool gridChanged = false;

    vector<QPolygonF> intimateSpaces,personalSpaces,socialSpaces, totalAffordances;
    vector<QPolygonF> affordancesBlocked;

    std::map<float, vector<QPolygonF>> mapCostObjects;

////////// GRID RELATED METHODS //////////
void updateFreeSpaceMap(bool drawGrid = true)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"()";

    grid.resetGrid();

    //To set occupied
    for (auto &&poly_intimate : iter::chain(intimateSpaces, affordancesBlocked))
        grid.markAreaInGridAs(poly_intimate, false);

    for(auto [cost,polygonVec] : mapCostObjects)
    {
        for (auto polygon : polygonVec)
            grid.modifyCostInGrid(polygon, cost);
    }

    for (auto &&poly_soc : socialSpaces)
        grid.modifyCostInGrid(poly_soc, 4.0);

    for (auto &&poly_per : personalSpaces)
        grid.modifyCostInGrid(poly_per, 6.0);


}

////////// CONTROLLER RELATED METHODS //////////
RoboCompLaser::TLaserData computeLaser(RoboCompLaser::TLaserData laserData)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Reading laser";


    auto lasernode = innerModel->getNode<InnerModelLaser>(LaserName);

    RoboCompLaser::TLaserData laserCombined;
    laserCombined = laserData;


    for (const auto &polyline : iter::chain(intimateSpaces,affordancesBlocked))
    {
        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::min();

        for (const auto &polylinePoint: polyline)
        {
            LocalPointPol lPol;
            QVec pInLaser = innerModel->transform(QString::fromStdString(LaserName), QVec::vec3(polylinePoint.x(), 0, polylinePoint.y()), "world");

            lPol.angle = atan2(pInLaser.x(), pInLaser.z());
            lPol.dist = sqrt(pow(pInLaser.x(),2) + pow(pInLaser.z(),2));

            if( lPol.angle < min ) min = lPol.angle;
            if( lPol.angle > max ) max = lPol.angle;

        }

        //Recorremos todas las muestras del laser
        for (auto &laserSample: laserCombined)
        {
            //Compruebo que la muestra del laser corta a la polilinea. Es decir si esta comprendida entre el maximo y el minimo de antes
            if (laserSample.angle >= min and laserSample.angle <= max and fabs(max-min) < 3.14 )
            {
                QVec lasercart = lasernode->laserTo(LaserName,laserSample.dist, laserSample.angle);

                //recta que une el 0,0 con el punto del laser
                QVec robotL = innerModel->transform(QString::fromStdString(RobotName), QString::fromStdString(LaserName));
                QLineF laserline(QPointF(robotL.x(), robotL.z()), QPointF(lasercart.x(), lasercart.z()));

                auto previousPoint = polyline[polyline.size()-1];
                QVec previousPointInLaser = innerModel->transform(QString::fromStdString(LaserName), (QVec::vec3(previousPoint.x(), 0, previousPoint.y())), "world");

                for (const auto &polylinePoint: polyline)
                {
                    QVec currentPointInLaser = innerModel->transform(QString::fromStdString(LaserName), (QVec::vec3(polylinePoint.x(), 0, polylinePoint.y())), "world");
                    QLineF polygonLine(QPointF(previousPointInLaser.x(), previousPointInLaser.z()), QPointF(currentPointInLaser.x(), currentPointInLaser.z()));

                    QPointF intersection;
                    auto intersectionType = laserline.intersect(polygonLine, &intersection);
                    float dist = QVector2D(intersection.x()-robotL.x(),intersection.y()-robotL.z()).length();

                    if ((intersectionType == QLineF::BoundedIntersection) and (dist<laserSample.dist))
                        laserSample.dist =  dist;

                    previousPointInLaser = currentPointInLaser;

                }
            }
        }
    }

    return laserCombined;

}

//CONTROLLER METHODS

bool findNewPath()
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Finding a new path to target...";
    pathPoints.clear();

	blockIDs.clear();
	softBlockIDs.clear();

    // extract target from current_path
    this->current_target.lock();
        auto target = this->current_target.p;
    this->current_target.unlock();

    std::list<QPointF> path = grid.computePath(currentRobotNose, target);

    if (path.size() > 0)
    {
        qDebug() << __FUNCTION__ << "(): Path with coherent size";
        pathPoints.push_back(currentRobotNose);

        for (const QPointF &p : path)
            pathPoints.push_back(p);

        lastPointInPath = pathPoints[pathPoints.size()-1];


	/*
        if(checkHumanSoftBlock())
        {
            this->current_target.lock();
                current_target.humanBlock.store(true);
            this->current_target.unlock();

            return false;
        }
        */
        qDebug() << __FUNCTION__ << "(): Path found";
        return true;
    }

    else
    {
        qDebug() << __FUNCTION__ << "(): Path not found";

	/*
        if(checkHumanBlock())
        {
            this->current_target.lock();
                current_target.humanBlock.store(true);
            this->current_target.unlock();

        }
	*/

        return false;
    }
}

/*
bool checkHumanBlock()
{
    qDebug()<<__FUNCTION__;

    grid.resetGrid();

    bool blockFound = false;

    this->current_target.lock();
    	auto target = this->current_target.p;
    this->current_target.unlock();

    std::list<QPointF> path = grid.computePath(currentRobotNose, target);

    if(!path.empty()) //hay alguna persona bloqueando el camino
    {
        for(auto pol : intimateSpaces) //change to intimateSpaces
        {
            grid.markAreaInGridAs(pol, false);
            path = grid.computePath(currentRobotNose, target);

            if (path.empty())
            {
                blockFound = true;

				for (auto p: totalPersons)
				{
					if (pol.containsPoint(QPointF(p.x, p.z), Qt::OddEvenFill))
						blockIDs.push_back(p.id);
				}

                break;
            }
        }
    }

    //he eliminado todas las polilíneas, con esto las vuelvo a dibujar
    updateFreeSpaceMap(false);

    return blockFound;
}

bool checkHumanSoftBlock()
{
    qDebug()<<__FUNCTION__;

    vector<QPolygonF> softBlockPolygonList;
    bool softBlockFound = false;

    for(auto pol : personalSpaces) //change to intimateSpaces
    {
        for (auto p: pathPoints)
        {
            if (pol.containsPoint(p, Qt::OddEvenFill))
            {
                softBlockFound = true;
                softBlockPolygonList.push_back(pol);
                break;
            }
        }
    }

    if(softBlockFound)
	{
    	for (auto polygon : softBlockPolygonList)
		{
    		vector<int32_t> groupID;

			for (auto p: totalPersons)
			{
				if (polygon.containsPoint(QPointF(p.x, p.z), Qt::OddEvenFill))
					groupID.push_back(p.id);
			}

			softBlockIDs.push_back(groupID);
		}
	}

    return softBlockFound;
}
*/

bool isVisible(QPointF p)
{
    QVec pointInLaser = innerModel->transform(QString::fromStdString(LaserName), QVec::vec3(p.x(),0,p.y()),"world");
    return laser_poly.containsPoint(QPointF(pointInLaser.x(),pointInLaser.z()), Qt::OddEvenFill);
}


void computeForces(const std::vector<QPointF> &path, const RoboCompLaser::TLaserData &lData)
{
    qDebug()<<"Navigation - "<< __FUNCTION__<<"(): Path size = "<< path.size();

    if (path.size() < 3) {
  	qDebug()<<"Navigation - "<< __FUNCTION__<<"(): [!] Path size ("<< path.size() <<") less than 3. Too short! [!]";
        return;
    }

    // Go through points using a sliding windows of 3
    for (auto group : iter::sliding_window(path, 3))
    {
        if (group.size() < 3)
            break; // break if too short

        auto p1 = QVector2D(group[0]);
        auto p2 = QVector2D(group[1]);
        auto p3 = QVector2D(group[2]);
        auto p = group[1];

        if (isVisible(p) == false) // if not visible (computed before) continue
            continue;

        // INTERNAL curvature forces on p2
        QVector2D iforce = ((p1 - p2) / (p1 - p2).length() + (p3 - p2) / (p3 - p2).length());

        // EXTERNAL forces. We need the minimun distance from each point to the obstacle(s). we compute the shortest laser ray to each point in the path
        // compute minimun distances to each point within the laser field

        std::vector<std::tuple<float, QVector2D>> distances;
        // Apply to all laser points a functor to compute the distances to point p2
        std::transform(std::begin(laser_cart), std::end(laser_cart), std::back_inserter(distances), [p, this](QPointF &t) { //lasercart is updated in UpdateLaserPolygon
            // compute distante from laser tip to point minus RLENGTH/2 or 0 and keep it positive
            float dist = (QVector2D(p) - QVector2D(t)).length() - (ROBOT_LENGTH / 2);
            if (dist <= 0)
                dist = 0.01;
            return std::make_tuple(dist, QVector2D(p) - QVector2D(t));
        });

        // compute min distance
        auto min = std::min_element(std::begin(distances), std::end(distances), [](auto &a, auto &b) { return std::get<float>(a) < std::get<float>(b); });
        float min_dist = std::get<float>(*min);

        QVector2D force = std::get<QVector2D>(*min);
        // rescale min_dist so 1 is ROBOT_LENGTH
        float magnitude = (1.f / ROBOT_LENGTH) * min_dist;
        // compute inverse square law
        magnitude = 10.f / (magnitude * magnitude);
        //if(magnitude > 25) magnitude = 25.;
        QVector2D f_force = magnitude * force.normalized();
        //qDebug() << magnitude << f_force;

        // Remove tangential component of repulsion force by projecting on line tangent to path (base_line)
        QVector2D base_line = (p1 - p3).normalized();
        const QVector2D itangential = QVector2D::dotProduct(f_force, base_line) * base_line;
        f_force = f_force - itangential;

        // update node pos
        auto total = (KI * iforce) + (KE * f_force);

        // limiters CHECK!!!!!!!!!!!!!!!!!!!!!!
        if (total.length() > 30)
            total = 8 * total.normalized();
        if (total.length() < -30)
            total = -8 * total.normalized();

        // move node only if they do not exit the laser polygon and do not get inside objects or underneath the robot.
        QPointF temp_p = p + total.toPointF();
        if(isVisible(temp_p)
                and (!currentRobotPolygon.containsPoint(temp_p, Qt::OddEvenFill))
                and (std::none_of(std::begin(intimateSpaces), std::end(intimateSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);}))
                and (std::none_of(std::begin(personalSpaces), std::end(personalSpaces),[temp_p](const auto &poly) { return poly.containsPoint(temp_p, Qt::OddEvenFill);})))
            p = temp_p;
    }

    pathPoints[0] = currentRobotNose;
}


void addPoints()
{
        qDebug()<<"Navigation - "<< __FUNCTION__;

    std::vector<std::tuple<int, QPointF>> points_to_insert;
    for (auto &&[k, group] : iter::enumerate(iter::sliding_window(pathPoints, 2)))
    {
        auto &p1 = group[0];
        auto &p2 = group[1];

        if (isVisible(p1) == false or isVisible(p2) == false) //not visible
            continue;

        float dist = QVector2D(p1 - p2).length();

        if (dist > ROAD_STEP_SEPARATION)
        {
            float l = 0.9 * ROAD_STEP_SEPARATION / dist; //Crucial que el punto se ponga mas cerca que la condición de entrada
            QLineF line(p1, p2);
            points_to_insert.push_back(std::make_tuple(k + 1, QPointF{line.pointAt(l)}));
        }
        //qDebug() << __FUNCTION__ << k;
    }
    for (const auto &[l, p] : iter::enumerate(points_to_insert))
    {
        if(!currentRobotPolygon.containsPoint(std::get<QPointF>(p), Qt::OddEvenFill))
        {
//                qDebug()<< "Add points  " << std::get<QPointF>(p);

            pathPoints.insert(pathPoints.begin() + std::get<int>(p) + l, std::get<QPointF>(p));
        }

    }
//        qDebug() << __FUNCTION__ << "points inserted " << points_to_insert.size();
}

void cleanPoints()
{
        qDebug()<<"Navigation - "<< __FUNCTION__;

    std::vector<QPointF> points_to_remove;
    for (const auto &group : iter::sliding_window(pathPoints, 2))
    {
        const auto &p1 = group[0];
        const auto &p2 = group[1];

        if ((!isVisible(p1)) or (!isVisible(p2))) //not visible
            continue;

        if (p2 == lastPointInPath)
            break;
        // check if p1 was marked to erase in the previous iteration
        if (std::find(std::begin(points_to_remove), std::end(points_to_remove), p1) != std::end(points_to_remove))
            continue;

        float dist = QVector2D(p1 - p2).length();
        if (dist < 0.5 * ROAD_STEP_SEPARATION)
            points_to_remove.push_back(p2);

        else if(currentRobotPolygon.containsPoint(p2, Qt::OddEvenFill))
        {
            qDebug()<<"-------------" << __FUNCTION__ << "(): ------------- Removing point inside robot ";
            points_to_remove.push_back(p2);
        }

    }


    for (auto &&p : points_to_remove)
    {
        pathPoints.erase(std::remove_if(pathPoints.begin(), pathPoints.end(), [p](auto &r) { return p == r; }), pathPoints.end());

    }
}


/**
* Medidas del robot.
*/
QPolygonF getRobotPolygon()
{
//        qDebug()<<"Navigation - "<< __FUNCTION__;

    QPolygonF robotP;


    auto bLWorld = innerModel->transform ("world", robotBottomLeft,  QString::fromStdString(RobotName));
    auto bRWorld = innerModel->transform ("world", robotBottomRight, QString::fromStdString(RobotName));
    auto tRWorld = innerModel->transform ("world", robotTopRight,    QString::fromStdString(RobotName));
    auto tLWorld = innerModel->transform ("world", robotTopLeft,     QString::fromStdString(RobotName));


    robotP << QPointF(bLWorld.x(),bLWorld.z());
    robotP << QPointF(bRWorld.x(),bRWorld.z());
    robotP << QPointF(tRWorld.x(),tRWorld.z());
    robotP << QPointF(tLWorld.x(),tLWorld.z());

    FILE *fd = fopen("robot.txt", "w");
    for (const auto &r: robotP)
    {
        fprintf(fd, "%d %d\n", (int)r.x(), (int)r.y());
    }

    fprintf(fd, "%d %d\n", (int)robotP[0].x(), (int)robotP[0].y());

    fclose(fd);


    return robotP;
}

void updateLaserPolygon(const RoboCompLaser::TLaserData &lData)
{
//        qDebug()<<"Navigation - "<< __FUNCTION__;

    laser_poly.clear(); //stores the points of the laser in lasers refrence system
    laser_cart.clear();
    auto lasernode = innerModel->getNode<InnerModelLaser>(LaserName);

    for (const auto &l : lData)
    {
        //convert laser polar coordinates to cartesian
        QVec laserc = lasernode->laserTo(LaserName,l.dist, l.angle);
        laser_poly << QPointF(laserc.x(),laserc.z());
        laser_cart.push_back(QPointF(laserc.x(),laserc.z()));
    }

    FILE *fd = fopen("laserPoly.txt", "w");
    for (const auto &lp : laser_poly)
    {
        QVec p = innerModel->transform("world",QVec::vec3(lp.x(),0,lp.y()),QString::fromStdString(LaserName));
        fprintf(fd, "%d %d\n", (int)p.x(), (int)p.z());
    }
    fclose(fd);

}

QPointF getRobotNose()
{
//        qDebug()<<"Navigation - "<< __FUNCTION__;
    auto robot = QPointF(currentRobotPose.x(),currentRobotPose.z());

    return (robot + QPointF( (robotZLong/2 + 200) * sin(currentRobotPose.ry()), (robotZLong/2 + 200) * cos(currentRobotPose.ry())));

}
 


};


#endif //PROJECT_NAVIGATION_H
