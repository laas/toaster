//This file will compute the spatial facts concerning agents present in the interaction.

#include "area_manager/PDGHumanReader.h"
#include "area_manager/PDGRobotReader.h"
#include "area_manager/PDGObjectReader.h"
#include "area_manager/AddArea.h"
#include "area_manager/RemoveArea.h"
#include "area_manager/PrintArea.h"
#include "area_manager/PrintAreas.h"
#include "area_manager/GetRelativePosition.h"
#include "area_manager/Area.h"
#include "toaster-lib/CircleArea.h"
#include "toaster-lib/PolygonArea.h"
#include "toaster-lib/MathFunctions.h"
#include "toaster-lib/Object.h"
#include <pdg/Fact.h>
#include <pdg/FactList.h>
#include <iterator>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>


namespace trans = bg::strategy::transform;
typedef bg::model::point<double, 2, bg::cs::cartesian> point_type;
namespace bn = boost::numeric;

// Vector of Area
// It should be possible to add an area on the fly with a ros service.
std::map<unsigned int, Area*> mapArea_;
std::map<unsigned int, Entity*> mapEntities_;


// TODO: verify the math for the offset!

// Entity should be a vector or a map with all entities
// This function update all the area that depends on an entity position.

void updateEntityArea(std::map<unsigned int, Area*>& mpArea, Entity* entity) {
    for (std::map<unsigned int, Area*>::iterator it = mpArea.begin(); it != mpArea.end(); ++it) {
        if (it->second->getMyOwner() == entity->getId()) {

            // Owner 2D frame
            trans::translate_transformer<double, 2, 2> translate(entity->getPosition().get<0>(), entity->getPosition().get<1>());
            trans::rotate_transformer<bg::radian, double, 2, 2> rotate(entity->getOrientation()[2]);

            // Rotate, then translate
            trans::ublas_transformer<double, 2, 2> rotateTranslate(bn::ublas::prod(translate.matrix(), rotate.matrix()));

            if (it->second->getIsCircle()) {

                point_type newCenter;
                rotateTranslate.apply(((CircleArea*) it->second)->getOffsetFromOwner(), newCenter);
                ((CircleArea*) it->second)->setCenter(newCenter);

            } else {

                bg::model::polygon<bg::model::d2::point_xy<double> > newPoly;
                //rotateTranslate.apply(((PolygonArea*) it->second)->getPolyRelative(), newPoly);
                bg::transform(((PolygonArea*) it->second)->getPolyRelative(), newPoly, rotateTranslate);
                ((PolygonArea*) it->second)->poly_ = newPoly;

            }
        }
    }
}

void updateInArea(Entity* ent, std::map<unsigned int, Area*>& mpArea) {
    for (std::map<unsigned int, Area*>::iterator it = mpArea.begin(); it != mpArea.end(); ++it) {
        // If we already know that entity is in Area, we update if needed.
        if (ent->isInArea(it->second->getId()))
            if (it->second->isPointInArea(MathFunctions::convert3dTo2d(ent->getPosition())))
                continue;
            else {
                printf("[area_manager] %s leaves Area %s\n", ent->getName().c_str(), it->second->getName().c_str());
                ent->removeInArea(it->second->getId());
                it->second->removeEntity(ent->getId());
                if (it->second->getIsRoom())
                    ent->setRoomId(0);
            }// Same if entity is not in Area
        else
            if (it->second->isPointInArea(MathFunctions::convert3dTo2d(ent->getPosition()))) {
            printf("[area_manager] %s enters in Area %s\n", ent->getName().c_str(), it->second->getName().c_str());
            ent->inArea_.push_back(it->second->getId());
            it->second->insideEntities_.push_back(ent->getId());

            //User has to be in a room. May it be a "global room".
            if (it->second->getIsRoom())
                ent->setRoomId(it->second->getId());
        } else {
            //printf("[area_manager][DEGUG] %s is not in Area %s, he is in %f, %f\n", ent->getName().c_str(), it->second->getName().c_str(), ent->getPosition().get<0>(), ent->getPosition().get<1>());
            continue;
        }
    }
}

// Return confidence: 0.0 if not facing 1.0 if facing

double isFacing(Entity* entFacing, Entity* entSubject, double angleThreshold, double& angleResult) {
    return MathFunctions::isInAngle(entFacing, entSubject, entFacing->getOrientation()[2], angleThreshold, angleResult);
}

bool areaCompatible(std::string areaEntType, EntityType entType) {
    if (entType == ROBOT) {
        if (areaEntType == "robots" || areaEntType == "agents" || areaEntType == "entities")
            return true;
        else
            return false;

    } else if (entType == HUMAN) {
        if (areaEntType == "humans" || areaEntType == "agents" || areaEntType == "entities")
            return true;
        else
            return false;

    } else if (entType == OBJECT) {
        if (areaEntType == "objects" || areaEntType == "entities")
            return true;
        else
            return false;
    }
}

void printMyArea(unsigned int id) {
    if (mapArea_[id]->getIsCircle())
        printf("Area name: %s, id: %d, owner id: %d, isRoom %d, factType: %s \n"
            "entityType: %s, isCircle: true, center: %f, %f, ray: %f\n", mapArea_[id]->getName().c_str(),
            id, mapArea_[id]->getMyOwner(), mapArea_[id]->getIsRoom(), mapArea_[id]->getFactType().c_str(), mapArea_[id]->getEntityType().c_str(),
            ((CircleArea*) mapArea_[id])->getCenter().get<0>(), ((CircleArea*) mapArea_[id])->getCenter().get<1>(), ((CircleArea*) mapArea_[id])->getRay());
    else
        printf("Area name: %s, id: %d, owner id: %d, isRoom %d, factType: %s \n"
            "entityType: %s, isCircle: false\n", mapArea_[id]->getName().c_str(),
            id, mapArea_[id]->getMyOwner(), mapArea_[id]->getIsRoom(), mapArea_[id]->getFactType().c_str(), mapArea_[id]->getEntityType().c_str());

    printf("inside entities: ");
    for (int i = 0; i < mapArea_[id]->insideEntities_.size(); i++)
        printf(" %d,", mapArea_[id]->insideEntities_[i]);

    printf("\n--------------------------\n");
}




///////////////////////////
//   Service functions   //
///////////////////////////

bool addArea(area_manager::AddArea::Request &req,
        area_manager::AddArea::Response & res) {

    Area* curArea;

    //If it is a circle area
    if (req.myArea.isCircle) {
        bg::model::point<double, 2, bg::cs::cartesian> center(req.myArea.center.x, req.myArea.center.y);
        bg::model::point<double, 2, bg::cs::cartesian> offset(req.myArea.offsetFromOwner.x, req.myArea.offsetFromOwner.y);

        CircleArea* myCircle = new CircleArea(req.myArea.id, center, req.myArea.ray);
        myCircle->setOffsetFromOwner(offset);
        curArea = myCircle;
    } else {
        //If it is a polygon
        double pointsPoly[req.myArea.poly.points.size()][2];
        for (int i = 0; i < req.myArea.poly.points.size(); i++) {
            pointsPoly[i][0] = req.myArea.poly.points[i].x;
            pointsPoly[i][1] = req.myArea.poly.points[i].y;
        }

        PolygonArea* myPoly = new PolygonArea(req.myArea.id, pointsPoly, req.myArea.poly.points.size());
        curArea = myPoly;
    }

    curArea->setIsCircle(req.myArea.isCircle);
    curArea->setEntityType(req.myArea.entityType);
    curArea->setFactType(req.myArea.factType);
    curArea->setMyOwner(req.myArea.myOwner);
    curArea->setName(req.myArea.name);
    curArea->setIsRoom(req.myArea.isRoom);

    mapArea_[curArea->getId()] = curArea;

    res.answer = true;
    ROS_INFO("request: added Area: id %d, name %s", req.myArea.id, req.myArea.name.c_str());
    ROS_INFO("sending back response: [%d]", (int) res.answer);
    return true;
}

bool removeArea(area_manager::RemoveArea::Request &req,
        area_manager::RemoveArea::Response & res) {

    ROS_INFO("request: removed Area: id %d, named: %s", req.id, mapArea_[req.id]->getName().c_str());
    mapArea_.erase(req.id);

    res.answer = true;
    ROS_INFO("sending back response: [%d]", (int) res.answer);
    return true;
}

bool printArea(area_manager::PrintArea::Request &req,
        area_manager::RemoveArea::Response & res) {

    printMyArea(req.id);
    res.answer = true;
    return true;
}

bool printAreas(area_manager::PrintAreas::Request &req,
        area_manager::PrintAreas::Response & res) {
    for (std::map<unsigned int, Area*>::iterator itArea = mapArea_.begin(); itArea != mapArea_.end(); ++itArea)
        printMyArea(itArea->first);
    return true;
}

bool getRelativePosition(area_manager::GetRelativePosition::Request &req,
        area_manager::GetRelativePosition::Response & res) {
    double pi = 3.1416;

    if (mapEntities_[req.subjectId] != NULL && mapEntities_[req.targetId] != NULL) {
        double angleResult;
        angleResult = MathFunctions::relativeAngle(mapEntities_[req.subjectId], mapEntities_[req.targetId], mapEntities_[req.subjectId]->getOrientation()[2]);
        if (angleResult > 0) {
            if (angleResult < pi / 6)
                res.direction = "ahead";
            else if (angleResult < pi / 4)
                res.direction = "ahead right";
            else if (angleResult < 3 * pi / 4)
                res.direction = "right";
            else if (angleResult < 5 * pi / 6)
                res.direction = "back right";
            else
                res.direction = "back";
        } else {
            if (-angleResult < pi / 6)
                res.direction = "ahead";
            else if (-angleResult < pi / 4)
                res.direction = "ahead left";
            else if (-angleResult < 3 * pi / 4)
                res.direction = "left";
            else if (-angleResult < 5 * pi / 6)
                res.direction = "back left";
            else
                res.direction = "back";
        }

        res.answer = true;
        return true;
    } else
        ROS_INFO("Requested entity is not in the list.");
    res.answer = false;
    return false;
}

int main(int argc, char** argv) {
    // Set this in a ros service
    const bool AGENT_FULL_CONFIG = false; //If false we will use only position and orientation

    ros::init(argc, argv, "area_manager");
    ros::NodeHandle node;

    //Data reading
    PDGHumanReader humanRd(node, AGENT_FULL_CONFIG);
    PDGRobotReader robotRd(node, AGENT_FULL_CONFIG);
    PDGObjectReader objectRd(node);

    //Services
    ros::ServiceServer serviceAdd = node.advertiseService("area_manager/add_area", addArea);
    ROS_INFO("Ready to add Area.");

    ros::ServiceServer serviceRemove = node.advertiseService("area_manager/remove_area", removeArea);
    ROS_INFO("Ready to remove Area.");

    ros::ServiceServer servicePrint = node.advertiseService("area_manager/print_area", printArea);
    ROS_INFO("Ready to print Area.");

    ros::ServiceServer servicePrints = node.advertiseService("area_manager/print_areas", printAreas);
    ROS_INFO("Ready to print Areas.");

    ros::ServiceServer serviceRelativePose = node.advertiseService("area_manager/get_relative_position", getRelativePosition);
    ROS_INFO("Ready to print get relative position.");

    // Publishing
    ros::Publisher fact_pub = node.advertise<pdg::FactList>("area_manager/factList", 1000);

    pdg::FactList factList_msg;
    pdg::Fact fact_msg;

    // Set this in a ros service?
    ros::Rate loop_rate(30);



    /************************/
    /* Start of the Ros loop*/
    /************************/

    //TODO: remove human / robot id and do it for all
    while (node.ok()) {

        ////////////////////////////////
        // Updating situational Areas //
        ////////////////////////////////

        //TODO: for more optimal computing, go through area and if it is a circle, update area.
        // for each area
        //   if current area is circle
        //     get area owner
        //     update area with owner position

        // Humans
        for (std::map<unsigned int, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
            // We update area with human center
            mapEntities_[it->first] = it->second;
            updateEntityArea(mapArea_, it->second);
        }

        // Robots
        for (std::map<unsigned int, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {
            // We update area with robot center
            mapEntities_[it->first] = it->second;
            updateEntityArea(mapArea_, it->second);
        }

        // Objects
        for (std::map<unsigned int, Object*>::const_iterator it = objectRd.lastConfig_.begin(); it != objectRd.lastConfig_.end(); ++it) {
            // We update area with object center
            mapEntities_[it->first] = it->second;
            updateEntityArea(mapArea_, it->second);
        }


        /////////////////////////////////
        // Updating in Area properties //
        /////////////////////////////////


        for (std::map<unsigned int, Entity*>::iterator it = mapEntities_.begin(); it != mapEntities_.end(); ++it) {
            // We update area with human center
            updateInArea(it->second, mapArea_);
        }


        ///////////////////////////////////////
        // Computing facts for each entities //
        ///////////////////////////////////////

        // TODO: replace code by a function for each fact computation


        for (std::map<unsigned int, Entity*>::iterator itEntity = mapEntities_.begin(); itEntity != mapEntities_.end(); ++itEntity) {
            for (std::map<unsigned int, Area*>::iterator itArea = mapArea_.begin(); itArea != mapArea_.end(); ++itArea) {

                if (areaCompatible(itArea->second->getEntityType(), itEntity->second->getEntityType())) {

                    Entity* ownerEnt;

                    // Let's find back the area owner:
                    if (robotRd.lastConfig_[itArea->second->getMyOwner()] != NULL)
                        ownerEnt = robotRd.lastConfig_[itArea->second->getMyOwner()];
                    else if (humanRd.lastConfig_[itArea->second->getMyOwner()] != NULL)
                        ownerEnt = humanRd.lastConfig_[itArea->second->getMyOwner()];
                    else if (objectRd.lastConfig_[itArea->second->getMyOwner()] != NULL)
                        ownerEnt = objectRd.lastConfig_[itArea->second->getMyOwner()];

                    // compute facts according to factType
                    // TODO: instead of calling it interaction, make a list of facts to compute?
                    if (itArea->second->getFactType() == "interaction") {

                        if (ownerEnt != NULL) {

                            // Now let's compute isFacing
                            //////////////////////////////

                            double confidence = 0.0;
                            // This is the actual angle between subject orientation
                            // and target. It gives left / right relation
                            // If positive, target is at right!
                            double angleResult = 0.0;
                            confidence = isFacing(itEntity->second, ownerEnt, 0.5, angleResult);
                            if (confidence > 0.0) {
                                printf("[area_manager][DEBUG] %s is facing %s with confidence %f, angleResult %f\n",
                                        itEntity->second->getName().c_str(), ownerEnt->getName().c_str(), confidence, angleResult);

                                //Fact Facing
                                fact_msg.property = "IsFacing";
                                fact_msg.propertyType = "posture";
                                fact_msg.subProperty = "orientationAngle";
                                fact_msg.subjectId = itEntity->first;
                                fact_msg.subjectName = itEntity->second->getName();
                                fact_msg.targetName = ownerEnt->getName();
                                fact_msg.targetId = ownerEnt->getId();
                                fact_msg.confidence = confidence;
                                fact_msg.doubleValue = angleResult;
                                fact_msg.valueType = 1;
                                fact_msg.factObservability = 0.5;
                                fact_msg.time = itEntity->second->getTime();

                                factList_msg.factList.push_back(fact_msg);
                            }


                            // Compute here other facts linked to interaction
                            //////////////////////////////////////////////////

                            // TODO


                        } // ownerEnt!= NULL

                    } else if (itArea->second->getFactType() == "") {

                    } else {
                        printf("[area_manager][WARNING] Area %s has factType %s, which is not available\n", itArea->second->getName().c_str(), itArea->second->getFactType().c_str());
                    }

                    //Fact in Area
                    fact_msg.property = "IsInArea";
                    fact_msg.propertyType = "position";

                    if (itArea->second->getIsRoom())
                        fact_msg.subProperty = "room";
                    else
                        fact_msg.subProperty = ownerEnt->getName();

                    fact_msg.subjectId = itEntity->first;
                    fact_msg.subjectName = itEntity->second->getName();
                    fact_msg.targetName = itArea->second->getName();
                    fact_msg.targetId = itArea->second->getId();
                    fact_msg.confidence = 1;
                    fact_msg.factObservability = 0.8;
                    fact_msg.time = itEntity->second->getTime();

                    factList_msg.factList.push_back(fact_msg);

                }
            }
        }


        fact_pub.publish(factList_msg);

        ros::spinOnce();

        factList_msg.factList.clear();

        loop_rate.sleep();
    }
    return 0;
}
