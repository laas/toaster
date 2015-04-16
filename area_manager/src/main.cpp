//This file will compute the spatial facts concerning agents present in the interaction.

#include "area_manager/PDGHumanReader.h"
#include "area_manager/PDGRobotReader.h"
#include "area_manager/PDGObjectReader.h"
#include "area_manager/AddArea.h"
#include "area_manager/RemoveArea.h"
#include "area_manager/PrintArea.h"
#include "area_manager/PrintAreas.h"
#include "area_manager/Area.h"
#include "toaster-lib/CircleArea.h"
#include "toaster-lib/PolygonArea.h"
#include "toaster-lib/MathFunctions.h"
#include "toaster-lib/Object.h"
#include <pdg/Fact.h>
#include <pdg/FactList.h>
#include <iterator>

// Entity should be a vector or a map with all entities
// This function update all the area that depends on an entity position.



// Vector of Area
// It should be possible to add an area on the fly with a ros service.
std::map<unsigned int, Area*> mapArea_;




void updateEntityArea(std::map<unsigned int, Area*>& mpArea, Entity* entity) {
    for (std::map<unsigned int, Area*>::iterator it = mpArea.begin(); it != mpArea.end(); ++it) {
        if (it->second->getMyOwner() == entity->getId() && it->second->getIsCircle())
            ((CircleArea*) it->second)->setCenter(MathFunctions::convert3dTo2d(entity->getPosition()));
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

double isFacing(Entity* entFacing, Entity* entSubject, double angleThreshold) {
    return MathFunctions::isInAngle(entFacing, entSubject, entFacing->getOrientation()[2], angleThreshold);
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

        CircleArea* myCircle = new CircleArea(req.myArea.id, center, req.myArea.ray);
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

        // Humans
        for (std::map<unsigned int, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
            // We update area with human center
            updateEntityArea(mapArea_, it->second);
        }

        // Robots
        for (std::map<unsigned int, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {
            // We update area with robot center
            updateEntityArea(mapArea_, it->second);
        }

        // Objects
        for (std::map<unsigned int, Object*>::const_iterator it = objectRd.lastConfig_.begin(); it != objectRd.lastConfig_.end(); ++it) {
            // We update area with object center
            updateEntityArea(mapArea_, it->second);
        }


        /////////////////////////////////
        // Updating in Area properties //
        /////////////////////////////////


        // Humans
        for (std::map<unsigned int, Human*>::iterator it = humanRd.lastConfig_.begin(); it != humanRd.lastConfig_.end(); ++it) {
            // We update area with human center
            updateInArea(it->second, mapArea_);
        }

        // Robots
        for (std::map<unsigned int, Robot*>::iterator it = robotRd.lastConfig_.begin(); it != robotRd.lastConfig_.end(); ++it) {
            // We update area with robot center
            updateInArea(it->second, mapArea_);
        }

        // Objects
        for (std::map<unsigned int, Object*>::const_iterator it = objectRd.lastConfig_.begin(); it != objectRd.lastConfig_.end(); ++it) {
            // We update area with object center
            updateInArea(it->second, mapArea_);
        }



        ///////////////////////////////////////
        // Computing facts for each entities //
        ///////////////////////////////////////



        // Humans
        for (std::map<unsigned int, Human*>::iterator itHuman = humanRd.lastConfig_.begin(); itHuman != humanRd.lastConfig_.end(); ++itHuman) {
            for (std::map<unsigned int, Area*>::iterator itArea = mapArea_.begin(); itArea != mapArea_.end(); ++itArea) {

                if (itArea->second->getEntityType() == "entities" || itArea->second->getEntityType() == "agents" || itArea->second->getEntityType() == "humans") {

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
                            confidence = isFacing(itHuman->second, ownerEnt, 0.5);
                            if (confidence > 0.0) {
                                printf("[area_manager][DEGUG] %s is facing %s with confidence %f\n",
                                        itHuman->second->getName().c_str(), ownerEnt->getName().c_str(), confidence);

                                //Fact Facing
                                fact_msg.property = "IsFacing";
                                fact_msg.propertyType = "posture";
                                fact_msg.subProperty = "orientationAngle";
                                fact_msg.subjectId = itHuman->first;
                                fact_msg.subjectName = itHuman->second->getName();
                                fact_msg.targetName = ownerEnt->getName();
                                fact_msg.targetId = ownerEnt->getId();
                                fact_msg.confidence = confidence;
                                fact_msg.factObservability = 0.5;
                                fact_msg.time = itHuman->second->getTime();

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

                    fact_msg.subjectId = itHuman->first;
                    fact_msg.subjectName = itHuman->second->getName();
                    fact_msg.targetName = itArea->second->getName();
                    fact_msg.targetId = itArea->second->getId();
                    fact_msg.confidence = 1;
                    fact_msg.factObservability = 0.8;
                    fact_msg.time = itHuman->second->getTime();

                    factList_msg.factList.push_back(fact_msg);

                }
            }
        }

        // Robots
        for (std::map<unsigned int, Robot*>::iterator itRobot = robotRd.lastConfig_.begin(); itRobot != robotRd.lastConfig_.end(); ++itRobot) {
            for (std::map<unsigned int, Area*>::iterator itArea = mapArea_.begin(); itArea != mapArea_.end(); ++itArea) {

                if (itArea->second->getEntityType() == "entities" || itArea->second->getEntityType() == "agents" || itArea->second->getEntityType() == "robots") {
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
                            confidence = isFacing(itRobot->second, ownerEnt, 0.5);
                            if (confidence > 0.0) {
                                printf("[area_manager][DEGUG] %s is facing %s with confidence %f\n",
                                        itRobot->second->getName().c_str(), ownerEnt->getName().c_str(), confidence);

                                //Fact Facing
                                fact_msg.property = "IsFacing";
                                fact_msg.propertyType = "posture";
                                fact_msg.subProperty = "orientationAngle";
                                fact_msg.subjectId = itRobot->first;
                                fact_msg.subjectName = itRobot->second->getName();
                                fact_msg.targetName = ownerEnt->getName();
                                fact_msg.targetId = ownerEnt->getId();
                                fact_msg.confidence = confidence;
                                fact_msg.factObservability = 0.5;
                                fact_msg.time = itRobot->second->getTime();

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

                    fact_msg.subjectId = itRobot->first;
                    fact_msg.subjectName = itRobot->second->getName();
                    fact_msg.targetName = itArea->second->getName();
                    fact_msg.targetId = itArea->second->getId();
                    fact_msg.confidence = 1;
                    fact_msg.factObservability = 0.8;
                    fact_msg.time = itRobot->second->getTime();

                    factList_msg.factList.push_back(fact_msg);

                }
            }
        }


        // Objects
        for (std::map<unsigned int, Object*>::iterator itObject = objectRd.lastConfig_.begin(); itObject != objectRd.lastConfig_.end(); ++itObject) {
            for (std::map<unsigned int, Area*>::iterator itArea = mapArea_.begin(); itArea != mapArea_.end(); ++itArea) {

                if (itArea->second->getEntityType() == "entities" || itArea->second->getEntityType() == "objects") {

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
                            confidence = isFacing(itObject->second, ownerEnt, 0.5);
                            if (confidence > 0.0) {
                                printf("[area_manager][DEGUG] %s is facing %s with confidence %f\n",
                                        itObject->second->getName().c_str(), ownerEnt->getName().c_str(), confidence);

                                //Fact Facing
                                fact_msg.property = "IsFacing";
                                fact_msg.propertyType = "posture";
                                fact_msg.subProperty = "orientationAngle";
                                fact_msg.subjectId = itObject->first;
                                fact_msg.subjectName = itObject->second->getName();
                                fact_msg.targetName = ownerEnt->getName();
                                fact_msg.targetId = ownerEnt->getId();
                                fact_msg.confidence = confidence;
                                fact_msg.factObservability = 0.5;
                                fact_msg.time = itObject->second->getTime();

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

                    fact_msg.subjectId = itObject->first;
                    fact_msg.subjectName = itObject->second->getName();
                    fact_msg.targetName = itArea->second->getName();
                    fact_msg.targetId = itArea->second->getId();
                    fact_msg.confidence = 1;
                    fact_msg.factObservability = 0.8;
                    fact_msg.time = itObject->second->getTime();

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
