/* 
 * File:   SparkFactReader.cpp
 * Author: gmilliez
 * 
 * Created on May 31, 2015, 1:00 PM
 */

#include <vector>

#include "pdg/SparkFactReader.h"

//Constructor

SparkFactReader::SparkFactReader() {
    std::cout << "[PDG] Initializing SparkFactReader" << std::endl;
}

void SparkFactReader::init(std::string posterName) {
    /* declaration of the poster reader threads */
    sparkFactPoster_ = new GenomPoster(posterName, (char*) (&sparkFactPosterStruct_), sizeof (sparkFactPosterStruct_), 10);
    sparkFactPoster_->getPosterStuct((char*) (&sparkFactPosterStruct_));
    nbFacts_ = sparkFactPosterStruct_.nbFact;
}

void SparkFactReader::updateFacts() {
    sparkFactPoster_->update();
    sparkFactPoster_->getPosterStuct((char*) (&sparkFactPosterStruct_));
    unsigned int i = 0; /// iterator on detected object

    // Verify that it was indeed updated
    if (sparkFactPoster_->getUpdatedStatus()) {
        nbFacts_ = sparkFactPosterStruct_.nbFact;
        toaster_msgs::FactList newFactList;
        toaster_msgs::Fact fact_msg;


        for (i = 0; i < nbFacts_; i++) {

            //Set fact
            //Fact moving toward
            fact_msg.property = sparkFactPosterStruct_.factList[i].property.name;
            fact_msg.propertyType = sparkFactPosterStruct_.factList[i].propertyType.name;
            fact_msg.subProperty = sparkFactPosterStruct_.factList[i].subProperty.name;
            fact_msg.subjectId = sparkFactPosterStruct_.factList[i].subjectId;
            fact_msg.subjectName = sparkFactPosterStruct_.factList[i].subjectName.name;
            fact_msg.targetId = sparkFactPosterStruct_.factList[i].targetId;
            fact_msg.targetName = sparkFactPosterStruct_.factList[i].targetName.name;
            fact_msg.ownerName = sparkFactPosterStruct_.factList[i].ownerName.name;
            fact_msg.ownerId = sparkFactPosterStruct_.factList[i].ownerId;
            fact_msg.stringValue = sparkFactPosterStruct_.factList[i].stringValue.name;
            fact_msg.doubleValue = sparkFactPosterStruct_.factList[i].doubleValue;
            fact_msg.valueType = sparkFactPosterStruct_.factList[i].valueType;
            fact_msg.confidence = sparkFactPosterStruct_.factList[i].confidence;
            fact_msg.time = sparkFactPosterStruct_.factList[i].time;

            newFactList.factList.push_back(fact_msg);

        }
        currentFactList_ = newFactList;
    }
}


//Destructor

SparkFactReader::~SparkFactReader() {
}

