/* 
 * File:   GenomPoster.h
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 *
 * Created on December 3, 2014, 6:19 PM
 */

#ifndef GENOMPOSTER_H
#define	GENOMPOSTER_H

#include <portLib.h>
#include <posterLib.h>
 #include <string>

class GenomPoster
{
    
public:
    GenomPoster(std::string name, char* posterStruct,
                int posterSize, unsigned long rate);

    virtual void update();

    //! Find the poster given a posterName
    //! returns true if OK and fill posterId
    bool findPoster();

    bool getUpdatedStatus(){return updatedStatus_;}    
    void setUpdatedStatust(bool status){updatedStatus_ = status;}
    bool isPosterFound(){return posterFound_;}
    bool getPosterStuct(char *posterStruct);


protected:
    std::string posterName_;
    POSTER_ID posterID_;
    /* _updating status is automatically updated */
    bool updatedStatus_;
    bool posterFound_;
    char* posterStruct_;
    int posterSize_;
};

#endif	/* GENOMPOSTER_H */

