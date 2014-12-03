/* 
 * File:   GenomPoster.cpp
 * Author: Grégoire Milliez
 * mail: gregoire.milliez@laas.fr
 *  Copyright 2014 LAAS/CNRS. All rights reserved.
 * 
 * Created on December 3, 2014, 6:23 PM
 */

#include "middleware/GenomPoster.h"

#include <iostream>
//#include <unistd.h>

#include <stdio.h>

GenomPoster::GenomPoster(std::string name, char* posterStruct, int posterSize, unsigned long rate)
{
    posterName_ = name;
    posterStruct_ = posterStruct;
    posterSize_ = posterSize;
    posterID_ = NULL;
    updatedStatus_ = false;
}


void GenomPoster::update() {
    if(posterID_ == NULL)
    {
        if(findPoster() == false)
        {
            updatedStatus_ = false;
        }
    } else {
      //printf("read poster %p \n", posterID_);
            int size = posterRead(posterID_, 0, posterStruct_, posterSize_);
 
            if( size != posterSize_)
            {
                updatedStatus_ = false;
                cout << "ERROR: GenomPoster::refresh() poster " << posterName_ << " size mismatch (" << size << " , " << posterSize_ << " ) for posterID " << posterID_ << endl;
            }
            //cout << "GenomPoster::refresh() poster " << posterName_ << endl;
            updatedStatus_ = true;
        };
    return;
}

bool GenomPoster::getPosterStuct(char *posterStruct) {
    posterStruct = posterStruct_;
    return updatedStatus_;
}

bool GenomPoster::findPoster()
{
    if(posterFind(posterName_.c_str(), &(posterID_))==ERROR)
    {
        cout << "ERROR: GenomPoster::findPoster can't find poster \""<< posterName_ << "\", retry within 2 sec. " << endl;
        posterID_ = NULL;
        return false;
    }
    cout << "*********************  INFO: GenomPoster::findPoster: Poster " <<  posterName_ << " found ***********************" << endl;
    return true;
}

