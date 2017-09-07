/*
 * File:   EntityReader.h
 * Author: gsarthou
 *
 * Created on 18/08/2017
 */

#ifndef ENTITYREADER_H
#define	ENTITYREADER_H

#include <ros/ros.h>

template <typename T>
class EntityReader {
public:
    std::map<std::string, T*> lastConfig_;

    EntityReader(bool fullConfig) {fullConfig_ = fullConfig; }
    ~EntityReader() {}

    bool isPresent(std::string id);

    void clear() { lastConfig_.clear(); }

    ros::Subscriber sub_;
    bool fullConfig_;
};

template <typename T>
bool EntityReader<T>::isPresent(std::string id) {
    timeval curTime;
    gettimeofday(&curTime, NULL);
    unsigned long now = curTime.tv_sec * pow(10, 9) + curTime.tv_usec;
    unsigned long timeThreshold = pow(10, 9);
    //std::cout << "current time: " << now <<  "  human time: " << m_LastTime << std::endl;
    long timeDif = lastConfig_[id]->getTime() - now;
    //std::cout << "time dif: " << timeDif << std::endl;

    if (fabs(timeDif) < timeThreshold)
        return true;
    else
        return false;
}

#endif	/* ENTITYREADER_H */
