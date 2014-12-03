// An object reader is a class that will read data from a middleware message
// and fill a DynamicObject class from toaster-lib accordingly to publish on a ros topic.

#include <ros/ros.h>
#include "toaster-lib/MovableObject.h"
#include <map>

class ObjectReader{

    public:
        std::map<unsigned int, MovableObject*> lastConfig_;
        unsigned int nbObjects_; /// total object number

        virtual void init() = 0;     // This function will depend on the middleware

        bool isPresent(unsigned int id);
};