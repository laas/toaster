#include "pdg/utility/XmlUtility.h"

#include "ros/ros.h"

// Parse XML
#include <tinyxml.h>
#include "ros/package.h"

using namespace std;

vector<string> loadPropertiesFromXml(string objectID) {
    vector<string> ret;

    std::stringstream pathIoTObj;
    TiXmlDocument listIoTObj;
    pathIoTObj << ros::package::getPath("pdg") << "/params/iot_objects.xml";
    listIoTObj = TiXmlDocument(pathIoTObj.str());

    if (!listIoTObj.LoadFile()) {
        ROS_WARN_ONCE("Error while loading xml file");
        ROS_WARN_ONCE("error #%d: %s", listIoTObj.ErrorId(), listIoTObj.ErrorDesc());
    }

    TiXmlHandle hdl(&listIoTObj);
    TiXmlElement *object_elem = hdl.FirstChild("objects").FirstChild("object").Element();

    std::string id;
    while (object_elem) { //for each element object
        id = object_elem->Attribute("id");
        if (id == objectID) {
            TiXmlHandle hdl(object_elem);
            TiXmlElement *fact_elem = hdl.FirstChild("fact").Element();

            string fact_name;

            while (fact_elem) { // for each fact
                fact_name = fact_elem->Attribute("name");
                // we add the fact name to the returned vector
                ret.push_back(fact_name);
                fact_elem = fact_elem->NextSiblingElement();
            }
        }
        object_elem = object_elem->NextSiblingElement();
    }
    return ret;
}

string loadValueFromXmlAsString(string objectID, string factName, string objectValueName, string objectValue) {
    ROS_INFO("loadValueFromXmlAsString objectID:%s factName:%s objectValueName:%s objectValue:%s", objectID.c_str(), factName.c_str(), objectValueName.c_str(), objectValue.c_str());
    std::string ret;

    std::stringstream pathIoTObj;
    TiXmlDocument listIoTObj;
    pathIoTObj << ros::package::getPath("pdg") << "/params/iot_objects.xml";
    listIoTObj = TiXmlDocument(pathIoTObj.str());

    if (!listIoTObj.LoadFile()) {
        ROS_WARN_ONCE("Error while loading xml file");
        ROS_WARN_ONCE("error #%d: %s", listIoTObj.ErrorId(), listIoTObj.ErrorDesc());
    }

    TiXmlHandle hdl(&listIoTObj);
    TiXmlElement *object_elem = hdl.FirstChild("objects").FirstChild("object").Element();

    std::string id;

    while (object_elem) { //for each element object
        id = object_elem->Attribute("id");
        if (id == objectID) {
            TiXmlHandle hdl(object_elem);
            TiXmlElement *fact_elem = hdl.FirstChild("fact").Element();

            std::string fact_name;

            while (fact_elem) { // for each fact
                fact_name = fact_elem->Attribute("name");
                if (fact_name == factName) {
                    TiXmlHandle hdl(fact_elem);
                    TiXmlElement *value_elem = hdl.FirstChild("value").Element();

                    std::string object_value_name;
                    std::string operation;
                    std::string object_value;
                    std::string fact_value;
                    while (value_elem) { // for each value
                        object_value_name = value_elem->Attribute("object_value_name");
                        operation = value_elem->Attribute("operation");
                        if (value_elem->Attribute("object_value")) {
                            object_value = value_elem->Attribute("object_value");
                        }
                        if (value_elem->Attribute("fact_value")) {
                            fact_value = value_elem->Attribute("fact_value");
                        }
                        if (objectValueName == object_value_name) {
                            if (operation == "copy") {
                                ret = objectValue;
                            }
                            else if (operation == "equals") {
                                if (object_value == objectValue) {
                                    ret = fact_value;
                                }
                            }
                        }
                        value_elem = value_elem->NextSiblingElement();
                    }
                }
                fact_elem = fact_elem->NextSiblingElement();
            }
        }
        object_elem = object_elem->NextSiblingElement();
    }
    return ret;
}
