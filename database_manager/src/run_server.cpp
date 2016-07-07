#include <stdio.h>
#include <sqlite3.h> 
#include <tinyxml.h>
#include <sstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include "toaster_msgs/GetInfoDB.h"
#include "toaster_msgs/SetInfoDB.h"
#include "toaster_msgs/ExecuteDB.h"
#include "toaster_msgs/PlotFactsDB.h"
#include "toaster_msgs/LoadSaveDB.h"
#include "toaster_msgs/Property.h"
#include "toaster_msgs/Agent.h"
#include "toaster_msgs/ToasterFactReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
#include "toaster_msgs/Event.h"
#include "toaster_msgs/Ontology.h"
#include "toaster_msgs/Id.h"
#include "toaster_msgs/FactList.h"
#include <fstream>



std::vector<std::string> agentList;

std::vector<toaster_msgs::Fact> myFactList;
std::vector<toaster_msgs::Property> myPropertyList;
std::vector<toaster_msgs::Id> myEntityList;
std::vector<toaster_msgs::Event> myEventList;
std::vector<toaster_msgs::Ontology> myOntologyList;

std::vector<std::string> myStringList;
std::vector<toaster_msgs::Fact> previousFactsState;

std::vector<ToasterFactReader*> factsReaders;
ToasterFactReader* readerAgent;
ToasterFactReader* readerArea;
ToasterFactReader* readerMove3d;
ToasterFactReader* readerPdg;

int nb_agents;

//sqlite database's pointer
sqlite3 *database;

std::string mainAgent;



////////////////////////////////////////////////////////////////////////
//////////callback functions//////////////

/**
 * generic callback for debug
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int callback(void *NotUsed, int argc, char **argv, char **azColName) {
    ROS_INFO("---");
    for (int i = 0; i < argc; i++) {
        ROS_INFO("%s = %s", azColName[i], argv[i] ? argv[i] : "NULL"); //if needed to debug
    }
    return 0;
}

/**
 * callback for operations on facts and memory tables 
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int get_facts_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int nb_el = 10;

    //ROS_INFO("---");

    for (int i = 0; i < argc / nb_el; i++) {
        //ROS_INFO("%s = %s", azColName[i*nb_el], argv[i*nb_el] ? argv[i*nb_el] : "NULL"); //needed to debug

        toaster_msgs::Fact f;
        f.subjectId = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
        f.property = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
        f.propertyType = argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL";
        f.targetId = argv[(i) * nb_el + 3] ? argv[(i) * nb_el + 3] : "NULL";
        f.valueType = (bool)(argv[(i) * nb_el + 4] ? argv[(i) * nb_el + 4] : "NULL");
        f.stringValue = argv[(i) * nb_el + 5] ? argv[i * nb_el + 5] : "NULL";
        f.doubleValue = atof(argv[i * nb_el + 6] ? argv[i * nb_el + 6] : "NULL");
        f.factObservability = atof(argv[i * nb_el + 7] ? argv[i * nb_el + 7] : "NULL");
        f.confidence = atof(argv[i * nb_el + 8] ? argv[i * nb_el + 8] : "NULL");
        f.timeStart = atof(argv[i * nb_el + 9] ? argv[i * nb_el + 9] : "NULL");
        f.timeEnd = atof(argv[i * nb_el + 10] ? argv[i * nb_el + 10] : "NULL");

        myFactList.push_back(f);
    }
    //ROS_INFO("---");
    return 0;
}

/**
 * callback for operations on property tables
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int property_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int i;
    int nb_el = 5;

    // ROS_INFO("---");

    for (i = 0; i < argc / nb_el; i++) {
        //ROS_INFO("%s = %s", azColName[i], argv[i] ? argv[i] : "NULL"); //if needed to debug

        toaster_msgs::Property p;
        p.id = atoi(argv[i * nb_el] ? argv[i * nb_el] : "NULL");
        p.color = argv[i * nb_el + 1] ? argv[i * nb_el + 1] : "NULL";
        p.height = argv[i * nb_el + 2] ? argv[i * nb_el + 2] : "NULL";
        p.linkedToId = atoi(argv[i * nb_el + 3] ? argv[i * nb_el + 3] : "NULL");
        p.linkType = argv[i * nb_el + 4] ? argv[i * nb_el + 4] : "NULL";

        myPropertyList.push_back(p);
    }
    //ROS_INFO("---");
    return 0;
}

/**
 * callback for operations on agent table
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int id_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int nb_el = 4;

    // ROS_INFO("---");

    for (int i = 0; i < argc / nb_el; i++) {
        // ROS_INFO("%s = %s", azColName[i], argv[i] ? argv[i] : "NULL"); //if needed to debug

        toaster_msgs::Id id;
        id.id = argv[i * nb_el] ? argv[i * nb_el] : "NULL";
        id.name = argv[i * nb_el] ? argv[i * nb_el] : "NULL";
        id.type = argv[i * nb_el + 2] ? argv[i * nb_el + 2] : "NULL";
        id.owner_id = argv[i * nb_el + 3] ? argv[i * nb_el + 3] : "NULL";

        myEntityList.push_back(id);
    }
    //ROS_INFO("---");
    return 0;
}

/**
 * callback for operations on events table
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int event_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int nb_el = 6;

    ROS_INFO("---");

    for (int i = 0; i < argc / nb_el; i++) {
        ROS_INFO("%s = %s", azColName[i * nb_el], argv[i * nb_el] ? argv[i * nb_el] : "NULL"); //if needed to debug

        toaster_msgs::Event e;
        e.subjectId = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
        e.property = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
        e.propertyType = argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL";
        e.targetId = argv[(i) * nb_el + 3] ? argv[(i) * nb_el + 3] : "NULL";
        e.factObservability = atof(argv[i * nb_el + 4] ? argv[i * nb_el + 4] : "NULL");
        e.confidence = atof(argv[i * nb_el + 5] ? argv[i * nb_el + 5] : "NULL");
        e.time = atof(argv[i * nb_el + 6] ? argv[i * nb_el + 6] : "NULL");

        myEventList.push_back(e);
    }
    ROS_INFO("---");
    return 0;
}

/**
 * callback for operations on ontology table
 * @param *NotUsed 	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int ontology_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    int nb_el = 3;

    //ROS_INFO("---");

    for (int i = 0; i < argc / nb_el; i++) {
        //ROS_INFO("%s = %s", azColName[i*nb_el], argv[i*nb_el] ? argv[i*nb_el] : "NULL"); //if needed to debug

        toaster_msgs::Ontology o;
        o.entityClass = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
        o.individual = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
        o.instantiated = (bool)(argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL");


        myOntologyList.push_back(o);
    }
    //ROS_INFO("---");
    return 0;
}

/**
 * callback for operations with any sql query
 * @param *NotUsed  	pointer(non used)
 * @param argc 	 	number of columns in row
 * @param **argv 		array of strings representing field in the row
 * @param **argv 		array of strings representing columns in the row
 * @return 0
 */
int sql_callback(void *NotUsed, int argc, char **argv, char **azColName) {
    for (int i = 0; i < argc; i++) {
        //ROS_INFO("%s = %s", azColName[i], argv[i] ? argv[i] : "NULL"); //id needed to debug
        myStringList.push_back(argv[i] ? argv[i] : "NULL");
    }
    return 0;
}



///////////////////////////////////////////////////////////////////////
//////xml launch functions/////////

/**
 * Get all information contained in the static property xml file 
 * @return void
 */
void launchStaticPropertyDatabase() {
    char *zErrMsg = 0;
    std::string sql;
    std::stringstream s;

    s << ros::package::getPath("database_manager") << "/database/static_property.xml"; //load xml file
    TiXmlDocument static_property(s.str());
    //TiXmlDocument static_property("src/database_manager/database/static_property.xml"); //load xml file (static way)

    if (!static_property.LoadFile()) {
        ROS_WARN_ONCE("Error while loading xml file for static properties");
        ROS_WARN_ONCE("error # %d", static_property.ErrorId());
        ROS_WARN_ONCE("%s", static_property.ErrorDesc());
        exit(0);
    } else {
        TiXmlHandle hdl(&static_property);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        while (elem) //for each element of the xml file
        {
            sql = (std::string)"INSERT INTO static_property_table (id, color,height,linkedToId,linkType) VALUES ("
                    + elem->Attribute("id") + ","
                    + elem->Attribute("color") + ","
                    + elem->Attribute("height") + ","
                    + elem->Attribute("linkedToId") + ","
                    + elem->Attribute("linkType") + (std::string)");";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_WARN_ONCE("SQL error with xml file: %s", zErrMsg);
                sqlite3_free(zErrMsg);
            }

            elem = elem->NextSiblingElement();
        }
    }
}

/**
 * Get all information about agents and objects contained in the xml file
 * @return void
 */
void launchIdList(std::string IDList) {
    char *zErrMsg = 0;
    std::string sql;
    std::stringstream s;

    s << ros::package::getPath("database_manager") << IDList.c_str(); //load xml file
    TiXmlDocument static_property(s.str());

    if (!static_property.LoadFile()) {
        ROS_WARN_ONCE("Erreur lors du chargement du fichier xml");
        ROS_WARN_ONCE("error # %d", static_property.ErrorId());
        ROS_WARN_ONCE("%s", static_property.ErrorDesc());
        exit(0);
    } else {
        TiXmlHandle hdl(&static_property);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        while (elem) //for each element of the xml file
        {
            std::string testAgent = (std::string)elem->Attribute("type");
            if (!testAgent.compare("'human'") || !testAgent.compare("'robot'")) //for each human or robot
            {
                agentList.push_back((std::string)elem->Attribute("id"));

                //we create one fact table
                sql = (std::string)"CREATE TABLE fact_table_" + (std::string)elem->Attribute("id") + " (" +
                        "subject_id 				 	unsigned long," +
                        "predicate         	            string," +
                        "propertyType                          string," +
                        "target_id        		     	unsigned long," +
                        "valueType			  	        bit," +
                        "valueString       	        	string," +
                        "valueDouble       	        	double," +
                        "observability   		   		unsinged short," +
                        "confidence   		        	unsigned short," +
                        "start   				        int," +
                        "end 					        int," +
                        "unique (subject_id,predicate,propertyType,target_id ,valueString ,observability,confidence ,end) );"; //unique fields are used to avoid doublons

                if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                    ROS_WARN_ONCE("SQL error l511: %s", zErrMsg);
                    sqlite3_free(zErrMsg);
                } else {
                    //ROS_INFO("Opened fact table successfully\n");
                }

                //and one memory table
                sql = (std::string)"CREATE TABLE memory_table_" + (std::string)elem->Attribute("id") + " (" +
                        "subject_id 				 unsigned long," +
                        "predicate         	         string," +
                        "propertyType                          string," +
                        "target_id        		     unsigned long," +
                        "valueType			  	     bit," +
                        "valueString       	         string," +
                        "valueDouble       	         double," +
                        "observability   		     unsinged short," +
                        "confidence   		         unsigned short," +
                        "start   				     int," +
                        "end 					     int );"; //in memory table facts aren't unique

                if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                    ROS_WARN_ONCE("SQL error l531: %s", zErrMsg);
                    sqlite3_free(zErrMsg);
                } else {
                    //ROS_INFO("Opened memory table successfully\n");
                }
            }

            sql = (std::string)"INSERT INTO id_table (id, name,type,owner_id) VALUES ( '"
                    + elem->Attribute("id") + "',"
                    + elem->Attribute("name") + ","
                    + elem->Attribute("type") + ","
                    + elem->Attribute("owner_id") + (std::string)" );";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_WARN_ONCE("SQL error with xml file: %s", zErrMsg);
                sqlite3_free(zErrMsg);
            }
            elem = elem->NextSiblingElement();
        }
    }
}

/**
 * Get all information about ontology contained in the xml file
 * @return void
 */
void launchOntology() {
    char *zErrMsg = 0;
    std::string sql;
    std::stringstream s;

    s << ros::package::getPath("database_manager") << "/database/ontology.xml"; //load xml file
    TiXmlDocument ontology(s.str());
    //TiXmlDocument static_property("src/database_manager/database/static_property.xml"); //load xml file (static way)

    if (!ontology.LoadFile()) {
        ROS_WARN_ONCE("Erreur lors du chargement du fichier xml");
        ROS_WARN_ONCE("error # %d", ontology.ErrorId());
        ROS_WARN_ONCE("%s", ontology.ErrorDesc());
        exit(0);
    } else {
        TiXmlHandle hdl(&ontology);
        TiXmlElement *elem = hdl.FirstChildElement().FirstChildElement().Element();

        while (elem) //for each element of the xml file
        {
            sql = (std::string)"INSERT INTO ontology_table (entityClass, individual, instantiated) VALUES ("
                    + elem->Attribute("entityClass") + ","
                    + elem->Attribute("individual") + ","
                    + elem->Attribute("instantiated") + (std::string)");";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_WARN_ONCE("SQL error with xml file: %s", zErrMsg);
                sqlite3_free(zErrMsg);
            }

            elem = elem->NextSiblingElement();
        }
    }
}


///////////////////////////////////////////////////////////////
////////basic functions/////

/**
 * Add an agent to the agent table	
 */
bool add_entity_db(std::string id, std::string ownerId, std::string name, std::string type) {
    //ROS_INFO("add_entity");

    std::string sql;
    std::string sql2;
    char *zErrMsg = 0;

    //add the agent in agents_table
    sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('" + id + "', '" + name + "' ,'" + type
            + "' ,'" + ownerId + "');";

    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        //ROS_INFO("SQL error1: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    }

    //now if this entity need facts table we create it
    zErrMsg = 0;

    if (type == "human" || type == "robot") {
        ROS_WARN("%d", type.compare("human"));
        sql = (std::string)"CREATE TABLE fact_table_" + id + " (" +
                "subject_id 				 unsigned long," +
                "predicate         	            string," +
                "propertyType                          string," +
                "target_id        		     unsigned long," +
                "valueType			  	        	bit," +
                "valueString       	        	string," +
                "valueDouble       	        	double," +
                "observability   		    unsinged short," +
                "confidence   		        unsigned short," +
                "start   				               int," +
                "end 					               int," +
                "unique (subject_id,predicate,propertyType,target_id ,valueString ,observability,confidence ,end) );";


        //and his memory table

        sql2 = (std::string)"CREATE TABLE memory_table_" + id + " (" +
                "subject_id 				 unsigned long," +
                "predicate         	            string," +
                "propertyType                          string," +
                "target_id        		     unsigned long," +
                "valueType			  	        	bit," +
                "valueString       	        	string," +
                "valueDouble       	        	double," +
                "observability   		    unsinged short," +
                "confidence   		        unsigned short," +
                "start   				               int," +
                "end 					               int);";

        //if the table creation is an echec, we remove the fact_table, the memory table and the agent from agents_table		
        if ((sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) || (sqlite3_exec(database, sql2.c_str(), callback, 0, &zErrMsg) != SQLITE_OK)) {
            ROS_WARN_ONCE("Echec lors de l'ajout de l'agent");
            ROS_WARN_ONCE("SQL error2: %s", zErrMsg);

            sql = (std::string)"DELETE from id_table where id=" + id + " and name='" + name + "'; SELECT * from id_table";

            zErrMsg = 0;

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3: %s\n", zErrMsg);
            }
        } else {
            agentList.push_back(id);
            nb_agents++;
        }
    }
    sqlite3_free(zErrMsg);
    //ROS_INFO("Entity successfully added\n");
    return true;
}

/**
 * Add a fact in targeted agent's fact table
 */
bool add_facts_to_agent_db(std::string agentId, std::vector<toaster_msgs::Fact> facts) {
    //ROS_INFO("add_facts_to_agent");


    for (std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++) {
        std::string sql;
        char *zErrMsg = 0;


        //we first check if there is allready a such fact in db
        sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId +
                +" where subject_id='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId)
                + "' and predicate='" + (std::string)it->property
                + "' and propertyType='" + (std::string)it->propertyType
                + "'and target_id='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId)
                + "';";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            //ROS_INFO("Redondant fact check\n");
        }

        zErrMsg = 0;

        // update fact if allready here
        if (!myFactList.empty()) {
            sql = (std::string)"UPDATE fact_table_" + (std::string)agentId +
                    +" set valueString='" + (std::string)it->stringValue
                    + "' , valueDouble=" + boost::lexical_cast<std::string>(it->doubleValue)
                    + ", valueType=" + boost::lexical_cast<std::string>((int) it->valueType)
                    + " where subject_id='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId)
                    + "' and predicate='" + (std::string)it->property
                    + "' and propertyType='" + (std::string)it->propertyType
                    + "' and target_id='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId)
                    + "'; SELECT * from fact_table_" + (std::string)agentId + ";";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error2 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Fact successfully updated\n");
            }
        } else //else 
        {
            sql = (std::string)"INSERT INTO fact_table_" + (std::string)agentId + " (subject_id,predicate,propertyType,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) + "','"
                    + (std::string)it->property + "','"
                    + (std::string)it->propertyType + "','"
                    + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) + "','"
                    + boost::lexical_cast<std::string>((int) it->valueType) + "','"
                    + (std::string)it->stringValue + "',"
                    + boost::lexical_cast<std::string>(it->doubleValue) + ","
                    + boost::lexical_cast<std::string>(it->factObservability) + ","
                    + boost::lexical_cast<std::string>(it->confidence) + ","
                    + boost::lexical_cast<std::string>(it->time) + ",0);";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Fact successfully added to agent in table\n");
            }


            //add a new event
            if (agentId == mainAgent) {
                sql = (std::string)"INSERT INTO events_table (subject_id,predicate,propertyType,target_id,observability,confidence,time) VALUES ('"
                        + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) + "','"
                        + (std::string)it->property + "','"
                        + (std::string)it->propertyType + "','"
                        + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) + "',"
                        + boost::lexical_cast<std::string>(it->factObservability) + ","
                        + boost::lexical_cast<std::string>(it->confidence) + ","
                        + boost::lexical_cast<std::string>(it->time) + ")";

                if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                    ROS_INFO("SQL error4 : %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                } else {
                    //ROS_INFO("Event successfully added\n");
                }
            }

            //and we add into id_table some new unknown entity (id is unique so there should not be duplicates)
            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)it->subjectId + "', '" + (std::string)it->subjectId + "' , '' " + ",'" + (std::string)it->subjectOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Subject object successfully added\n");
            }


            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)it->targetId + "', '" + (std::string)it->targetId + "' , 'object' " + ",'" + (std::string)it->targetOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Target object successfully added\n");
            }
        }
        myFactList = std::vector<toaster_msgs::Fact>();
    }
    return true;
}

/**
 * Add a fact in the planning table
 */
bool add_facts_planning_db(std::vector<toaster_msgs::Fact> facts) {
    //ROS_INFO("add_facts_to_agent");

    for (std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++) {
        std::string sql;
        char *zErrMsg = 0;


        //we first check if there is allready a such fact in db
        sql = (std::string)"SELECT * from planning_table" +
                +" where subject_id='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId)
                + "' and predicate='" + (std::string)it->property
                + "' and propertyType='" + (std::string)it->propertyType
                + "'and target_id='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId)
                + "';";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            //ROS_INFO("Redondant fact check\n");
        }

        zErrMsg = 0;

        // update fact if allready here
        if (!myFactList.empty()) {
            sql = (std::string)"UPDATE planning_table" +
                    +" set valueString='" + (std::string)it->stringValue
                    + "' , valueDouble=" + boost::lexical_cast<std::string>(it->doubleValue)
                    + ", valueType=" + boost::lexical_cast<std::string>((int) it->valueType)
                    + " where subject_id='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId)
                    + "' and predicate='" + (std::string)it->property
                    + "' and propertyType='" + (std::string)it->propertyType
                    + "' and target_id='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId)
                    + "'; SELECT * from planning_table" + ";";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error2 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Fact successfully updated\n");
            }
        } else //else 
        {
            sql = (std::string)"INSERT INTO planning_table" + " (subject_id,predicate,propertyType,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) + "','"
                    + (std::string)it->property + "','"
                    + (std::string)it->propertyType + "','"
                    + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) + "','"
                    + boost::lexical_cast<std::string>((int) it->valueType) + "','"
                    + (std::string)it->stringValue + "',"
                    + boost::lexical_cast<std::string>(it->doubleValue) + ","
                    + boost::lexical_cast<std::string>(it->factObservability) + ","
                    + boost::lexical_cast<std::string>(it->confidence) + ","
                    + boost::lexical_cast<std::string>(it->time) + ",0);";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Fact successfully added to agent in table\n");
            }

        }
        myFactList = std::vector<toaster_msgs::Fact>();
    }
    return true;
}

/**
 * Remove facts form targeted agent's fact table without precising the either subject or target (put NULL instead)
 * @param reference to request
 * @param reference to response
 * @return true 
 */
bool remove_facts_to_agent_db(std::string agentId, std::vector<toaster_msgs::Fact> facts) {
    //ROS_INFO("remove_facts_to_agent");



    std::string sql;
    char *zErrMsg = 0;
    const char* data = "Callback function called";

    for (std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++) {
        //first we get all information of the fact from fact table
        if (it->targetId == "NULL" && it->subjectId == "NULL") {
            sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId +
                    " where predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else if (it->targetId == "NULL") {
            sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId +
                    " where subject_id ='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else if (it->subjectId == "NULL") {
            sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId +
                    " where target_id ='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else {
            sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId +
                    " where subject_id ='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) +
                    "' and target_id ='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        }
        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg)) {
            ROS_INFO("SQL error 1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            //ROS_INFO("Time successfully saved\n");
        }

        //then we can delete it
        if (it->targetId == "NULL" && it->subjectId == "NULL") {
            sql = "DELETE from fact_table_" + (std::string)agentId +
                    " where predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else if (it->targetId == "NULL") {
            sql = "DELETE from fact_table_" + (std::string)agentId +
                    " where subject_id ='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else if (it->subjectId == "NULL") {
            sql = "DELETE from fact_table_" + (std::string)agentId +
                    " where target_id ='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        } else {
            sql = "DELETE from fact_table_" + (std::string)agentId +
                    " where subject_id ='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId) +
                    "' and target_id ='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) +
                    "' and predicate ='" + (std::string)it->property +
                    "' and propertyType ='" + (std::string)it->propertyType + "'";
        }

        if (sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg)) {
            ROS_INFO("SQL error 2 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            //ROS_INFO("Fact successfully removed\n");
        }

        for (std::vector<toaster_msgs::Fact>::iterator itt = myFactList.begin(); itt != myFactList.end(); itt++) {
            //finally we add it into memory table
            sql = (std::string)"INSERT into memory_table_" + (std::string)agentId + " (subject_id,predicate,propertyType,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(itt->subjectId) + "','"
                    + (std::string)itt->property + "','"
                    + (std::string)itt->propertyType + "','"
                    + boost::lexical_cast<std::string>(itt->targetId) + "','"
                    + boost::lexical_cast<std::string>((int) itt->valueType) + "','"
                    + (std::string)itt->stringValue + "',"
                    + boost::lexical_cast<std::string>(itt->doubleValue) + ","
                    + boost::lexical_cast<std::string>(itt->factObservability) + ","
                    + boost::lexical_cast<std::string>(itt->confidence) + ","
                    + boost::lexical_cast<std::string>(itt->timeStart) + ","
                    + boost::lexical_cast<std::string>(itt->time) + ")";



            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                //ROS_INFO("Fact successfully added to memory table\n");
            }
            //and add a new event
            if (agentId == mainAgent) {
                ros::Time now = ros::Time::now();
                sql = (std::string)"INSERT INTO events_table (subject_id,predicate,propertyType,target_id,observability,confidence,time) VALUES ('"
                        + boost::lexical_cast<std::string>(itt->subjectId) + "','!"
                        + (std::string)itt->property + "','"
                        + (std::string)itt->propertyType + "','"
                        + boost::lexical_cast<std::string>(itt->targetId) + "',"
                        + boost::lexical_cast<std::string>(itt->factObservability) + ","
                        + boost::lexical_cast<std::string>(itt->confidence) + ","
                        + boost::lexical_cast<std::string>(now.toNSec()) + ")";

                if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                    ROS_INFO("SQL error4 : %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                } else {
                    //ROS_INFO("Event successfully added\n");
                }
            }
        }

        myFactList = std::vector<toaster_msgs::Fact>();
    }

    return true;
}

/**
 * Add an event to events table
 * @param reference to request
 * @param reference to response
 * @return true 
 */
bool add_event_db(toaster_msgs::Event event) {
    //ROS_INFO("add_event");

    std::string sql;
    char *zErrMsg = 0;

    sql = (std::string)"INSERT INTO events_table (subject_id,predicate,propertyType,target_id,observability,confidence,time) VALUES ('"
            + boost::lexical_cast<std::string>(event.subjectId) + boost::lexical_cast<std::string>(event.subjectOwnerId) + "','"
            + (std::string)event.property + "','"
            + (std::string)event.propertyType + "','"
            + boost::lexical_cast<std::string>(event.targetId) + boost::lexical_cast<std::string>(event.targetOwnerId) + "',"
            + boost::lexical_cast<std::string>(event.factObservability) + ", "
            + boost::lexical_cast<std::string>(event.confidence) + ", "
            + boost::lexical_cast<std::string>(event.time) + ");";

    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_INFO("SQL error1: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //ROS_INFO("Event successfully added");
    }
    return true;
}

/**
 * Empty the planning table
 */
void empty_database_planning_db() {
    //ROS_INFO("sql order");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;

    sql = (std::string)"DELETE from planning_table";

    if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // ROS_INFO("SQL order obtained successfully\n");
    }

}

/**
 * Add or remove information to the db
 * @add: true if add, false if remove
 * @infoType: ENTITY, FACT or EVENT 
 */
bool set_info_db(toaster_msgs::SetInfoDB::Request &req, toaster_msgs::SetInfoDB::Response &res) {

    if (req.infoType == "ENTITY") {
        return add_entity_db(req.id, req.ownerId, req.name, req.type);
    } else if (req.infoType == "FACT") {
        if (req.add) {
            if (req.agentId == "PLANNING") {
                return add_facts_planning_db(req.facts);
            } else {
                return add_facts_to_agent_db(req.agentId, req.facts);
            }
        } else {
            return remove_facts_to_agent_db(req.agentId, req.facts);
        }
    } else if (req.infoType == "RESET_PLANNING") {
        empty_database_planning_db();
        return add_facts_planning_db(req.facts);
    } else if (req.infoType == "EVENT") {
        return add_event_db(req.event);
    }
    return false;
}




////////////////getting info//////////////////////////

/**
 * Get all facts from the planning table
 */
std::pair<bool, toaster_msgs::FactList> get_all_facts_planning_db() {
    //ROS_INFO("get_facts_from_agent");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, toaster_msgs::FactList> res;

    sql = (std::string)"SELECT * from planning_table";

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1376: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Facts from agent obtained successfully\n");
    }


    //return informations from table
    if (!myFactList.empty()) {
        for (int i = 0; i < myFactList.size(); i++) {
            res.second.factList.push_back(myFactList[i]);
        }
        res.first = true;
    } else {
        res.first = true;
    }

    myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


    return res;
}

/**
 * Get all facts known from an agent (current and past)
 */
std::pair<bool, toaster_msgs::FactList> get_all_facts_from_agent_db(std::string agentId) {
    //ROS_INFO("get_facts_from_agent");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, toaster_msgs::FactList> res;

    sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId;

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1376: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Facts from agent obtained successfully\n");
    }

    sql = (std::string)"SELECT * from memory_table_" + (std::string)agentId;

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1385: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Facts from agent memory obtained successfully\n");
    }

    //return informations from table
    if (!myFactList.empty()) {
        for (int i = 0; i < myFactList.size(); i++) {
            res.second.factList.push_back(myFactList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


    return res;
}

/**
 * Get values from a known fact in agent's fact table or memory table
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, toaster_msgs::FactList> get_fact_value_from_agent_db(std::string agentId, toaster_msgs::Fact reqFact) {
    //ROS_INFO("get_fact_value_from_agent");


    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, toaster_msgs::FactList> res;



    sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId
            + " where subject_id='" + boost::lexical_cast<std::string>(reqFact.subjectId) + boost::lexical_cast<std::string>(reqFact.subjectOwnerId)
            + "' and predicate='" + (std::string)reqFact.property
            + "' and propertyType='" + (std::string)reqFact.propertyType
            + "' and target_id='" + boost::lexical_cast<std::string>(reqFact.targetId) + boost::lexical_cast<std::string>(reqFact.targetOwnerId) + "';";

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1431: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Current fact value from robot obtained successfully\n");
    }

    sql = (std::string)"SELECT * from memory_table_" + (std::string)agentId
            + " where subject_id='" + boost::lexical_cast<std::string>(reqFact.subjectId) + boost::lexical_cast<std::string>(reqFact.subjectOwnerId)
            + "' and predicate='" + (std::string)reqFact.property
            + "' and propertyType='" + (std::string)reqFact.propertyType
            + "' and target_id='" + boost::lexical_cast<std::string>(reqFact.targetId) + boost::lexical_cast<std::string>(reqFact.targetOwnerId) + "';";

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1443: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Fact value from robot memory obtained successfully\n");
    }

    //return informations from table
    if (!myFactList.empty()) {
        res.second.factList.push_back(myFactList[0]);
        res.first = true;
    } else {
        res.first = false;
    }

    myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


    return res;
}

/**
 * Get all current facts known from an agent 
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, toaster_msgs::FactList> get_current_facts_from_agent_db(std::string agentId) {
    //ROS_INFO("get_currents_facts_from_agent");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, toaster_msgs::FactList> res;

    sql = (std::string)"SELECT * from fact_table_" + (std::string)agentId;

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1481: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Facts from agent obtained successfully\n");
    }

    //return informations from table
    if (!myFactList.empty()) {
        for (int i = 0; i < myFactList.size(); i++) {
            res.second.factList.push_back(myFactList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


    return res;
}

/**
 * Get all paased facts known from an agent 
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, toaster_msgs::FactList> get_passed_facts_from_agent_db(std::string agentId) {
    //ROS_INFO("get_passed_facts_from_agent");


    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, toaster_msgs::FactList> res;

    sql = (std::string)"SELECT * from memory_table_" + (std::string)agentId;

    if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1522: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Facts from agent memory obtained successfully\n");
    }

    //return informations from table
    if (!myFactList.empty()) {
        for (int i = 0; i < myFactList.size(); i++) {
            res.second.factList.push_back(myFactList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


    return res;
}

std::pair<bool, std::vector<toaster_msgs::Property> > get_properties_db() {
    //ROS_INFO("get_properties");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Property> > res;

    sql = (std::string)"SELECT * from static_property_table;";

    if (sqlite3_exec(database, sql.c_str(), property_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1575: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Static property table obtained successfully\n");
    }

    //return informations from table
    if (!myPropertyList.empty()) {
        for (int i = 0; i < myPropertyList.size(); i++) {
            res.second.push_back(myPropertyList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myPropertyList = std::vector<toaster_msgs::Property>(); //empty myPropertyList

    return res;
}

/**
 * Get values of a specific static property 
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Property> > get_property_value_db(int id) {
    //ROS_INFO("get_property_value");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Property> > res;

    sql = (std::string)"SELECT * from static_property_table" + " where id=" + boost::lexical_cast<std::string>(id);

    if (sqlite3_exec(database, sql.c_str(), property_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1612: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Static property value obtained successfully\n");
    }

    //return informations from table
    if (!myPropertyList.empty()) {
        res.second.push_back(myPropertyList[0]);
        res.first = true;
    } else {
        res.first = false;
    }

    myPropertyList = std::vector<toaster_msgs::Property>(); //empty myPropertyList

    return res;
}

/**
 * Get all agents
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Id> > get_agents_db() {
    //ROS_INFO("get_agents");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Id> > res;

    sql = (std::string)"SELECT * from id_table where type ='human' or type='robot';";

    if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1647: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Agents from id_table obtained successfully\n");
    }

    //return informations from table		
    if (!myEntityList.empty()) {
        for (int i = 0; i < myEntityList.size(); i++) {
            res.second.push_back(myEntityList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

    return res;
}

/**
 * Get all ids
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Id> > get_id_db() {
    //ROS_INFO("get_all_id");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Id> > res;

    sql = (std::string)"SELECT * from id_table;";

    if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1684: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // fprintf(stdout, "Id table obtained successfully\n");
    }

    //return informations from table		
    if (!myEntityList.empty()) {
        for (int i = 0; i < myEntityList.size(); i++) {
            res.second.push_back(myEntityList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

    return res;
}

/**
 * Get values of a specific agent 
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Id> > get_id_value_db(std::string id, std::string name) {
    //ROS_INFO("get_id_value");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Id> > res;

    sql = (std::string)"SELECT * from id_table where id='" + boost::lexical_cast<std::string>(id) + "' and name='" + (std::string)name + "';";

    if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1721: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Id informations obtained successfully\n");
    }

    //return informations from table
    if (!myEntityList.empty()) {
        res.second.push_back(myEntityList[0]);
        res.first = true;
    } else {
        res.first = false;
    }

    myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

    return res;
}

/**
 * Get all events
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Event> > get_events_db() {
    //ROS_INFO("get_event");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Event> > res;

    sql = (std::string)"SELECT * from events_table;";

    if (sqlite3_exec(database, sql.c_str(), event_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1756: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Events table obtained successfully\n");
    }

    //return informations from table
    if (!myEventList.empty()) {
        for (int i = 0; i < myEventList.size(); i++) {
            res.second.push_back(myEventList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myEventList = std::vector<toaster_msgs::Event>(); //empty myEventList

    return res;
}

/**
 * Get values of a specific event
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Event> > get_event_value_db(toaster_msgs::Event reqEvent) {
    //ROS_INFO("get_event_value");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Event> > res;

    sql = (std::string)"SELECT * from events_table where subject_id='" + boost::lexical_cast<std::string>(reqEvent.subjectId)
            + "' and predicate='" + (std::string)reqEvent.property
            + "' and propertyType='" + (std::string)reqEvent.propertyType
            + "' and target_id='" + boost::lexical_cast<std::string>(reqEvent.targetId) + "';";

    if (sqlite3_exec(database, sql.c_str(), event_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1797: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // fprintf(stdout, "Event obtained successfully\n");
    }

    //return informations from table
    if (!myEventList.empty()) {
        res.second.push_back(myEventList[0]);
        res.first = true;
    } else {

        res.first = false;
    }

    myEventList = std::vector<toaster_msgs::Event>(); //empty myEventList

    return res;
}

/**
 * Get all ontologies
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Ontology> > get_ontologies_db() {
    //ROS_INFO("get_ontologies");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Ontology> > res;

    sql = (std::string)"SELECT * from ontology_table;";

    if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1835: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Ontology table obtained successfully\n");
    }

    //return informations from table
    if (!myOntologyList.empty()) {
        for (int i = 0; i < myOntologyList.size(); i++) {
            res.second.push_back(myOntologyList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

    return res;
}

/**
 * Get values of a all leaves of ontology table starting from a entityClass field
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Ontology> > get_ontology_leaves_db(std::string entityClass) {
    //ROS_INFO("get_ontology_leaves");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Ontology> > res;

    sql = (std::string) "WITH tree(g,ind,inst)  AS ( "
            + "SELECT entityClass ,individual, instantiated  FROM ontology_table WHERE entityClass ='" + (std::string)entityClass
            + "' UNION ALL"
            + " SELECT entityClass ,individual, instantiated  FROM ontology_table o"
            + " INNER JOIN tree t"
            + " ON t.ind = o.entityClass)"
            + "SELECT * FROM tree where inst = 'true';";

    if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1878: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Ontology leaves obtained successfully\n");
    }

    //return informations from table
    if (!myOntologyList.empty()) {
        for (int i = 0; i < myOntologyList.size(); i++) {
            res.second.push_back(myOntologyList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

    return res;
}

/**
 * Get values of a specific ontology
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<toaster_msgs::Ontology> > get_ontology_values_db(std::string entityClass) {
    //ROS_INFO("get_ontology_values");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<toaster_msgs::Ontology> > res;

    sql = (std::string)"SELECT * from ontology_table where entityClass='" + (std::string)entityClass + "';";

    if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1915: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        //fprintf(stdout, "Ontology values obtained successfully\n");
    }

    //return informations from table
    if (!myOntologyList.empty()) {
        for (int i = 0; i < myOntologyList.size(); i++) {
            res.second.push_back(myOntologyList[i]);
        }
        res.first = true;
    } else {
        res.first = false;
    }

    myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

    return res;
}

/**
 * get information from the db
 * @type = FACT,PROPERTY,AGENT, ID, EVENT, ONTOLOGY
 * @subtype = ALL, VALUE, LEAVE, CURRENT or OLD
 * 
 */
bool get_info_db(toaster_msgs::GetInfoDB::Request &req, toaster_msgs::GetInfoDB::Response &res) {

    if (req.type == "FACT") {
        if (req.subType == "ALL") {
            std::pair<bool, toaster_msgs::FactList> answer = get_all_facts_from_agent_db(req.agentId);
            res.boolAnswer = answer.first;
            res.resFactList = answer.second;
        } else if (req.subType == "VALUE") {
            std::pair<bool, toaster_msgs::FactList> answer = get_fact_value_from_agent_db(req.agentId, req.reqFact);
            res.boolAnswer = answer.first;
            res.resFactList = answer.second;
        } else if (req.subType == "CURRENT") {
            std::pair<bool, toaster_msgs::FactList> answer = get_current_facts_from_agent_db(req.agentId);
            res.boolAnswer = answer.first;
            res.resFactList = answer.second;
        } else if (req.subType == "OLD") {
            std::pair<bool, toaster_msgs::FactList> answer = get_passed_facts_from_agent_db(req.agentId);
            res.boolAnswer = answer.first;
            res.resFactList = answer.second;
        } else if (req.subType == "PLANNING") {
            std::pair<bool, toaster_msgs::FactList> answer = get_all_facts_planning_db();
            res.boolAnswer = answer.first;
            res.resFactList = answer.second;
        }
    } else if (req.type == "PROPERTY") {
        if (req.subType == "ALL") {
            std::pair<bool, std::vector<toaster_msgs::Property> > answer = get_properties_db();
            res.boolAnswer = answer.first;
            res.resProperties = answer.second;
        } else if (req.subType == "VALUE") {
            std::pair<bool, std::vector<toaster_msgs::Property> > answer = get_property_value_db(req.id);
            res.boolAnswer = answer.first;
            res.resProperties = answer.second;
        }
    } else if (req.type == "AGENT") {
        std::pair<bool, std::vector<toaster_msgs::Id> > answer = get_agents_db();
        res.boolAnswer = answer.first;
        res.resId = answer.second;
    } else if (req.type == "ID") {
        if (req.subType == "ALL") {
            std::pair<bool, std::vector<toaster_msgs::Id> > answer = get_id_db();
            res.boolAnswer = answer.first;
            res.resId = answer.second;
        } else if (req.subType == "VALUE") {
            std::pair<bool, std::vector<toaster_msgs::Id> > answer = get_id_value_db(req.idString, req.name);
            res.boolAnswer = answer.first;
            res.resId = answer.second;
        }
    } else if (req.type == "EVENT") {
        if (req.subType == "ALL") {
            std::pair<bool, std::vector<toaster_msgs::Event> > answer = get_events_db();
            res.boolAnswer = answer.first;
            res.resEventList = answer.second;
        } else if (req.subType == "VALUE") {
            std::pair<bool, std::vector<toaster_msgs::Event> > answer = get_event_value_db(req.reqEvent);
            res.boolAnswer = answer.first;
            res.resEventList = answer.second;
        }
    } else if (req.type == "ONTOLOGY") {
        if (req.subType == "ALL") {
            std::pair<bool, std::vector<toaster_msgs::Ontology> > answer = get_ontologies_db();
            res.boolAnswer = answer.first;
            res.resOntology = answer.second;
        } else if (req.subType == "VALUE") {
            std::pair<bool, std::vector<toaster_msgs::Ontology> > answer = get_ontology_values_db(req.entityClass);
            res.boolAnswer = answer.first;
            res.resOntology = answer.second;
        } else if (req.subType == "LEAVE") {
            std::pair<bool, std::vector<toaster_msgs::Ontology> > answer = get_ontology_leaves_db(req.entityClass);
            res.boolAnswer = answer.first;
            res.resOntology = answer.second;
        }
    }
    return true;
}

////////////////EXECUTE SERVICE/////////////////////////////

/**
 * Return true if a given list of fact is in the fact table of a given agent
 */
bool are_in_table_db(std::string agent, std::vector<toaster_msgs::Fact> facts) {

    std::string sql;
    char *zErrMsg = 0;
    const char* data = "Callback function called";

    for (std::vector<toaster_msgs::Fact>::iterator it = facts.begin(); it != facts.end(); it++) {
        sql = (std::string)"SELECT * from fact_table_" + agent
                + " where subject_id='" + boost::lexical_cast<std::string>(it->subjectId) + boost::lexical_cast<std::string>(it->subjectOwnerId)
                + "' and predicate='" + boost::lexical_cast<std::string>(it->property)
                + "' and target_id='" + boost::lexical_cast<std::string>(it->targetId) + boost::lexical_cast<std::string>(it->targetOwnerId) + "';";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1857: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            // fprintf(stdout, "Current fact value from robot obtained successfully\n");
        }

        //return informations from table
        if (myFactList.empty()) {
            return false;

        }
        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList
    }

    return true;
}

/**
 * Execute in selected table the SQl request casted in the Request.order field 
 * @param reference to request
 * @param reference to response
 * @return true 
 */
std::pair<bool, std::vector<std::string> > execute_SQL_db(std::string order) {
    //ROS_INFO("sql order");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;
    std::pair<bool, std::vector<std::string> > res;

    sql = order;

    if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l1961: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // ROS_INFO("SQL order obtained successfully\n");
    }

    //return informations from table
    for (int i = 0; i < myStringList.size(); i++) {
        res.second.push_back(myStringList[i]);
    }
    res.first = true;

    myStringList = std::vector<std::string>(); //empty myStringList

    return res;
}

/**
 * Empty the table of all agents
 */
void empty_database_db() {
    //ROS_INFO("sql order");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;


    for (std::vector<std::string>::iterator it = agentList.begin(); it != agentList.end(); it++) {
        sql = (std::string)"DELETE from fact_table_" + *it;

        if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            // ROS_INFO("SQL order obtained successfully\n");
        }

        sql = (std::string)"DELETE from memory_table_" + *it;

        if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            // ROS_INFO("SQL order obtained successfully\n");
        }
    }

    sql = (std::string)"DELETE from events_table";

    if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // ROS_INFO("SQL order obtained successfully\n");
    }

}

/**
 * Empty the table of an agents
 */
void empty_database_for_agent_db(std::string agent) {
    //ROS_INFO("sql order");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;

    sql = (std::string)"DELETE from fact_table_" + agent;

    if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // ROS_INFO("SQL order obtained successfully\n");
    }

    sql = (std::string)"DELETE from memory_table_" + agent;

    if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
        fprintf(stderr, "SQL error l2205: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        // ROS_INFO("SQL order obtained successfully\n");
    }

}

/** removes the tables of all the agents, vacate event table and set new id table (i.e as an input) 
and create new memory and fact tables accordingly. Save new id_list xml file in /database_manager/database and 
in execute_db service use newIDTable as '/database/filename' where filename is name of xml file
with new ID_table list **/

bool reset_tables(std::string newIDTable) {
    char **pazResult;
    int pnRow;
    int pnColumn;
    char *err_msg = NULL;
    int ret;
    int i;
    char *zErrMsg = 0;
    const char* data = "Callback function called";
    // reading current id_table
    std::string sql = (std::string)"SELECT * from id_table";
    ret = sqlite3_get_table(database, sql.c_str(), &pazResult, &pnRow, &pnColumn, &err_msg);
    if (ret != SQLITE_OK) {
        fprintf(stderr, "Error: %s\n", err_msg);
        sqlite3_free(err_msg);
        return false;

    } else {
        for (i = 1; i <= (pnRow); i++) {
            std::string agentId = pazResult[(i * (pnColumn)) + 0];
            std::string type = pazResult[(i * (pnColumn)) + 2];
            boost::algorithm::to_lower(agentId);

            if (!type.compare("robot") || !type.compare("human")) {
                // removing fact_table and memory table for existing agents
                std::string sql1 = (std::string)"DROP TABLE fact_table_" + agentId;
                std::string sql2 = (std::string)"DROP TABLE memory_table_" + agentId;
                if (sqlite3_exec(database, sql1.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
                    fprintf(stderr, "SQL error : %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                    return false;
                } else {
                    //ROS_INFO("fact table dropped successfully\n");
                }
                if (sqlite3_exec(database, sql2.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
                    fprintf(stderr, "SQL error : %s\n", zErrMsg);
                    sqlite3_free(zErrMsg);
                    return false;
                } else {
                    //ROS_INFO("memory table dropped successfully\n");
                }

            }
        }
        // removing current id_table
        std::string sql3 = (std::string)"DROP TABLE id_table";
        if (sqlite3_exec(database, sql3.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
            return false;

        } else {
            //ROS_INFO("id table dropped successfully\n");
        }

        sql = (std::string)"CREATE TABLE id_table(" +
                "id 		 CHAR(50)," +
                "name        	 CHAR(50)," +
                "type       	 CHAR(50)," +
                "owner_id      	 INT," +
                "unique(id) );";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_WARN_ONCE("SQL error l174: %s", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            // clears current agentList
            agentList.clear();
            //get id informations form new id_list and create new id table and also fact_table and memory_table for agents in it 
            launchIdList(newIDTable);
            //ROS_INFO("Opened id table successfully\n");
        }

    }

    return true;
}

/**
 * Set the topics the database should read
 */
void set_topics(bool areaTopic, bool agentTopic, bool move3dTopic, bool pdgTopic) {

    factsReaders.clear();
    if (areaTopic) {
        factsReaders.push_back(readerArea);
    }
    if (agentTopic) {
        factsReaders.push_back(readerAgent);
    }
    if (move3dTopic) {
        factsReaders.push_back(readerMove3d);
    }
    if (pdgTopic) {
        factsReaders.push_back(readerPdg);
    }

}

/**
 * print the db in the console
 * @return void
 */
void print() {
    ROS_INFO("*********************** database info ***************************");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;

    //
    ROS_INFO(" ");
    ROS_WARN("reading id database");

    sql = (std::string)"SELECT * from id_table;";
    sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg);

    //
    ROS_INFO(" ");
    ROS_WARN("reading static property database");

    sql = (std::string)"SELECT * from static_property_table;";
    //sqlite3_exec(database, sql.c_str(), callback, (void*)data, &zErrMsg);

    //
    ROS_INFO(" ");
    ROS_WARN("reading fact database");

    for (int i = 0; i < agentList.size(); i++) {
        std::cout << "\nfact_table_" << (std::string)agentList[i] << "\n";
        sql = (std::string)"SELECT * from fact_table_" + (std::string)agentList[i];
        sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg);
    }

    //
    ROS_INFO(" ");
    ROS_WARN("reading memory database");

    for (int i = 0; i < agentList.size(); i++) {
        std::cout << "\nmemory_table_" << (std::string)agentList[i] << "\n";
        sql = (std::string)"SELECT * from memory_table_" + (std::string)agentList[i];
        sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg);
    }

    //
    ROS_INFO(" ");
    ROS_WARN("reading ontology database");

    sql = (std::string)"SELECT * from ontology_table;";
    //sqlite3_exec(database, sql.c_str(), callback, (void*)data, &zErrMsg);

    //
    ROS_INFO(" ");
    ROS_WARN("reading events database");

    sql = (std::string)"SELECT * from events_table;";
    sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg);
}

/**
 * print the db in the console
 * @return void
 */
void print_agent(std::string agent) {
    ROS_INFO("*********************** database info ***************************");

    char *zErrMsg = 0;
    const char* data = "Callback function called";
    std::string sql;

    //
    ROS_INFO(" ");
    ROS_WARN("reading fact database");

    for (int i = 0; i < agentList.size(); i++) {
        if ((std::string)agentList[i] == agent) {
            std::cout << "\nfact_table_" << (std::string)agentList[i] << "\n";
            sql = (std::string)"SELECT * from fact_table_" + (std::string)agentList[i];
            sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg);
        }
    }

}

/**
 * execute commands 
 * @command: ARE_IN_TABLE, SQL, EMPTY, PRINT, SET_TOPICS
 * @type: ALL, AGENT 
 */
bool execute_db(toaster_msgs::ExecuteDB::Request &req, toaster_msgs::ExecuteDB::Response &res) {

    if (req.command == "ARE_IN_TABLE") {
        res.boolAnswer = are_in_table_db(req.agent, req.facts);
    } else if (req.command == "SQL") {
        std::pair<bool, std::vector<std::string> > answer = execute_SQL_db(req.order);
        res.boolAnswer = answer.first;
        res.results = answer.second;
    } else if (req.command == "EMPTY") {
        if (req.type == "ALL") {
            empty_database_db();
        } else if (req.type == "AGENT") {
            empty_database_for_agent_db(req.agent);
        } else if (req.type == "PLANNING") {
            empty_database_planning_db();
        }
    } else if (req.command == "PRINT") {
        if (req.type == "ALL") {
            print();
        } else if (req.type == "AGENT") {
            print_agent(req.agent);
        }
    } else if (req.command == "SET_TOPICS") {
        set_topics(req.areaTopic, req.agentTopic, req.move3dTopic, req.pdgTopic);
    } else if (req.command == "RESET_TABLES") {
        res.boolAnswer = reset_tables(req.newIDList);
    }
    return true;
}
//////////////////////////////////
///for graphical representation///
/////////////////////////////////

/**
 * plot facts transitions in file package/plot_fact/req.reqFact.dat
 */
bool plot_facts_db(toaster_msgs::PlotFactsDB::Request &req, toaster_msgs::PlotFactsDB::Response &res) {

    std::string sql;
    char **pazResult;
    int pnRow;
    int pnColumn;
    char *err_msg = NULL;
    int ret;
    long long int i;


    // sql query to get events related to the requested entity
    // within the given time window and for the requested fact.
    sql = (std::string)"SELECT * from events_table where subject_id='" +
            boost::lexical_cast<std::string>(req.subjectID) + "' and target_id='" +
            boost::lexical_cast<std::string>(req.targetID) + "' and time>='" +
            boost::lexical_cast<std::string>(req.timeStart) + "' and time<='" +
            boost::lexical_cast<std::string>(req.timeEnd) + "' and (predicate ='" +
            boost::lexical_cast<std::string>(req.reqFact) + "' or predicate='!" +
            boost::lexical_cast<std::string>(req.reqFact) + "');";

    // results of the query are stored in the table
    ret = sqlite3_get_table(database, sql.c_str(), &pazResult, &pnRow, &pnColumn, &err_msg);

    if (ret != SQLITE_OK) {
        fprintf(stderr, "Error: %s\n", err_msg);
        sqlite3_free(err_msg);
        res.boolAnswer = false;
        return false;
    } else {
        std::map<long long int, int> fn;
        std::vector<long long int> data(pnRow + 2);
        std::string start = boost::lexical_cast<std::string>(req.timeStart);
        data[0] = std::atoll(start.c_str());
        long long int timeStart = data[0];
        std::string end = boost::lexical_cast<std::string>(req.timeEnd);
        std::string reqFact = boost::lexical_cast<std::string>(req.reqFact);
        long long int timeEnd = std::atoll(end.c_str());

        // interpreting state of fact at start time
        if (pazResult[(1 * (pnColumn)) + 1] == reqFact && atoll(pazResult[(1 * (pnColumn)) + 6]) >= timeStart) {
            fn[timeStart] = 0;
        }//if first event within the window is positive (like IsMoving) then state of fact should be false at starting point of window
        else {
            fn[timeStart] = 1;
        } //if first event within the window is negative (like !IsMoving) then state of fact should be false at starting point of window

        // interpreting state of fact at end time
        if (pazResult[(pnRow * (pnColumn)) + 1] == reqFact && atoll(pazResult[(pnRow * (pnColumn)) + 6]) <= timeEnd) {
            fn[timeEnd] = 1;
        } else {
            fn[timeEnd] = 0;
        }

        data[pnRow + 1] = timeEnd;

        // allocation of the value to state of fact i.,e 0 or 1
        for (i = 1; i <= (pnRow); i++) {
            fprintf(stderr, "subject_id: %s \n predicate: %s \n propertyType: %s \n time: %s \n",
                    pazResult[(i * (pnColumn)) + 0], pazResult[(i * (pnColumn)) + 1], pazResult[(i * (pnColumn)) + 2], pazResult[(i * (pnColumn)) + 6]);
            data[i] = atoll(pazResult[(i * (pnColumn)) + 6]);
            std::string status = pazResult[(i * (pnColumn)) + 1];

            if (status.compare(reqFact))
                fn[atoll(pazResult[(i * (pnColumn)) + 6])] = 1;
            else
                fn[atoll(pazResult[(i * (pnColumn)) + 6])] = 0;
        }

        std::ofstream myfile;
        std::string myPath = ros::package::getPath("database_manager") + "/plot_fact/" + boost::lexical_cast<std::string>(req.reqFact) + ".dat";
        myfile.open(myPath.c_str());
        ROS_INFO("file path is %s", myPath.c_str()); // open gnuplot terminal and plot the data in this file
        long long int diff = timeEnd - timeStart;

        // writing data to fact with first column as time stamp and second column 0 or 1 (state of fact)
        ROS_INFO("Writing to fact.dat file");
        for (i = 0; i < diff; i = i + 100000) {
            long long int x = timeStart + i;
            int k = 0;
            int flag = 0;
            for (int j = 0; j < pnRow + 2; j++) {
                if (x < data[j]) {
                    k = j - 1;
                    flag = 1;
                }
                if (flag == 1)
                    break;
            }
            int y = fn[data[k]];
            myfile << x << "        " << y << "\n";
        }
        // closing the file
        myfile.close();
        res.boolAnswer = true;
        return true;


    }
}

////////////////////////////////////////////
/////////TO LOAD and SAVE the database///////
////////////////////////////////////////////

/**
 * uses sql functions to load and save the current state of the db
 */
bool load_save_db(toaster_msgs::LoadSaveDB::Request &req, toaster_msgs::LoadSaveDB::Response &res) {
    int rc; /* Function return code */
    sqlite3 *pFile; /* Database connection opened on zFilename */
    sqlite3_backup *pBackup; /* Backup object used to copy data */
    sqlite3 *pTo; /* Database to copy to (pFile or pInMemory) */
    sqlite3 *pFrom; /* Database to copy from (pFile or pInMemory) */
    std::string const fileName = boost::lexical_cast<std::string>(req.fileName);
    rc = sqlite3_open(fileName.c_str(), &pFile);
    if (rc == SQLITE_OK) {

        pFrom = (req.toSave ? database : pFile);
        pTo = (req.toSave ? pFile : database);
        pBackup = sqlite3_backup_init(pTo, "main", pFrom, "main");
        if (pBackup) {
            (void) sqlite3_backup_step(pBackup, -1);
            (void) sqlite3_backup_finish(pBackup);
        }
        rc = sqlite3_errcode(pTo);
    }
    (void) sqlite3_close(pFile);
    res.sqlstatus = rc;
    return true;


}

///////////////////////////////////////////////////////////
// UPDATE WORLD STATE ////////////
//////////////////////////////////////////////////////////

bool isVisibleBy(std::string entity, std::string agent) {

    if (entity == agent) {
        return true;
    }

    std::vector<toaster_msgs::Fact> toTestVector;

    toaster_msgs::Fact toTest;
    toTest.subjectId = entity;
    toTest.targetId = agent;
    toTest.property = "isVisibleBy";

    toTestVector.push_back(toTest);

    return are_in_table_db(mainAgent, toTestVector);

}

void update_world_states(ros::NodeHandle& node, std::vector<ToasterFactReader*> factsReader) {
    /**************************/
    /* World State management */
    /**************************/

    //We get the new state
    std::vector<toaster_msgs::Fact> newState;
    for (std::vector<ToasterFactReader*>::iterator it = factsReader.begin(); it != factsReader.end(); it++) {
        if ((*it)->lastMsgFact.factList.size() > 0) {
            newState.insert(newState.end(), (*it)->lastMsgFact.factList.begin(), (*it)->lastMsgFact.factList.end());
        }
    }
    //If update, make modification to current db:

    std::vector<toaster_msgs::Fact> toAdd;
    bool find = false;
    for (int i = 0; i < newState.size(); ++i) {
        for (int j = 0; j < previousFactsState.size(); ++j) {
            if ((previousFactsState[j].subjectId.compare(newState[i].subjectId) == 0)
                    && (previousFactsState[j].property.compare(newState[i].property) == 0)
                    && (previousFactsState[j].propertyType.compare(newState[i].propertyType) == 0)
                    && (previousFactsState[j].subProperty.compare(newState[i].subProperty) == 0)
                    && (previousFactsState[j].subjectOwnerId.compare(newState[i].subjectOwnerId) == 0)
                    && (previousFactsState[j].targetId.compare(newState[i].targetId) == 0)
                    && (previousFactsState[j].targetOwnerId.compare(newState[i].targetOwnerId) == 0)
                    && (previousFactsState[j].targetId.compare(newState[i].targetId) == 0)) {

                find = true;
                break;
            } else
                continue;
        }
        // Remove facts that are no longer there
        if (!find) {
            toAdd.push_back(newState[i]);
        }
        find = false;
    }
    if (toAdd.size() > 0) {
        add_facts_to_agent_db(mainAgent, toAdd);
    }

    std::vector<toaster_msgs::Fact> toRemove;
    bool removedFact = true;
    for (int j = 0; j < previousFactsState.size(); ++j) {
        for (int i = 0; i < newState.size(); ++i) {
            if ((previousFactsState[j].subjectId.compare(newState[i].subjectId) == 0)
                    && (previousFactsState[j].property.compare(newState[i].property) == 0)
                    && (previousFactsState[j].propertyType.compare(newState[i].propertyType) == 0)
                    && (previousFactsState[j].subProperty.compare(newState[i].subProperty) == 0)
                    && (previousFactsState[j].subjectOwnerId.compare(newState[i].subjectOwnerId) == 0)
                    && (previousFactsState[j].targetId.compare(newState[i].targetId) == 0)
                    && (previousFactsState[j].targetOwnerId.compare(newState[i].targetOwnerId) == 0)
                    && (previousFactsState[j].targetId.compare(newState[i].targetId) == 0)) {
                removedFact = false;
                break;
            } else
                continue;
        }
        // Remove facts that are no longer there
        if (removedFact) {
            toRemove.push_back(previousFactsState[j]);
        }
        removedFact = true;
    }
    if (toRemove.size() > 0) {
        remove_facts_to_agent_db(mainAgent, toRemove);
    }

    previousFactsState = newState;
}

void conceptual_perspective_taking() {

    std::vector<toaster_msgs::Fact> toTest, toAdd, toRm;

    //we get the main agent facts
    std::pair<bool, toaster_msgs::FactList> response = get_current_facts_from_agent_db(mainAgent);

    //for all other agents we add all facts observable and where subject and target are visible.
    for (int i = 1; i < agentList.size(); i++) {
        for (int y = 0; y < response.second.factList.size(); y++) {
            if (response.second.factList[y].factObservability > 0.0) {
                if (isVisibleBy(response.second.factList[y].subjectId, agentList[i]) && isVisibleBy(response.second.factList[y].targetId, agentList[i])) {
                    //check if the fact is already in the agent table
                    toTest.push_back(response.second.factList[y]);
                    if (!are_in_table_db(agentList[i], toTest)) {
                        toAdd.push_back(response.second.factList[y]);
                    }
                    toTest.clear();
                }
            }
        }
        if (toAdd.size() > 0) {
            add_facts_to_agent_db(agentList[i], toAdd);
            toAdd.clear();
        }
    }

    //for all other agents we check if the fact in their table are still in the robot table
    for (int i = 1; i < agentList.size(); i++) {
        response = get_current_facts_from_agent_db(agentList[i]);
        for (int y = 0; y < response.second.factList.size(); y++) {
            if (!are_in_table_db(mainAgent, toTest)) {
                if (isVisibleBy(response.second.factList[y].subjectId, agentList[i]) && isVisibleBy(response.second.factList[y].targetId, agentList[i])) {
                    toRm.push_back(response.second.factList[y]);
                }
            }
            toTest.clear();
        }
        if (toRm.size() > 0) {
            remove_facts_to_agent_db(agentList[i], toRm);
            toRm.clear();
        }
    }
}

//init server

void initServer() {


    ////////////////////////////
    //// DATABASE CREATION /////
    ////////////////////////////

    char *zErrMsg = 0;
    std::string sql;
    nb_agents = 0;

    if (sqlite3_open(NULL, &database)) {
        ROS_WARN_ONCE("Can't create database: %s", sqlite3_errmsg(database));
        exit(0);
    }



    //// ID TABLE CREATION ///////
    sql = (std::string)"CREATE TABLE id_table(" +
            "id 		 CHAR(50)," +
            "name        	 CHAR(50)," +
            "type       	 CHAR(50)," +
            "owner_id      	 INT," +
            "unique(id) );";

    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_WARN_ONCE("SQL error l174: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        std::string idList = "/database/id_list.xml";
        launchIdList(idList); //get id informations form xml file
        ROS_INFO("Opened id table successfully\n");
    }


    //// ONTOLOGY TABLE CREATION //////
    sql = (std::string)"CREATE TABLE ontology_table(" +
            "entityClass  		 CHAR(50)," +
            "individual        	 CHAR(50)," +
            "instantiated      	 BOOLEAN);";


    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_WARN_ONCE("SQL error l189: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        ROS_INFO("Opened ontology table successfully\n");
        launchOntology();
    }


    //// PROPERTY TABLE CREATION /////
    sql = (std::string)"CREATE TABLE static_property_table(" +
            "id 			 INT," +
            "color             	 CHAR(50)," +
            "height        		 INT," +
            "linkedToId      	 INT," +
            "linkType       	 CHAR(50), " +
            " unique (id) );";

    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_WARN_ONCE("SQL error l207: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        ROS_INFO("Opened static property table successfully\n");
        launchStaticPropertyDatabase(); //get static properties informations form xml file
    }


    //// EVENTS TABLE CREATION /////
    sql = (std::string)"CREATE TABLE events_table(" +
            "subject_id 				unsigned long," +
            "predicate         	                string," +
            "propertyType                          string," +
            "target_id        		     	unsigned long," +
            "observability   		    	unsinged short," +
            "confidence   		        	unsigned short," +
            "time   				int)";

    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_WARN_ONCE("SQL error l225: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        ROS_INFO("Opened events table successfully\n");
    }

    //PLANNING TABLE CREATION
    sql = (std::string)"CREATE TABLE planning_table(" +
            "subject_id 				 	unsigned long," +
            "predicate         	            string," +
            "propertyType                          string," +
            "target_id        		     	unsigned long," +
            "valueType			  	        bit," +
            "valueString       	        	string," +
            "valueDouble       	        	double," +
            "observability   		   		unsinged short," +
            "confidence   		        	unsigned short," +
            "start   				        int," +
            "end 					        int," +
            "unique (subject_id,predicate,propertyType,target_id ,valueString ,observability,confidence ,end) );"; //unique fields are used to avoid doublons
    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
        ROS_WARN_ONCE("SQL error l225: %s", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        ROS_INFO("Opened planning table successfully\n");
    }

}

/**
 * Main function
 * @return 0
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "database_server");
    ros::NodeHandle node;


    //// SERVICES DECLARATION  /////

    ros::ServiceServer set_info_service;
    ros::ServiceServer get_info_service;
    ros::ServiceServer execute_service;
    ros::ServiceServer plot_service;
    ros::ServiceServer save_service;


    //////////////////////////////////////////////////////////////////////
    //// SERVICES INSTANCIATION  /////
    set_info_service = node.advertiseService("database_manager/set_info", set_info_db);
    get_info_service = node.advertiseService("database_manager/get_info", get_info_db);
    execute_service = node.advertiseService("database_manager/execute", execute_db);
    plot_service = node.advertiseService("database_manager/plot_facts", plot_facts_db);
    save_service = node.advertiseService("database_manager/load_save", load_save_db);

    ///////////////////////////////////////////////////////////////




    initServer();

    ToasterFactReader factRdAgent(node, "agent_monitor/factList");
    ToasterFactReader factRdArea(node, "area_manager/factList");
    ToasterFactReader factRdMove3D(node, "move3d_facts/factList");
    ToasterFactReader factRdPdg(node, "pdg/factList");
    readerAgent = &factRdAgent;
    readerArea = &factRdArea;
    readerMove3d = &factRdMove3D;
    readerPdg = &factRdPdg;


    if (node.hasParam("/database/mainAgent")) {
        node.getParam("/database/mainAgent", mainAgent);
    } else {
        mainAgent = "PR2_ROBOT";
    }

    //Get topics from params if exist
    bool activate;
    if (node.hasParam("/database/area_manager")) {
        node.getParam("/database/area_manager", activate);
        if (activate) {
            factsReaders.push_back(readerArea);
        }
    }
    if (node.hasParam("/database/agent_monitor")) {
        node.getParam("/database/agent_monitor", activate);
        if (activate) {
            factsReaders.push_back(readerAgent);
        }
    }
    if (node.hasParam("/database/move3d_facts")) {
        node.getParam("/database/move3d_facts", activate);
        if (activate) {
            factsReaders.push_back(readerMove3d);
        }
    }
    if (node.hasParam("/database/pdg_facts")) {
        node.getParam("/database/pdg_facts", activate);
        if (activate) {
            factsReaders.push_back(readerPdg);
        }
    }


    ros::Rate loop_rate(30);

    while (ros::ok()) {
        //std::cout << "\n\n\n";
        //db.readDb();
        update_world_states(node, factsReaders);
        conceptual_perspective_taking();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}



