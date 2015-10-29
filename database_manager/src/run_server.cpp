#include <stdio.h>
#include <sqlite3.h> 
#include <tinyxml.h>
#include <sstream>

#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"

#include "toaster_msgs/GetProperties.h"
#include "toaster_msgs/GetPropertyValue.h"
#include "toaster_msgs/ExecuteSQL.h"
#include "toaster_msgs/Property.h"
#include "toaster_msgs/GetCurrentFacts.h"
#include "toaster_msgs/GetPassedFacts.h"
#include "toaster_msgs/GetAgents.h"
#include "toaster_msgs/GetId.h"
#include "toaster_msgs/GetIdValue.h"
#include "toaster_msgs/Agent.h"
#include "toaster_msgs/ToasterFactReader.h"
#include "toaster_msgs/ToasterObjectReader.h"
#include "toaster_msgs/AddEvent.h"
#include "toaster_msgs/GetEvents.h"
#include "toaster_msgs/GetEventValue.h"
#include "toaster_msgs/GetOntologies.h"
#include "toaster_msgs/GetOntologyValues.h"
#include "toaster_msgs/GetOntologyLeaves.h"
#include "toaster_msgs/Event.h"
#include "toaster_msgs/Ontology.h"
#include "toaster_msgs/Fact.h"
#include "toaster_msgs/Id.h"
#include "toaster_msgs/FactList.h"
#include "toaster_msgs/AddAgent.h"
#include "toaster_msgs/AddEntity.h"
#include "toaster_msgs/AddArea.h"
#include "toaster_msgs/AddFact.h"
#include "toaster_msgs/AddFactToAgent.h"
#include "toaster_msgs/RemoveFact.h"
#include "toaster_msgs/RemoveFactToAgent.h"
#include "toaster_msgs/GetFacts.h"
#include "toaster_msgs/GetFactValue.h"



std::vector<std::string> agentList;

std::vector<toaster_msgs::Fact> myFactList;
std::vector<toaster_msgs::Property> myPropertyList;
std::vector<toaster_msgs::Id> myEntityList;
std::vector<toaster_msgs::Event> myEventList;
std::vector<toaster_msgs::Ontology> myOntologyList;

std::vector<std::string> myStringList;
std::vector<toaster_msgs::Fact> previousFactsState;

class run_server {
    //attributes
    ros::NodeHandle node;

    int nb_agents;


    //// SERVICES DECLARATION  /////

    ros::ServiceServer add_entity_service;
    ros::ServiceServer add_fact_service;
    ros::ServiceServer add_fact_to_agent_service;
    ros::ServiceServer remove_fact_to_agent_service;
    ros::ServiceServer remove_fact_service;

    //robot facts services
    ros::ServiceServer get_facts_service;
    ros::ServiceServer get_fact_value_service;

    ros::ServiceServer get_current_facts_service;
    ros::ServiceServer get_passed_facts_service;

    //agents' facts services
    ros::ServiceServer get_facts_from_agent_service;
    ros::ServiceServer get_fact_value_from_agent_service;

    ros::ServiceServer get_current_facts_from_agent_service;
    ros::ServiceServer get_passed_facts_from_agent_service;

    //other services
    ros::ServiceServer execute_SQL_service;

    ros::ServiceServer get_properties;
    ros::ServiceServer get_property_value;

    ros::ServiceServer get_agents_service;
    ros::ServiceServer get_id_service;
    ros::ServiceServer get_id_value_service;

    ros::ServiceServer add_event_service;
    ros::ServiceServer get_events_service;
    ros::ServiceServer get_event_value_service;

    ros::ServiceServer get_ontologies_service;
    ros::ServiceServer get_ontology_values_service;
    ros::ServiceServer get_ontology_leaves_service;

    //sqlite database's pointer
    sqlite3 *database;

    /// FACTS READER ///
    ToasterFactReader* factRdSpark_;
    ToasterFactReader* factRdPdg_;
    ToasterFactReader* factRdArea_;
    ToasterFactReader* factRdAM_;

public:

    //construtor

    run_server() {
        ros::NodeHandle node;


        ToasterFactReader factRdSpark(node, "spark/factList");
        ToasterFactReader factRdPdg(node, "pdg/factList");
        ToasterFactReader factRdArea(node, "area_manager/factList");
        ToasterFactReader factRdAM(node, "agent_monitor/factList");

        factRdSpark_ = &factRdSpark;
        factRdPdg_ = &factRdPdg;
        factRdArea_ = &factRdArea;
        factRdAM_ = &factRdAM;


        //////////////////////////////////////////////////////////////////////
        //// SERVICES INSTANCIATION  /////
        //facts services
        add_entity_service = node.advertiseService("database/add_entity", &run_server::add_entity_db, this);

        add_fact_service = node.advertiseService("database/add_fact", &run_server::add_fact_db, this);
        add_fact_to_agent_service = node.advertiseService("database/add_fact_to_agent", &run_server::add_fact_to_agent_db, this);

        remove_fact_service = node.advertiseService("database/remove_fact", &run_server::remove_fact_db, this);
        remove_fact_to_agent_service = node.advertiseService("database/remove_fact_to_agent", &run_server::remove_fact_to_agent_db, this);

        //robot facts services
        get_facts_service = node.advertiseService("database/get_facts", &run_server::get_facts_db, this);
        get_fact_value_service = node.advertiseService("database/get_fact_value", &run_server::get_fact_value_db, this);

        get_current_facts_service = node.advertiseService("database/get_current_facts", &run_server::get_current_facts_db, this);
        get_passed_facts_service = node.advertiseService("database/get_passed_facts", &run_server::get_passed_facts_db, this);

        //agents' facts services
        get_facts_from_agent_service = node.advertiseService("database/get_facts_from_agent", &run_server::get_facts_from_agent_db, this);
        get_fact_value_from_agent_service = node.advertiseService("database/get_fact_value_from_agent", &run_server::get_fact_value_from_agent_db, this);

        get_current_facts_from_agent_service = node.advertiseService("database/get_current_facts_from_agent", &run_server::get_current_facts_from_agent_db, this);
        get_passed_facts_from_agent_service = node.advertiseService("database/get_passed_facts_from_agent", &run_server::get_passed_facts_from_agent_db, this);

        //other services
        execute_SQL_service = node.advertiseService("database/execute_SQL", &run_server::execute_SQL_db, this);

        get_properties = node.advertiseService("database/get_properties", &run_server::get_properties_db, this);
        get_property_value = node.advertiseService("database/get_property_value", &run_server::get_property_value_db, this);

        get_agents_service = node.advertiseService("database/get_agents", &run_server::get_agents_db, this);
        get_id_service = node.advertiseService("database/get_id", &run_server::get_id_db, this);
        get_id_value_service = node.advertiseService("database/get_id_value", &run_server::get_id_value_db, this);


        //event services
        add_event_service = node.advertiseService("database/add_event", &run_server::add_event_db, this);
        get_events_service = node.advertiseService("database/get_events", &run_server::get_events_db, this);
        get_event_value_service = node.advertiseService("database/get_event_value", &run_server::get_event_value_db, this);


        //ontology services
        get_ontologies_service = node.advertiseService("database/get_ontologies", &run_server::get_ontologies_db, this);
        get_ontology_values_service = node.advertiseService("database/get_ontology_values", &run_server::get_ontology_values_db, this);
        get_ontology_leaves_service = node.advertiseService("database/get_ontology_leaves", &run_server::get_ontology_leaves_db, this);


        ///////////////////////////////////////////////////////////////



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
            ROS_INFO("Opened id table successfully\n");
            launchIdList(); //get id informations form xml file
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

    }

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
    static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
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
    static int get_facts_callback(void *NotUsed, int argc, char **argv, char **azColName) {
        int nb_el = 10;

        ROS_INFO("---");

        for (int i = 0; i < argc / nb_el; i++) {
            //ROS_INFO("%s = %s", azColName[i*nb_el], argv[i*nb_el] ? argv[i*nb_el] : "NULL"); //needed to debug

            toaster_msgs::Fact f;
            f.subjectId = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
            f.property = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
            f.targetId = argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL";
            f.valueType = (bool)(argv[(i) * nb_el + 3] ? argv[(i) * nb_el + 3] : "NULL");
            f.stringValue = argv[(i) * nb_el + 4] ? argv[i * nb_el + 4] : "NULL";
            f.doubleValue = atof(argv[i * nb_el + 5] ? argv[i * nb_el + 5] : "NULL");
            f.factObservability = atof(argv[i * nb_el + 6] ? argv[i * nb_el + 6] : "NULL");
            f.confidence = atof(argv[i * nb_el + 7] ? argv[i * nb_el + 7] : "NULL");
            f.timeStart = atof(argv[i * nb_el + 8] ? argv[i * nb_el + 8] : "NULL");
            f.timeEnd = atof(argv[i * nb_el + 9] ? argv[i * nb_el + 9] : "NULL");

            myFactList.push_back(f);
        }
        ROS_INFO("---");
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
    static int property_callback(void *NotUsed, int argc, char **argv, char **azColName) {
        int i;
        int nb_el = 5;

        ROS_INFO("---");

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
        ROS_INFO("---");
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
    static int id_callback(void *NotUsed, int argc, char **argv, char **azColName) {
        int nb_el = 4;

        ROS_INFO("---");

        for (int i = 0; i < argc / nb_el; i++) {
            // ROS_INFO("%s = %s", azColName[i], argv[i] ? argv[i] : "NULL"); //if needed to debug

            toaster_msgs::Id id;
            id.id = argv[i * nb_el] ? argv[i * nb_el] : "NULL";
            id.name = argv[i * nb_el] ? argv[i * nb_el] : "NULL";
            id.type = argv[i * nb_el + 2] ? argv[i * nb_el + 2] : "NULL";
            id.owner_id = argv[i * nb_el + 3] ? argv[i * nb_el + 3] : "NULL";

            myEntityList.push_back(id);
        }
        ROS_INFO("---");
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
    static int event_callback(void *NotUsed, int argc, char **argv, char **azColName) {
        int nb_el = 6;

        ROS_INFO("---");

        for (int i = 0; i < argc / nb_el; i++) {
            //ROS_INFO("%s = %s", azColName[i*nb_el], argv[i*nb_el] ? argv[i*nb_el] : "NULL"); //if needed to debug

            toaster_msgs::Event e;
            e.subjectId = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
            e.property = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
            e.targetId = argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL";
            e.factObservability = atof(argv[i * nb_el + 3] ? argv[i * nb_el + 3] : "NULL");
            e.confidence = atof(argv[i * nb_el + 4] ? argv[i * nb_el + 4] : "NULL");
            e.time = atof(argv[i * nb_el + 5] ? argv[i * nb_el + 5] : "NULL");

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
    static int ontology_callback(void *NotUsed, int argc, char **argv, char **azColName) {
        int nb_el = 3;

        ROS_INFO("---");

        for (int i = 0; i < argc / nb_el; i++) {
            //ROS_INFO("%s = %s", azColName[i*nb_el], argv[i*nb_el] ? argv[i*nb_el] : "NULL"); //if needed to debug

            toaster_msgs::Ontology o;
            o.entityClass = argv[(i) * nb_el + 0] ? argv[(i) * nb_el + 0] : "NULL";
            o.individual = argv[(i) * nb_el + 1] ? argv[(i) * nb_el + 1] : "NULL";
            o.instantiated = (bool)(argv[(i) * nb_el + 2] ? argv[(i) * nb_el + 2] : "NULL");


            myOntologyList.push_back(o);
        }
        ROS_INFO("---");
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
    static int sql_callback(void *NotUsed, int argc, char **argv, char **azColName) {
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
    void launchIdList() {
        char *zErrMsg = 0;
        std::string sql;
        std::stringstream s;

        s << ros::package::getPath("database_manager") << "/database/id_list.xml"; //load xml file
        TiXmlDocument static_property(s.str());
        //TiXmlDocument static_property("src/database_manager/database/id_list.xml"); //load xml file

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
                            "target_id        		     	unsigned long," +
                            "valueType			  	        string," +
                            "valueString       	        	string," +
                            "valueDouble       	        	double," +
                            "observability   		   		unsinged short," +
                            "confidence   		        	unsigned short," +
                            "start   				        int," +
                            "end 					        int," +
                            "unique (subject_id,predicate,target_id ,valueString ,observability,confidence ,end) );"; //unique fields are used to avoid doublons

                    if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                        ROS_WARN_ONCE("SQL error l511: %s", zErrMsg);
                        sqlite3_free(zErrMsg);
                    } else {
                        ROS_INFO("Opened fact table successfully\n");
                    }

                    //and one memory table
                    sql = (std::string)"CREATE TABLE memory_table_" + (std::string)elem->Attribute("id") + " (" +
                            "subject_id 				 unsigned long," +
                            "predicate         	         string," +
                            "target_id        		     unsigned long," +
                            "valueType			  	     string," +
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
                        ROS_INFO("Opened memory table successfully\n");
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




    ///////////////////////////////////////////////////////
    /////debug function/////

    /**
     * in consol read function for debug or anything
     * @return void
     */
    void readDb() {
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


    ///////////////////////////////////////////////////////////////
    ////////basic functions/////

    /**
     * Add an agent to the agent table	
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool add_entity_db(toaster_msgs::AddEntity::Request &req, toaster_msgs::AddEntity::Response &res) {
        ROS_INFO("add_entity");

        std::string sql;
        std::string sql2;
        char *zErrMsg = 0;

        //add the agent in agents_table
        sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('" + (std::string)req.id + "', '" + (std::string)req.name + "' ,'" + (std::string)req.type
                + "' ,'" + (std::string)req.ownerId + "');";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        }

        //now if this entity need facts table we create it
        zErrMsg = 0;

        if (req.type == "human" || req.type == "robot") {
            ROS_WARN("%d", req.type.compare("human"));
            sql = (std::string)"CREATE TABLE fact_table_" + (std::string)req.id + " (" +
                    "subject_id 				 unsigned long," +
                    "predicate         	            string," +
                    "target_id        		     unsigned long," +
                    "valueType			  	        	string," +
                    "valueString       	        	string," +
                    "valueDouble       	        	double," +
                    "observability   		    unsinged short," +
                    "confidence   		        unsigned short," +
                    "start   				               int," +
                    "end 					               int," +
                    "unique (subject_id,predicate,target_id ,valueString ,observability,confidence ,end) );";


            //and his memory table

            sql2 = (std::string)"CREATE TABLE memory_table_" + (std::string)req.id + " (" +
                    "subject_id 				 unsigned long," +
                    "predicate         	            string," +
                    "target_id        		     unsigned long," +
                    "valueType			  	        	string," +
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

                sql = (std::string)"DELETE from id_table where id=" + (std::string)req.id + " and name='" + (std::string)req.name + "'; SELECT * from id_table";

                zErrMsg = 0;

                if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                    ROS_INFO("SQL error3: %s\n", zErrMsg);
                }
            } else {
                agentList.push_back((std::string)req.id);
                nb_agents++;
            }
        }
        sqlite3_free(zErrMsg);
        ROS_INFO("Entity successfully added\n");
        return true;
    }

    /**
     * Add a fact to main fact table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool add_fact_db(toaster_msgs::AddFact::Request &req, toaster_msgs::AddFact::Response &res) {
        ROS_INFO("add_fact");

        std::string sql;
        char *zErrMsg = 0;

        //we first check if there is already such fact in db
        sql = (std::string)"SELECT * from fact_table_" + agentList[0]
                + " where subject_id='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId)
                + "' and predicate='" + (std::string)req.fact.property
                + "' and target_id='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId)
                + "' ;";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Redondant fact check\n");
        }

        zErrMsg = 0;

        // update fact if allready here
        if (!myFactList.empty()) {
            sql = (std::string)"UPDATE fact_table_" + agentList[0]
                    + " set valueString='" + (std::string)req.fact.stringValue
                    + "' , valueDouble=" + boost::lexical_cast<std::string>(req.fact.doubleValue)
                    + " where subject_id='" + (std::string)req.fact.subjectId + boost::lexical_cast<std::string>(req.fact.subjectOwnerId)
                    + "' and predicate='" + (std::string)req.fact.property
                    + "' and target_id='" + (std::string)req.fact.targetId + boost::lexical_cast<std::string>(req.fact.targetOwnerId)
                    + "';";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error2 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully updated\n");
            }
        } else //else add fact to the table
        {
            zErrMsg = 0;

            sql = (std::string)"INSERT INTO fact_table_" + agentList[0]
                    + " (subject_id,predicate,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) + "','"
                    + (std::string)req.fact.property + "','"
                    + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "','"
                    + (std::string)req.fact.stringValue + "','"
                    + (std::string)req.fact.stringValue + "',"
                    + boost::lexical_cast<std::string>(req.fact.doubleValue) + ","
                    + boost::lexical_cast<std::string>(req.fact.factObservability) + ","
                    + boost::lexical_cast<std::string>(req.fact.confidence) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ",0)";


            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully added\n");
            }

            //a new event
            sql = (std::string)"INSERT INTO events_table (subject_id,predicate,target_id,observability,confidence,time) VALUES ('"
                    + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) + "','"
                    + (std::string)req.fact.property + "','"
                    + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "',"
                    + boost::lexical_cast<std::string>(req.fact.factObservability) + ","
                    + boost::lexical_cast<std::string>(req.fact.confidence) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ")";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error4 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Event successfully added\n");
            }


            //and we add into id_table some new unknown entity (id is unique so there should not be duplicates)
            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)req.fact.subjectId + "', '" + (std::string)req.fact.subjectId + "' , '' " + ",'" + (std::string)req.fact.subjectOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Subject object successfully added\n");
            }


            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)req.fact.targetId + "', '" + (std::string)req.fact.targetId + "' , 'object' " + ",'" + (std::string)req.fact.targetOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Target object successfully added\n");
            }

        }

        myFactList = std::vector<toaster_msgs::Fact>();

        return true;
    }

    /**
     * Remove a fact from main fact table	
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool remove_fact_db(toaster_msgs::RemoveFact::Request &req, toaster_msgs::RemoveFact::Response &res) {
        ROS_INFO("remove_fact");

        std::string sql;
        char *zErrMsg = 0;
        const char* data = "Callback function called";

        //we first get fact informations form fact table
        sql = (std::string)"SELECT * from fact_table_" + agentList[0] +
                " where subject_id ='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) +
                "' and predicate ='" + (std::string)req.fact.property +
                "' and target_id ='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "'; ";


        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg)) {
            ROS_INFO("SQL error1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Informations successfully saved\n");
        }

        if (myFactList.empty()) {
            ROS_INFO("No such fact in database\n");
            return true;
        } else {
            //if there is such fact then we can remove it
            sql = (std::string)"DELETE from fact_table_" + agentList[0] +
                    " where subject_id ='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) +
                    "' and predicate ='" + (std::string)req.fact.property +
                    "' and target_id='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "';";

            if (sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg)) {
                ROS_INFO("SQL error2 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully removed\n");
            }
            //finally we add it in memory table
            sql = (std::string)"INSERT into memory_table_" + agentList[0]
                    + " (subject_id,predicate,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(myFactList[0].subjectId) + "','"
                    + (std::string)myFactList[0].property + "','"
                    + boost::lexical_cast<std::string>(myFactList[0].targetId) + "','"
                    + (std::string)myFactList[0].stringValue + "','"
                    + (std::string)myFactList[0].stringValue + "',"
                    + boost::lexical_cast<std::string>(myFactList[0].doubleValue) + ","
                    + boost::lexical_cast<std::string>(myFactList[0].factObservability) + ","
                    + boost::lexical_cast<std::string>(myFactList[0].confidence) + ","
                    + boost::lexical_cast<std::string>(myFactList[0].timeStart) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ")";


            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully added to memory table\n");
            }

            //and add a new event
            sql = (std::string)"INSERT INTO events_table (subject_id,predicate,target_id,observability,confidence,time) VALUES ('"
                    + boost::lexical_cast<std::string>(myFactList[0].subjectId) + "','!"
                    + (std::string)myFactList[0].property + "','"
                    + boost::lexical_cast<std::string>(myFactList[0].targetId) + "',"
                    + boost::lexical_cast<std::string>(myFactList[0].factObservability) + ","
                    + boost::lexical_cast<std::string>(myFactList[0].confidence) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ")";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error4 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Event successfully added\n");
            }

        }
        myFactList = std::vector<toaster_msgs::Fact>();
        return true;
    }

    /**
     * Add a fact in targeted agent's fact table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool add_fact_to_agent_db(toaster_msgs::AddFactToAgent::Request &req, toaster_msgs::AddFactToAgent::Response &res) {
        ROS_INFO("add_fact_to_agent");

        std::string sql;
        char *zErrMsg = 0;


        //we first check if there is allready a such fact in db
        sql = (std::string)"SELECT * from fact_table_" + (std::string)req.agentId +
                +" where subject_id='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId)
                + "' and predicate='" + (std::string)req.fact.property
                + "'and target_id='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId)
                + "';";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Redondant fact check\n");
        }

        zErrMsg = 0;

        // update fact if allready here
        if (!myFactList.empty()) {
            sql = (std::string)"UPDATE fact_table_" + (std::string)req.agentId +
                    +" set valueString='" + (std::string)req.fact.stringValue
                    + "' , valueDouble=" + boost::lexical_cast<std::string>(req.fact.doubleValue)
                    + " where subject_id='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId)
                    + "' and predicate='" + (std::string)req.fact.property
                    + "' and target_id='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId)
                    + "'; SELECT * from fact_table_" + (std::string)req.agentId + ";";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error2 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully updated\n");
            }
        } else //else 
        {
            sql = (std::string)"INSERT INTO fact_table_" + (std::string)req.agentId + " (subject_id,predicate,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                    + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) + "','"
                    + (std::string)req.fact.property + "','"
                    + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "','"
                    + (std::string)req.fact.stringValue + "','"
                    + (std::string)req.fact.stringValue + "',"
                    + boost::lexical_cast<std::string>(req.fact.doubleValue) + ","
                    + boost::lexical_cast<std::string>(req.fact.factObservability) + ","
                    + boost::lexical_cast<std::string>(req.fact.confidence) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ",0);";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error3 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Fact successfully added to agent in table\n");
            }


            //add a new event
            sql = (std::string)"INSERT INTO events_table (subject_id,predicate,target_id,observability,confidence,time) VALUES ('"
                    + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) + "','"
                    + (std::string)req.fact.property + "','"
                    + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) + "',"
                    + boost::lexical_cast<std::string>(req.fact.factObservability) + ","
                    + boost::lexical_cast<std::string>(req.fact.confidence) + ","
                    + boost::lexical_cast<std::string>(req.fact.time) + ")";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                ROS_INFO("SQL error4 : %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Event successfully added\n");
            }

            //and we add into id_table some new unknown entity (id is unique so there should not be duplicates)
            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)req.fact.subjectId + "', '" + (std::string)req.fact.subjectId + "' , '' " + ",'" + (std::string)req.fact.subjectOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Subject object successfully added\n");
            }


            sql = (std::string)"INSERT INTO id_table (id,name, type, owner_id) VALUES ('"
                    + (std::string)req.fact.targetId + "', '" + (std::string)req.fact.targetId + "' , 'object' " + ",'" + (std::string)req.fact.targetOwnerId + "');";

            if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
                sqlite3_free(zErrMsg);
            } else {
                ROS_INFO("Target object successfully added\n");
            }
        }
        myFactList = std::vector<toaster_msgs::Fact>();

        return true;
    }

    /**
     * Remove a fact form targeted agent's fact table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool remove_fact_to_agent_db(toaster_msgs::AddFactToAgent::Request &req, toaster_msgs::AddFactToAgent::Response &res) {
        ROS_INFO("remove_fact_to_agent");



        std::string sql;
        char *zErrMsg = 0;
        const char* data = "Callback function called";

        //first we get all information of the fact from fact table
        sql = (std::string)"SELECT * from fact_table_" + (std::string)req.agentId +
                " where subject_id ='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) +
                "' and predicate ='" + (std::string)req.fact.property +
                "' and target_id ='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) +
                "' and valueString  ='" + (std::string)req.fact.stringValue +
                "' and valueDouble =" + boost::lexical_cast<std::string>(req.fact.doubleValue) +
                " and observability =" + boost::lexical_cast<std::string>(req.fact.factObservability) +
                ";";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg)) {
            ROS_INFO("SQL error : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Time successfully saved\n");
        }

        //then we can delete it
        sql = "DELETE from fact_table_" + (std::string)req.agentId +
                " where subject_id ='" + boost::lexical_cast<std::string>(req.fact.subjectId) + boost::lexical_cast<std::string>(req.fact.subjectOwnerId) +
                "' and predicate ='" + (std::string)req.fact.property +
                "' and target_id ='" + boost::lexical_cast<std::string>(req.fact.targetId) + boost::lexical_cast<std::string>(req.fact.targetOwnerId) +
                "' and valueString  ='" + (std::string)req.fact.stringValue +
                "' and valueDouble =" + boost::lexical_cast<std::string>(req.fact.doubleValue) +
                " and observability =" + boost::lexical_cast<std::string>(req.fact.factObservability) +
                " ;";

        if (sqlite3_exec(database, sql.c_str(), callback, (void*) data, &zErrMsg)) {
            ROS_INFO("SQL error : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Fact successfully removed\n");
        }

        //finally we add it into memory table
        sql = (std::string)"INSERT into memory_table_" + (std::string)req.agentId + " (subject_id,predicate,target_id,valueType,valueString,valueDouble,observability,confidence,start,end) VALUES ('"
                + boost::lexical_cast<std::string>(myFactList[0].subjectId) + "','"
                + (std::string)myFactList[0].property + "','"
                + boost::lexical_cast<std::string>(myFactList[0].targetId) + "','"
                + (std::string)myFactList[0].stringValue + "','"
                + (std::string)myFactList[0].stringValue + "',"
                + boost::lexical_cast<std::string>(myFactList[0].doubleValue) + ","
                + boost::lexical_cast<std::string>(myFactList[0].factObservability) + ","
                + boost::lexical_cast<std::string>(myFactList[0].confidence) + ","
                + boost::lexical_cast<std::string>(myFactList[0].timeStart) + ","
                + boost::lexical_cast<std::string>(req.fact.time) + ")";



        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Fact successfully added to memory table\n");
        }
        //and add a new event
        sql = (std::string)"INSERT INTO events_table (subject_id,predicate,target_id,observability,confidence,time) VALUES ('"
                + boost::lexical_cast<std::string>(myFactList[0].subjectId) + "','!"
                + (std::string)myFactList[0].property + "','"
                + boost::lexical_cast<std::string>(myFactList[0].targetId) + "',"
                + boost::lexical_cast<std::string>(myFactList[0].factObservability) + ","
                + boost::lexical_cast<std::string>(myFactList[0].confidence) + ","
                + boost::lexical_cast<std::string>(req.fact.time) + ")";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error4 : %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Event successfully added\n");
        }

        myFactList = std::vector<toaster_msgs::Fact>();

        return true;
    }

    /**
     * Add an event to events table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool add_event_db(toaster_msgs::AddEvent::Request &req, toaster_msgs::AddEvent::Response &res) {
        ROS_INFO("add_event");

        std::string sql;
        char *zErrMsg = 0;

        sql = (std::string)"INSERT INTO events_table (subject_id,predicate,target_id,observability,confidence,time) VALUES ('"
                + boost::lexical_cast<std::string>(req.event.subjectId) + boost::lexical_cast<std::string>(req.event.subjectOwnerId) + "','"
                + (std::string)req.event.property + "','"
                + boost::lexical_cast<std::string>(req.event.targetId) + boost::lexical_cast<std::string>(req.event.targetOwnerId) + "',"
                + boost::lexical_cast<std::string>(req.event.factObservability) + ", "
                + boost::lexical_cast<std::string>(req.event.confidence) + ", "
                + boost::lexical_cast<std::string>(req.event.time) + ");";

        if (sqlite3_exec(database, sql.c_str(), callback, 0, &zErrMsg) != SQLITE_OK) {
            ROS_INFO("SQL error1: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("Event successfully added");
        }
        return true;
    }







    ////////////////////getting facts from robot/////////////////

    /**
     * Get all known facts (current and past)
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_facts_db(toaster_msgs::GetFacts::Request &req, toaster_msgs::GetFacts::Response &res) {
        ROS_INFO("get_facts");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from fact_table_" + agentList[0];

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1189: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Currents facts from robot obtained successfully\n");
        }

        sql = (std::string)"SELECT * from memory_table_" + agentList[0];

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1198: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from robot memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[i].subjectId = myFactList[i].subjectId;
                myFactList[i].targetId = myFactList[i].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList

        return true;
    }

    /**
     * Get values from one fact in the main fact table or memory table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_fact_value_db(toaster_msgs::GetFactValue::Request &req, toaster_msgs::GetFactValue::Response &res) {
        ROS_INFO("get_fact_value");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from fact_table_" + agentList[0]
                + " where subject_id='" + boost::lexical_cast<std::string>(req.reqFact.subjectId) + boost::lexical_cast<std::string>(req.reqFact.subjectOwnerId)
                + "' and predicate='" + boost::lexical_cast<std::string>(req.reqFact.property)
                + "' and target_id='" + boost::lexical_cast<std::string>(req.reqFact.targetId) + boost::lexical_cast<std::string>(req.reqFact.targetOwnerId) + "';";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1240: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Current fact value from robot obtained successfully\n");
        }

        zErrMsg = 0;
        sql = (std::string)"SELECT * from memory_table_" + agentList[0]
                + " where subject_id='" + boost::lexical_cast<std::string>(req.reqFact.subjectId) + boost::lexical_cast<std::string>(req.reqFact.subjectOwnerId)
                + "' and predicate='" + (std::string)req.reqFact.property
                + "' and target_id='" + boost::lexical_cast<std::string>(req.reqFact.targetId) + boost::lexical_cast<std::string>(req.reqFact.targetOwnerId) + "';";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1253: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Fact value from robot memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            myFactList[0].subjectId = myFactList[0].subjectId;
            myFactList[0].targetId = myFactList[0].targetId;
            res.resFact = myFactList[0];
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


        return true;
    }

    /**
     * Get all current known facts 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_current_facts_db(toaster_msgs::GetCurrentFacts::Request &req, toaster_msgs::GetCurrentFacts::Response &res) {
        ROS_INFO("get_currents_facts");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from fact_table_" + agentList[0];

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1291: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Currents facts from robot obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[i].subjectId = myFactList[i].subjectId;
                myFactList[i].targetId = myFactList[i].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList

        return true;
    }

    /**
     * Get all passed known facts 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_passed_facts_db(toaster_msgs::GetPassedFacts::Request &req, toaster_msgs::GetPassedFacts::Response &res) {
        ROS_INFO("get_passed_facts");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;


        sql = (std::string)"SELECT * from memory_table_" + agentList[0];

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1331: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from robot memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[i].subjectId = myFactList[i].subjectId;
                myFactList[i].targetId = myFactList[i].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList

        return true;
    }





    ////////////////getting facts from agent//////////////////////////

    /**
     * Get all facts known from an agent (current and past)
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_facts_from_agent_db(toaster_msgs::GetFacts::Request &req, toaster_msgs::GetFacts::Response &res) {
        ROS_INFO("get_facts_from_agent");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from fact_table_" + (std::string)req.agentId;

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1376: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from agent obtained successfully\n");
        }

        sql = (std::string)"SELECT * from memory_table_" + (std::string)req.agentId;

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1385: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from agent memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[i].subjectId = myFactList[i].subjectId;
                myFactList[i].targetId = myFactList[i].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


        return true;
    }

    /**
     * Get values from a known fact in agent's fact table or memory table
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_fact_value_from_agent_db(toaster_msgs::GetFactValue::Request &req, toaster_msgs::GetFactValue::Response &res) {
        ROS_INFO("get_fact_value_from_agent");


        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;



        sql = (std::string)"SELECT * from fact_table_" + (std::string)req.agentId
                + " where subject_id='" + boost::lexical_cast<std::string>(req.reqFact.subjectId) + boost::lexical_cast<std::string>(req.reqFact.subjectOwnerId)
                + "' and predicate='" + (std::string)req.reqFact.property
                + "' and target_id='" + boost::lexical_cast<std::string>(req.reqFact.targetId) + boost::lexical_cast<std::string>(req.reqFact.targetOwnerId) + "';";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1431: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Current fact value from robot obtained successfully\n");
        }

        sql = (std::string)"SELECT * from memory_table_" + (std::string)req.agentId
                + " where subject_id='" + boost::lexical_cast<std::string>(req.reqFact.subjectId) + boost::lexical_cast<std::string>(req.reqFact.subjectOwnerId)
                + "' and predicate='" + (std::string)req.reqFact.property
                + "' and target_id='" + boost::lexical_cast<std::string>(req.reqFact.targetId) + boost::lexical_cast<std::string>(req.reqFact.targetOwnerId) + "';";

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1443: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Fact value from robot memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            myFactList[0].subjectId = myFactList[0].subjectId;
            myFactList[0].targetId = myFactList[0].targetId;
            res.resFact = myFactList[0];
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


        return true;
    }

    /**
     * Get all current facts known from an agent 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_current_facts_from_agent_db(toaster_msgs::GetFacts::Request &req, toaster_msgs::GetFacts::Response &res) {
        ROS_INFO("get_currents_facts_from_agent");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from fact_table_" + (std::string)req.agentId;

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1481: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from agent obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[0].subjectId = myFactList[0].subjectId;
                myFactList[0].targetId = myFactList[0].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


        return true;
    }

    /**
     * Get all paased facts known from an agent 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_passed_facts_from_agent_db(toaster_msgs::GetFacts::Request &req, toaster_msgs::GetFacts::Response &res) {
        ROS_INFO("get_passed_facts_from_agent");


        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from memory_table_" + (std::string)req.agentId;

        if (sqlite3_exec(database, sql.c_str(), get_facts_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1522: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Facts from agent memory obtained successfully\n");
        }

        //return informations from table
        if (!myFactList.empty()) {
            for (int i = 0; i < myFactList.size(); i++) {
                myFactList[i].subjectId = myFactList[i].subjectId;
                myFactList[i].targetId = myFactList[i].targetId;
                res.resFactList.factList.push_back(myFactList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myFactList = std::vector<toaster_msgs::Fact>(); //empty myFactList


        return true;
    }












    /////////////////getting other information from database//////////////////

    /**
     * Get all static properties
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_properties_db(toaster_msgs::GetProperties::Request &req, toaster_msgs::GetProperties::Response &res) {
        ROS_INFO("get_properties");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from static_property_table;";

        if (sqlite3_exec(database, sql.c_str(), property_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1575: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Static property table obtained successfully\n");
        }

        //return informations from table
        if (!myPropertyList.empty()) {
            for (int i = 0; i < myPropertyList.size(); i++) {
                res.resProperty.push_back(myPropertyList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myPropertyList = std::vector<toaster_msgs::Property>(); //empty myPropertyList

        return true;
    }

    /**
     * Get values of a specific static property 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_property_value_db(toaster_msgs::GetPropertyValue::Request &req, toaster_msgs::GetPropertyValue::Response &res) {
        ROS_INFO("get_property_value");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from static_property_table" + " where id=" + boost::lexical_cast<std::string>(req.id);

        if (sqlite3_exec(database, sql.c_str(), property_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1612: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Static property value obtained successfully\n", req.id);
        }

        //return informations from table
        if (!myPropertyList.empty()) {
            res.resProperty = myPropertyList[0];
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myPropertyList = std::vector<toaster_msgs::Property>(); //empty myPropertyList

        return true;
    }

    /**
     * Get all agents
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_agents_db(toaster_msgs::GetId::Request &req, toaster_msgs::GetId::Response &res) {
        ROS_INFO("get_agents");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from id_table where type ='human' or type='robot';";

        if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1647: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Agents from id_table obtained successfully\n");
        }

        //return informations from table		
        if (!myEntityList.empty()) {
            for (int i = 0; i < myEntityList.size(); i++) {
                res.resId.push_back(myEntityList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

        return true;
    }

    /**
     * Get all ids
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_id_db(toaster_msgs::GetId::Request &req, toaster_msgs::GetId::Response &res) {
        ROS_INFO("get_all_id");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from id_table;";

        if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1684: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Id table obtained successfully\n");
        }

        //return informations from table		
        if (!myEntityList.empty()) {
            for (int i = 0; i < myEntityList.size(); i++) {
                res.resId.push_back(myEntityList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

        return true;
    }

    /**
     * Get values of a specific agent 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_id_value_db(toaster_msgs::GetIdValue::Request &req, toaster_msgs::GetIdValue::Response &res) {
        ROS_INFO("get_id_value");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from id_table where id='" + boost::lexical_cast<std::string>(req.id) + "' and name='" + (std::string)req.name + "';";

        if (sqlite3_exec(database, sql.c_str(), id_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1721: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Id informations obtained successfully\n");
        }

        //return informations from table
        if (!myEntityList.empty()) {
            res.resId = myEntityList[0];
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myEntityList = std::vector< toaster_msgs::Id >(); //empty myEntityList

        return true;
    }

    /**
     * Get all events
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_events_db(toaster_msgs::GetEvents::Request &req, toaster_msgs::GetEvents::Response &res) {
        ROS_INFO("get_event");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from events_table;";

        if (sqlite3_exec(database, sql.c_str(), event_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1756: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Events table obtained successfully\n");
        }

        //return informations from table
        if (!myEventList.empty()) {
            for (int i = 0; i < myEventList.size(); i++) {
                myEventList[i].subjectId = myEventList[i].subjectId;
                myEventList[i].targetId = myEventList[i].targetId;
                res.resEventList.push_back(myEventList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myEventList = std::vector<toaster_msgs::Event>(); //empty myEventList

        return true;
    }

    /**
     * Get values of a specific event
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_event_value_db(toaster_msgs::GetEventValue::Request &req, toaster_msgs::GetEventValue::Response &res) {
        ROS_INFO("get_event_value");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from events_table where subject_id='" + boost::lexical_cast<std::string>(req.reqEvent.subjectId)
                + "' and predicate='" + (std::string)req.reqEvent.property
                + "' and target_id='" + boost::lexical_cast<std::string>(req.reqEvent.targetId) + "';";

        if (sqlite3_exec(database, sql.c_str(), event_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1797: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Event obtained successfully\n");
        }

        //return informations from table
        if (!myEventList.empty()) {
            myEventList[0].subjectId = myEventList[0].subjectId;
            myEventList[0].targetId = myEventList[0].targetId;
            res.resEvent = myEventList[0];
            res.boolAnswer = true;
        } else {

            res.boolAnswer = false;
        }

        myEventList = std::vector<toaster_msgs::Event>(); //empty myEventList

        return true;
    }

    /**
     * Get all ontologies
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_ontologies_db(toaster_msgs::GetOntologies::Request &req, toaster_msgs::GetOntologies::Response &res) {
        ROS_INFO("get_ontologies");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from ontology_table;";

        if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1835: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Ontology table obtained successfully\n");
        }

        //return informations from table
        if (!myOntologyList.empty()) {
            for (int i = 0; i < myOntologyList.size(); i++) {
                res.resOntology.push_back(myOntologyList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

        return true;
    }

    /**
     * Get values of a all leaves of ontology table starting from a entityClass field
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_ontology_leaves_db(toaster_msgs::GetOntologyLeaves::Request &req, toaster_msgs::GetOntologyLeaves::Response &res) {
        ROS_INFO("get_ontology_leaves");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string) "WITH tree(g,ind,inst)  AS ( "
                + "SELECT entityClass ,individual, instantiated  FROM ontology_table WHERE entityClass ='" + (std::string)req.entityClass
                + "' UNION ALL"
                + " SELECT entityClass ,individual, instantiated  FROM ontology_table o"
                + " INNER JOIN tree t"
                + " ON t.ind = o.entityClass)"
                + "SELECT * FROM tree where inst = 'true';";

        if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1878: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Ontology leaves obtained successfully\n");
        }

        //return informations from table
        if (!myOntologyList.empty()) {
            for (int i = 0; i < myOntologyList.size(); i++) {
                res.resOntology.push_back(myOntologyList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

        return true;
    }

    /**
     * Get values of a specific ontology
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool get_ontology_values_db(toaster_msgs::GetOntologyValues::Request &req, toaster_msgs::GetOntologyValues::Response &res) {
        ROS_INFO("get_ontology_values");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = (std::string)"SELECT * from ontology_table where entityClass='" + (std::string)req.entityClass + "';";

        if (sqlite3_exec(database, sql.c_str(), ontology_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1915: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            fprintf(stdout, "Ontology values obtained successfully\n");
        }

        //return informations from table
        if (!myOntologyList.empty()) {
            for (int i = 0; i < myOntologyList.size(); i++) {
                res.resOntology.push_back(myOntologyList[i]);
            }
            res.boolAnswer = true;
        } else {
            res.boolAnswer = false;
        }

        myOntologyList = std::vector<toaster_msgs::Ontology>(); //empty myOntologyList

        return true;
    }








    /////////////////////SQl request function/////////////

    /**
     * Execute in selected table the SQl request casted in the Request.order field 
     * @param reference to request
     * @param reference to response
     * @return true 
     */
    bool execute_SQL_db(toaster_msgs::ExecuteSQL::Request &req, toaster_msgs::ExecuteSQL::Response &res) {
        ROS_INFO("sql order");

        char *zErrMsg = 0;
        const char* data = "Callback function called";
        std::string sql;

        sql = req.order;

        if (sqlite3_exec(database, sql.c_str(), sql_callback, (void*) data, &zErrMsg) != SQLITE_OK) {
            fprintf(stderr, "SQL error l1961: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            ROS_INFO("SQL order obtained successfully\n");
        }

        //return informations from table
        for (int i = 0; i < myStringList.size(); i++) {
            res.results.push_back(myStringList[i]);
        }
        res.boolAnswer = true;

        myStringList = std::vector<std::string>(); //empty myStringList

        return true;
    }











    ///////////////////////////////////////////////////////////
    // UPDATE WORLD STATE ////////////
    //////////////////////////////////////////////////////////

    void update_world_states() {
        /**************************/
        /* World State management */
        /**************************/

        //We get the new state
        std::vector<toaster_msgs::Fact> newState;
        newState.insert(newState.end(), this->factRdArea_->lastMsgFact.factList.begin(), this->factRdArea_->lastMsgFact.factList.end());
        newState.insert(newState.end(), this->factRdSpark_->lastMsgFact.factList.begin(), this->factRdSpark_->lastMsgFact.factList.end());
        newState.insert(newState.end(), this->factRdPdg_->lastMsgFact.factList.begin(), this->factRdPdg_->lastMsgFact.factList.end());
        newState.insert(newState.end(), this->factRdAM_->lastMsgFact.factList.begin(), this->factRdAM_->lastMsgFact.factList.end());


        //If update, make modification to current db:
        bool removedFact = true;


        // Is there new facts that were not there before?
        // TODO: don't use a request but internal function!
        ros::ServiceClient AddFactClient = node.serviceClient<toaster_msgs::AddFact>("database/add_fact");
        toaster_msgs::AddFact srvAdd;
        ros::ServiceClient RemoveFactClient = node.serviceClient<toaster_msgs::RemoveFact>("database/remove_fact");
        toaster_msgs::RemoveFact srvRm;


        for (unsigned int i = 0; i < newState.size(); i++) {
            srvAdd.request.fact = newState[i];
            AddFactClient.call(srvAdd);
            // This function also takes care of events table.
            // It also avoid double.

            
        }


        for (int j = 0; j < previousFactsState.size(); ++j) {
            for (int i = 0; i < newState.size(); ++i) {
                if ((previousFactsState[j].subjectId == newState[i].subjectId)
                        && (previousFactsState[j].property == newState[i].property)
                        && (previousFactsState[j].propertyType == newState[i].propertyType)
                        && (previousFactsState[j].subProperty == newState[i].subProperty)
                        && (previousFactsState[j].subjectOwnerId == newState[i].subjectOwnerId)
                        && (previousFactsState[j].targetId == newState[i].targetId)
                        && (previousFactsState[j].targetOwnerId == newState[i].targetOwnerId)) {
                    removedFact = false;
                    break;
                } else
                    continue;
            }
            // Remove facts that are no longer there
            if (removedFact) {
                srvRm.request.fact = previousFactsState[j];
                RemoveFactClient.call(srvRm);
                //This will also handle memory and events.
            }
            removedFact = true;
        }
        previousFactsState = newState;
    }

    // This is done in add / remove fact
    /*void conceptual_perspective_taking() {

        ros::ServiceClient GetFactsClient = node.serviceClient<toaster_msgs::GetFacts>("database_get_facts");
        ros::ServiceClient AddFactToAgentClient = node.serviceClient<toaster_msgs::AddFactToAgent>("database_add_fact_to_agent");
        toaster_msgs::GetFacts srv;
        toaster_msgs::AddFactToAgent srv2;

        GetFactsClient.call(srv);


        for (int y = 0; y < srv.response.resFactList.factList.size(); y++) {
            if (srv.response.resFactList.factList[y].factObservability > 0.0) {
                srv2.request.fact = srv.response.resFactList.factList[y];
                srv2.request.agentId = srv.response.resFactList.factList[y].targetId;
                AddFactToAgentClient.call(srv2);

                srv2 = toaster_msgs::AddFactToAgent();
            }
        }
    }*/


};

/**
 * Main function
 * @return 0
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "run_server");

    run_server rs = run_server();


    ros::Rate loop_rate(2);

    while (ros::ok()) {
        std::cout << "\n\n\n";
        rs.readDb();
        //rs.update_world_state();
        //rs.conceptual_perspective_taking();
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}