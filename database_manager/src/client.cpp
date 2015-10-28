#include "ros/ros.h"
#include <cstdlib>
#include "std_msgs/String.h"
#include "toaster_msgs/Event.h"
#include "toaster_msgs/Ontology.h"
#include "toaster_msgs/AddAgent.h"
#include "toaster_msgs/AddEntity.h"
#include "toaster_msgs/AddArea.h"
#include "toaster_msgs/AddFact.h"
#include "toaster_msgs/RemoveFact.h"
#include "toaster_msgs/Object.h"
#include "toaster_msgs/Area.h"
#include "toaster_msgs/AreaList.h"
#include "geometry_msgs/Point.h"
#include "toaster_msgs/ObjectList.h"
#include "toaster_msgs/Human.h"
#include "toaster_msgs/AddFactToAgent.h"
#include "toaster_msgs/RemoveFactToAgent.h"
#include "toaster_msgs/GetFacts.h"
#include "toaster_msgs/GetFactValue.h"
#include "toaster_msgs/GetProperties.h"
#include "toaster_msgs/GetPropertyValue.h"
#include "toaster_msgs/ExecuteSQL.h"
#include "toaster_msgs/Property.h"
#include <inttypes.h>

#include "toaster_msgs/GetAgents.h"
#include "toaster_msgs/GetId.h"
#include "toaster_msgs/GetIdValue.h"

#include "toaster_msgs/GetCurrentFacts.h"
#include "toaster_msgs/GetPassedFacts.h"

#include "toaster_msgs/AddEvent.h"
#include "toaster_msgs/GetEvents.h"
#include "toaster_msgs/GetEventValue.h"

#include "toaster_msgs/GetOntologies.h"
#include "toaster_msgs/GetOntologyValues.h"
#include "toaster_msgs/GetOntologyLeaves.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
 
    ROS_INFO("usage: add");
 
 

  ros::NodeHandle ns;
  ros::ServiceClient client1 = ns.serviceClient<toaster_msgs::AddEntity>("database/add_entity");
  ros::ServiceClient client2 = ns.serviceClient<toaster_msgs::AddArea>("database/add_area");
  ros::ServiceClient client3 = ns.serviceClient<toaster_msgs::AddFact>("database/add_fact");
  ros::ServiceClient client4 = ns.serviceClient<toaster_msgs::RemoveFact>("database/remove_fact");
  
  ros::ServiceClient client5 = ns.serviceClient<toaster_msgs::AddFactToAgent>("database/add_fact_to_agent");
  ros::ServiceClient client6 = ns.serviceClient<toaster_msgs::RemoveFactToAgent>("database/remove_fact_to_agent");
  
  
  ros::ServiceClient client7 = ns.serviceClient<toaster_msgs::GetFacts>("database/get_facts");
  ros::ServiceClient client8 = ns.serviceClient<toaster_msgs::GetFactValue>("database/get_fact_value");
  ros::ServiceClient client9 = ns.serviceClient<toaster_msgs::GetFacts>("database/get_facts_from_agent");
  ros::ServiceClient client10= ns.serviceClient<toaster_msgs::GetFactValue>("database/get_fact_value_from_agent");
  
  ros::ServiceClient client11 = ns.serviceClient<toaster_msgs::GetProperties>("database/get_properties");
  ros::ServiceClient client12= ns.serviceClient<toaster_msgs::GetPropertyValue>("database/get_property_value");
  
  ros::ServiceClient client13= ns.serviceClient<toaster_msgs::ExecuteSQL>("database/execute_SQL");

  ros::ServiceClient client14 = ns.serviceClient<toaster_msgs::GetAgents>("database/get_agents");
  ros::ServiceClient client15= ns.serviceClient<toaster_msgs::GetIdValue>("database/get_id_value");  
  
  
  ros::ServiceClient client16 = ns.serviceClient<toaster_msgs::GetCurrentFacts>("database/get_current_facts");
  ros::ServiceClient client17= ns.serviceClient<toaster_msgs::GetPassedFacts>("database/get_passed_facts");  
  
  ros::ServiceClient client18 = ns.serviceClient<toaster_msgs::GetCurrentFacts>("database/get_current_facts_from_agent");
  ros::ServiceClient client19= ns.serviceClient<toaster_msgs::GetPassedFacts>("database/get_passed_facts_from_agent"); 
  
  ros::ServiceClient client20= ns.serviceClient<toaster_msgs::AddEvent>("database/add_event"); 
  ros::ServiceClient client21= ns.serviceClient<toaster_msgs::GetEvents>("database/get_events");
  ros::ServiceClient client22= ns.serviceClient<toaster_msgs::GetEventValue>("database/get_event_value");
  
  ros::ServiceClient client23= ns.serviceClient<toaster_msgs::GetOntologies>("database/get_ontologies");
  ros::ServiceClient client24= ns.serviceClient<toaster_msgs::GetOntologyValues>("database/get_ontology_values");
  ros::ServiceClient client25= ns.serviceClient<toaster_msgs::GetOntologyLeaves>("database/get_ontology_leaves");
  
 
  
  
  toaster_msgs::AddEntity srv1;
  srv1.request.id = "pierre";
  srv1.request.type = "human";
  
  toaster_msgs::AddEntity srv112;
  srv112.request.id = "bol";
  srv112.request.type = "object";
  
  toaster_msgs::AddEntity srv113;
  srv113.request.id = "nao";
  srv113.request.type = "robot";
  
  toaster_msgs::AddArea srv2;
  srv2.request.myArea.id = 2;
  srv2.request.myArea.name = "patou";
  
  toaster_msgs::AddFact srv3;
  srv3.request.fact.subjectId = "cube";
  srv3.request.fact.subjectOwnerId = "(pr2)";
  srv3.request.fact.property = "is on";
  srv3.request.fact.targetId = "table";
  srv3.request.fact.targetOwnerId = "(pr2)";
  srv3.request.fact.stringValue = "y";
  srv3.request.fact.doubleValue = 14;
  srv3.request.fact.factObservability = 1;
  
  toaster_msgs::AddFact srv31;
  srv31.request.fact.subjectId = "pr2";
  srv31.request.fact.property = "see";
  srv31.request.fact.targetId = "jean";
  srv31.request.fact.stringValue = "y";
  srv31.request.fact.doubleValue = 14;
  srv31.request.fact.factObservability = 1;
  
   toaster_msgs::AddFact srv32;
  srv32.request.fact.subjectId = "pr2";
  srv32.request.fact.property = "see";
  srv32.request.fact.targetId = "jean";
  srv32.request.fact.stringValue = "z";
  srv32.request.fact.doubleValue = 187;
  srv32.request.fact.factObservability = 1;
  
  
  toaster_msgs::RemoveFact srv4;
  srv4.request.fact.subjectId = "cube";
  srv4.request.fact.subjectOwnerId = "(pr2)";
  srv4.request.fact.property = "is on";
  srv4.request.fact.targetId = "table";
  srv4.request.fact.targetOwnerId = "(pr2)";
  srv4.request.fact.stringValue = "y";
  srv4.request.fact.doubleValue = 14;
  srv4.request.fact.factObservability = 1;
  

  
  toaster_msgs::AddFactToAgent srv5;
  srv5.request.agentId = "pierre";
  srv5.request.fact.subjectId = "pierre";
  srv5.request.fact.property = "see";
  srv5.request.fact.targetId = "pr2";

  
  toaster_msgs::AddFactToAgent srv51;
  srv51.request.agentId = "pierre";
  srv51.request.fact.subjectId = "pierre";
  srv51.request.fact.property= "hold";
  srv51.request.fact.targetId = "mug";
  srv51.request.fact.targetOwnerId = "(pr2)";
  
  toaster_msgs::RemoveFactToAgent srv6;
  srv6.request.agentId = "pierre";
  srv6.request.fact.subjectId = "pierre";
  srv6.request.fact.property = "hold";
  srv6.request.fact.targetId = "mug";
  srv6.request.fact.targetOwnerId = "(pr2)";

   
  toaster_msgs::GetFacts srv7;
  
  toaster_msgs::GetFactValue srv8;
  srv8.request.reqFact.subjectId = "pr2";
  srv8.request.reqFact.property = "see";
  srv8.request.reqFact.targetId = "jean";
  
  toaster_msgs::GetFacts srv9;
   srv9.request.agentId = "pierre";
  srv9.request.reqFact.subjectId = "robot";
  
  toaster_msgs::GetFactValue srv10;
  srv10.request.agentId = "pierre";
  srv10.request.reqFact.subjectId = "pierre";
  srv10.request.reqFact.property = "hold";
  srv10.request.reqFact.targetId = "mug";
  srv10.request.reqFact.targetOwnerId = "(pr2)";
  
  
   toaster_msgs::GetProperties srv11;

	toaster_msgs::GetPropertyValue srv12;
	srv12.request.id = 1;
	
	
	toaster_msgs::ExecuteSQL srv13;
	srv13.request.order = (std::string) "SELECT subject_id from events_table join id_table on events_table.subject_id = id_table.id and id_table.type ='human';";
	
		/*
	srv13.request.order = (std::string) "WITH test(g,ind,inst)  AS ( "
    + "SELECT entityClass ,individual, instantiated  FROM ontology_table WHERE entityClass = 'viewable' "
    +" UNION ALL"
    + " SELECT entityClass ,individual, instantiated  FROM ontology_table o"
														+" INNER JOIN test t"
														+" ON t.ind = o.entityClass)"
	+ "SELECT * FROM test where inst = 'true';";
	*/
	

	//srv13.request.targettedField = "fact";

	/*
	srv13.request.table.push_back("fact_table");
	srv13.request.tableId.push_back(1);
	srv13.request.order.push_back("INNER JOIN fact_table1;");
	*/
	
	/*
	srv13.request.table.push_back("static_property_table");
	srv13.request.tableId.push_back(1);
	srv13.request.order.push_back("SELECT * from static_property_table;");
	
	srv13.request.table.push_back("agents_table");
	srv13.request.tableId.push_back(1);
	srv13.request.order.push_back("SELECT * from agents_table;");
	*/
	toaster_msgs::GetCurrentFacts srv16;
	
	toaster_msgs::GetPassedFacts srv17;
	
	toaster_msgs::GetCurrentFacts srv18;
	srv18.request.agentId = "pierre";
	
	toaster_msgs::GetPassedFacts srv19;
	srv19.request.agentId = "pierre";
	
	toaster_msgs::AddEvent srv20;
	srv20.request.event.subjectId = "paul";
	srv20.request.event.property = "start to play";
	srv20.request.event.targetId= "ps3";
	
	toaster_msgs::GetEvents srv21;
	
	toaster_msgs::GetEventValue srv22;
	srv22.request.reqEvent.subjectId = "paul";
	srv22.request.reqEvent.property = "start to play";
	srv22.request.reqEvent.targetId= "ps3";

	toaster_msgs::GetOntologies srv23;
	
	toaster_msgs::GetOntologyValues srv24;
	srv24.request.entityClass = "agent";
	
	toaster_msgs::GetOntologyLeaves srv25;
	srv25.request.entityClass = "viewable";
	
	ROS_INFO("addEntity");
  client1.call(srv1);
  client1.call(srv112);
  client1.call(srv113);
  //client2.call(srv2);
  ROS_INFO("addFact");
  client3.call(srv3);
  ROS_INFO("addAfact2");
  client3.call(srv31);
    ROS_INFO("addAfact3");
  client3.call(srv32);
  ROS_INFO("removefact");
  client4.call(srv4);
  client5.call(srv5);
  client5.call(srv51);
  client6.call(srv6);
   client5.call(srv5);
   client7.call(srv7);
   
   ROS_INFO("GetFacts");
   for(int i = 0; i<srv7.response.resFactList.factList.size(); i++)
   {
	   
		std::cout << "                                " +  (std::string)srv7.response.resFactList.factList[i].subjectId + " " + (std::string)srv7.response.resFactList.factList[i].property + " " + (std::string)srv7.response.resFactList.factList[i].targetId + "\n";
		ROS_INFO("%d ", (int)srv7.response.boolAnswer);
   }
   
   client8.call(srv8);
   
	    ROS_INFO("GetFactValue");
		std::cout <<  "                                " +  srv8.response.resFact.subjectId + " " + srv8.response.resFact.property  + " " +  srv8.response.resFact.targetId + "\n";
		ROS_INFO("%d ", (int)srv8.response.boolAnswer);
  
   client9.call(srv9);
   
    ROS_INFO("GetFactsFromAgent");
   for(int i = 0; i<srv9.response.resFactList.factList.size(); i++)
   {
		std::cout << "                                " +  srv9.response.resFactList.factList[i].subjectId + " " + srv9.response.resFactList.factList[i].property + " " + srv9.response.resFactList.factList[i].targetId + "\n";
		ROS_INFO("%d ", (int)srv9.response.boolAnswer);
   }
   
   client10.call(srv10);
   
   	    ROS_INFO("GetFactValueFromAgent");
   	   
		std::cout <<   "                                " +  srv10.response.resFact.subjectId + " " + srv10.response.resFact.property + " " + srv10.response.resFact.targetId + "\n";
		ROS_INFO("%d ", (int)srv10.response.boolAnswer);
  
   
  client11.call(srv11);

    ROS_INFO("GetProperties");
     ROS_INFO("taille %d",  (int)srv11.response.resProperty.size());
   for(int i = 0; i<srv11.response.resProperty.size(); i++)
   {
		std::cout << "                                " + srv11.response.resProperty[i].linkType + "\n";
		ROS_INFO("%d ",  (int)srv11.response.boolAnswer );
   }
    
    client12.call(srv12);
     
       ROS_INFO("GetPropertiValue");
 
		std::cout <<  "                                " +  srv12.response.resProperty.linkType + "\n";
		ROS_INFO("%d ", (int)srv12.response.boolAnswer );
   
    
     client13.call(srv13);
     
     ROS_INFO("execute SQL");
     
   for(int i = 0; i<srv13.response.results.size(); i++)
   {
		std::cout << "                                " +   srv13.response.results[i] + "\n";
		
   }
   ROS_INFO("%d ", (int)srv13.response.boolAnswer );
   
   
   
   client16.call(srv16);
      ROS_INFO("GetCurrentsFacts");
   for(int i = 0; i<srv16.response.resFactList.factList.size(); i++)
   {
	   
		std::cout << "                                " +   srv16.response.resFactList.factList[i].subjectId + "\n";
		ROS_INFO("%d ", (int)srv16.response.boolAnswer );
   }
   
   
   client17.call(srv17);
   
         ROS_INFO("GetpAASEDFacts");
   for(int i = 0; i<srv17.response.resFactList.factList.size(); i++)
   {
	   
		std::cout <<  "                                " +  srv17.response.resFactList.factList[i].subjectId + "\n";
		ROS_INFO("%d ", srv17.response.boolAnswer );
   } 
   
   client18.call(srv18);
   
            ROS_INFO("GetCurrentsFactsfROMaGENT");
   for(int i = 0; i<srv18.response.resFactList.factList.size(); i++)
   {
	   
		std::cout <<  "                                " +  srv18.response.resFactList.factList[i].subjectId + " " + srv18.response.resFactList.factList[i].property + " " + srv18.response.resFactList.factList[i].targetId + "\n";
		ROS_INFO("%d ", (int)srv18.response.boolAnswer );
   }
   

   
   
   client19.call(srv19);

               ROS_INFO("GetpASSEDFactsfROMaGENT");
   for(int i = 0; i<srv19.response.resFactList.factList.size(); i++)
   {
	   
		std::cout <<  "                                " +  srv19.response.resFactList.factList[i].subjectId + " " + srv19.response.resFactList.factList[i].property + " " + srv19.response.resFactList.factList[i].targetId + "\n";
		ROS_INFO("%d ",  (int)srv19.response.boolAnswer);
   }
   
   client20.call(srv20);
   
     ROS_INFO("ADDEVENT");
     
        client21.call(srv21);
   
     ROS_INFO("GETEVENTS");
    for(int i = 0; i<srv21.response.resEventList.size(); i++)
   {
	   
		std::cout << "                                " +   srv21.response.resEventList[i].subjectId + " " + srv21.response.resEventList[i].property + " " + srv21.response.resEventList[i].targetId + "\n";
		ROS_INFO("%d \n",  (int)srv21.response.boolAnswer);
   }
     
     client22.call(srv22);
 
     ROS_INFO("GETEVENTValue");
		std::cout << "                                " +   srv22.response.resEvent.subjectId + " " + srv22.response.resEvent.property + " " + srv22.response.resEvent.targetId + "\n";
		ROS_INFO("%d ",  (int)srv22.response.boolAnswer);
		
		
		     client23.call(srv23);
 
     ROS_INFO("GETOntologies");

    for(int i = 0; i<srv23.response.resOntology.size(); i++)
   {
	   
		std::cout <<  "                                " +  srv23.response.resOntology[i].entityClass + " " + srv23.response.resOntology[i].individual + " " +  "\n";
		ROS_INFO("%d ",  (int)srv23.response.boolAnswer);
   }
	
	client24.call(srv24);
        ROS_INFO("GETOntologyvalues");
        
    for(int i = 0; i<srv24.response.resOntology.size(); i++)
   {
		std::cout <<  "                                " +  srv24.response.resOntology[i].entityClass + " " + srv24.response.resOntology[i].individual + " " +  "\n";
		ROS_INFO("%d ",  (int)srv24.response.boolAnswer);
   }
        
        
        client25.call(srv25);
             ROS_INFO("GETOntologyleaves");
             
             
    for(int i = 0; i<srv25.response.resOntology.size(); i++)
   {
		std::cout <<  "                                " +  srv25.response.resOntology[i].entityClass + " " + srv25.response.resOntology[i].individual + " " +  "\n";
		ROS_INFO("%d ",  (int)srv25.response.boolAnswer);
   }
                
             
             
   
   
  return 0;
}
