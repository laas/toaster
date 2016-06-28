## How to use
To start the database_manager ros node, run the following command on terminal -

```shell
> rosrun database_manager run_server
```

## Description
To store and manage the facts generated from Spatial and Temporal Reasoning components, and to add a memory to our system, we created a module based on a SQL Temporal Database. Apart from the conceptual perspective taking (as described in belief_manager), another capacity of the database component is the event based time management. This provides memory to the system.
When facts are received, the time of detection is present in one of their variables. When the database manager detects a shift in a property, it updates the belief tables as explained in previous section, but it also records the event. To do so, we add a table filled with each event that occurs, recording the time when the property changes. This is a significant component of TOASTER as it is used for testing and debugging this situation assessment framework.

## Implementation details
This module continuously reads facts from PDG, area_manager, agent_monitor and move3d_facts. All these facts are updated in the robot's belief state. Once the robot belief state is updated, it updates the other agent's belief state by the idea of conceptual perspective taking. Figure below explains its implementation :
 
![] (https://github.com/Greg8978/toaster/blob/master/doc/LatexSource/img/database.jpg)

## Inputs
To maintain robot's own belief state, the information produced by perception, geometrical reasoning and inferences are collected by the database management module. Facts computed from area_manager, agent_monitor and move3d_facts are stored in this module. 

## Outputs
The output of this component is a sql database with fact table and memory table for each agent present in ID table. The fact table stores all the fact true for the agent at that time instant. While, the memory table stores facts that the agent has believed to be true. There is an event table as well to track the occurrence of events like when robot started moving and when it stopped moving. However, during an interaction, many properties that describes an object or an agent may not evolve in time and thus, be considered as static (color, name, age, ownership). To store these static properties, an extra table is present in the database and can be loaded at the start of the interaction, or filled online (if the robot acquire new knowledge on entities). For more details, check https://github.com/Greg8978/toaster/blob/master/database_manager/doc/database%20doc.pdf.

## Services
Services of database_manager allows you to add entity, facts and events in respective tables and view all the tables. Its services are described below : 

* **execute** - This service enables users to access the data in tables in various ways by putting different commands.

**Shell command:**

```shell
 rosservice call /database/execute "command: ''
type: ''
facts:
- {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '',
  targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '',
  confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
agent: ''
order: ''
areaTopic: false
agentTopic: false
move3dTopic: false
pdgTopic: false" 
```
Set the _areaTopic, agentTopic, move3dTopic, pdgTopic _as per your requirement. Using command "ARE_IN_TABLE" with agent and facts, it checks if the given facts are present in agent's fact table. "SQL" command with the query in order request message, executes the SQL query in database and displays the results. The command "EMPTY" with type "AGENT" for a given agent removed that agent from ID table and other tables related to this agent. With "ALL" type, it does the same for all agents. Using command "PRINT" with type "AGENT" prints tables of the given agent, while type "ALL" prints all tables in database.

* **get\_info** - This service helps to get data from any given table. With the combination of type as "FACT" and subType as "ALL", it displays all the facts in the given agent's table at current time and before that also. While subType "VALUE", shows value of the given fact for the specified agent at current time. The subType "CURRENT" gives all the current facts of the given agent while "OLD" displays older facts. Similarly, combination of "ALL" subType with type as "EVENT", "ONTOLOGY", "ID" and "PROPERTY" displays respective tables from database. While subtype "VALUE" with all possible type gives values from respective tables. The request message looks like this :

**Shell command:**

```shell
rosservice call /database/get_info "type: ''
subType: ''
agentId: ''
reqFact: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
reqEvent: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}
id: 0
idString: ''
name: ''
entityClass: ''" 

```

* **plot_facts** - This service is developed with the purpose of examining the functioning of TOASTER in real-time environment. It plots the given fact for the given subject and target entity in specified time window. This service creates "fact_name.dat" (eg. IsMoving.dat) file in plot_fact folder of database_manager. You can also see the path of file on the database_manager terminal. file in your home directory. To plot the graph install gnuplot and run "gnuplot" command on terminal. In gnuplot terminal, run following commands:

```shell
gnuplot> set yrange[-1:2]
gnuplot>  plot "path of plot_fact/fact_name.dat" using 1:2 with lines
```
And you will see the plot of desired fact with time.

**Shell command:**

```shell
rosservice call /database/plot_facts "{subjectID: '', targetID: '', timeStart: '', timeEnd: '', reqFact: ''}"
```

* **set\_info** - 

**Shell command:**

```shell
rosservice call /database/set_info "add: false
infoType: ''
agentId: ''
facts:
- {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '', subjectOwnerId: '',
  targetOwnerId: '', valueType: false, factObservability: 0.0, doubleValue: 0.0, stringValue: '',
  confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}
event: {property: '', propertyType: '', subProperty: '', subjectId: '', targetId: '',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0}
id: ''
name: ''
type: ''
ownerId: ''"
```

## Examples
As an example, if a Mug was on a table and the robot detects that the Mug is now in Bob's hand, it will remove the fact _Mug isOn Kitchen_Table_ from the belief tables of agents able to perceive the change, and add the event _Bob pickUp Mug _in the event table.
In memory table, we use as starting time the time when the agent noticed a property, and as ending time the moment when the agent detects a change in it.
One of the application can be to understand when a human asks "Where is the mug that was on the table" by detecting in his memory table which mug she/he is talking about and looking in the current fact table where it currently is. In addition, the robot could even tell the related event that created the change: "It is now in the sink, Bob picked it up 5 minutes ago".

## Future work and possible improvement
This module is one of the last implementation of TOASTER. More tests should be done to ensure the database is able to answer fast enough to the requests.
