toaster
=======

Tracking Of Agent (actions and belief) and Spatio-TEmporal Reasoning.


![TOASTER screenshot 1](doc/media/human.png)
![TOASTER screenshot 2](doc/media/full.png)

This framework implement ros nodes that play a specific role in building situation assessment 
for the robot to understand its surrounding environment and have spatio-temporal reasoning on
agents actions and belief.

The framework is built in a modular way so that it would be easy to extend the reasoning
by including new type of data input, new kind of reasoning or new hypothesis for the situation
assessment.

This framework could be used for various application, from navigation to joint action,
from situated dialogue to temporal reasoning on events.


Nodes included :
----------------

pdg : Perception Data Gathering
This node will collect all the perception data (agents configuration, object positions...).
These data will be cast in toaster-lib data types and send in topics.

area_manager :
This node will use the data from PDG to compute geometric facts concerning the environment.

agent_monitor :

belief_manager :

toaster_visualizer :

database_manager :


A  [complete documentation](doc/media/toaster/index.html) is available.
