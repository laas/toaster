toaster
=======

Tracking Of Human (actions and belief) and Spatio-TEmporal Reasoning.

This framework implement ros nodes that play a specific role in building situation assessment 
for the robot to understand its surrounding environment and have spatio-temporal reasoning on
agents actions and belief.

The framework is build in a modular way so that it would be easy to extend the reasoning
by including new type of data input, new kind of reasoning or new hypothesis for the situation
assessment.

This framework could be used for various application, from navigation to joint action,
from situated dialogue to temporal reasoning on events.




Nodes included:

PDG: Perception Data Gathering
This node will collect all the perception data (agents configuration, object positions...).
These data will be cast in toaster-lib data types and send in topics.

SPAR: SPAtial Reasoning
This node will use the data from PDG to compute geometric facts concerning the environment.

