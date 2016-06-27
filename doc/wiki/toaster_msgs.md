This package defines all ROS messages and services used in TOASTER. All the messages type like Area, Agent, Entity, Fact, etc, have been defined in msg folder while service like AddAgent, AddArea, etc, in srv folder. Descriptions in .msg and .srv files makes it easy for ROS tools to automatically generate source code for the message type in several target languages.


## Implementation details

toaster_msgs includes uses one message type in defining another higher level of message. This establishes relationship of hierarchy and aggregation among various messages, can be seen in the figure below. Services used in various modules are described in subsequent sections. To see the detailed structure of each messages refer to [here](https://github.com/Greg8978/toaster/tree/master/toaster_msgs/msg) and [here](https://github.com/Greg8978/toaster/tree/master/toaster_msgs/srv) for services.

This package also provides classes to help other modules to read the messages and to convert then into `toaster-lib` data structure.
These class are `ToasterFactReader`, `ToasterObjectReader`, `ToasterHumanReader`, `ToasterRobotReader`. 


![](https://writelatex.s3.amazonaws.com/rztjkrqdrypx/uploads/2528/6319118/1.jpg)

 
 
## Symbolic representation and fact
Our situation assessment modules perform geometric computations to get a symbolic representation of the world. This symbolic representation is made based on a list of a data structure named a `fact`. In our system, a `fact is a vector, with several fields, used to represent a property of the environment. We detail in [Facts](https://github.com/Greg8978/toaster/wiki/Facts) the fields of this vector.


