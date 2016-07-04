## How to use
To start the PDG ros node, run the following command on terminal -

```shell
> rosrun pdg pdg
```

## Description
In a scenario of human-robot interaction, data concerning three entities (humans, objects and robots) may come from various sensors with heterogeneous data-types. As we wish to keep the geometrical reasoning generic while having a flexible data-source system, we use a separate component **PDG (Perceived Data Gathering)** to collect the data from all the required sensors and publish them in a unique format usable by any other TOASTER component.

The current version already supports several inputs. For human tracking, TOASTER can get data from kinect-like and motion capture devices. Concerning objects, an object recognizer based on tags and stereo vision is supported. Finally, two different models of robots are directly functional in TOASTER.
However the set of inputs is easily extensible by adding a new sensor data reader to the PDG module. Additionally, TOASTER can use HRI simulators. The robotic simulator MORSE and a built-in simulation component (toaster_simu) are supported as inputs for any type of entities (objects, humans or robots). These two simulators allow to quickly test TOASTER components and allow to have a partially simulated interaction. As an example, we can have a real human interacting with a simulated robot by using data obtained from a motion capture for the human in real world, and data from a simulator output for the robot pose.

The following graph sum up the messages and services of PDG.

![](https://github.com/Greg8978/toaster/blob/master/doc/LatexSource/img/pdg.jpg)




## Services
On running this node, one can access following services -

+ **/pdg/manage_stream**

It enables to specify, at any moment, which sensors to use as raw input data. This makes the PDG component highly adaptable to the data needed for the current task and to the set of available sensors.
Command to call this service

```shell
>  rosservice call /pdg/manage_stream ...
```

**Notes:** _You can use tab for auto-completion to get the message format. _

Set the parameters as per the sensors in use 

 ```shell
> rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false,
  toasterSimuHuman: false, pr2Robot: false, spencerRobot: false, toasterSimuRobot: false,
  toasterSimuObject: false}
```

+ **/pdg/set_entity_pose**

It is used to set the position of any entity. The request parameters id and type points to the id and type of concerned entity whose position is to be changed. ownerID (optional) is a valid parameter for entity type of objects or joints.

```shell
rosservice call /toaster\_simu/set\_entity\_pose "{id: '', 
 ownerId: '', 
 type: '', 
 pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.0"}"
```

+ **/pdg/put_in_hand**

This service has provision of adding an object in any agent's (robot or human) hand or joint. 

```shell
> rosservice call /pdg/put_in_hand "objectId: ''
agentId: ''
jointName: ''" 
```

+ **/pdg/remove_from_hand**

It enables to remove object from agent's hand.

```shell
> rosservice call /pdg/remove_from_hand "objectId: ''" 
```



