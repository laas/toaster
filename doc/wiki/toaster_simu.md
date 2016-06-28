## How to use
To start the toaster_simu ros node, run the following command on terminal -

```shell
> rosrun toaster_simu toaster_simu
```

## Description

`toaster_simu` is an inbuilt-simulation component of TOASTER which allows to add entities (objects, robots, humans and joints)
and provides the desired environment to TOASTER for situation assessment.
This makes possible to quickly set up an interaction environment in order to test new features or new modules without requiring any robotic device or any real environment.

**WARNING** toaster_simu is not a real simulator as it is only meant to add entities (human, robot, objects) to the environment in order to quickly test toaster modules. If you wish to simulate perception or to have further functionalities, please use a real simulator as [morse](https://www.openrobots.org/wiki/morse/).

The following figure gives an idea of the implementation of services and topics in toaster_simu.

![](https://github.com/Greg8978/toaster/blob/master/doc/LatexSource/img/toasterSimu.jpg)



## Inputs
Inputs for this module are id, type and position of entities to be added to the environment being assessed. They are provided through services as described below. Another set of inputs will be from keyboard if any entity is teleoperated (see figure below for keyboard teleoperation keys).

![](https://writelatex.s3.amazonaws.com/rztjkrqdrypx/uploads/2525/6319109/1.jpg)

## Outputs
Outputs for toaster_simu component are published on topics `/toaster_simu/humanList`, `/toaster_simu/robotList` and `/toaster_simu/objectList` which are subscribed by higher level component PDG.

## Services
This component provides the following services :

**add_entity** - enables you to add an entity to the environment. It takes a string `id` to give to the entity, a string `name` which will be also use to find the corresponding 3D model, a string `type` which should be set to the entity type (human, robot, object or joint). The last parameter is the string `ownerId`. It is used for joint entities to give the id of the agent it belongs to.

**Shell command to call the service:**
```shell
rosservice call /toaster_simu/add_entity "id: ''
name: ''
type: ''
ownerId: ''"
```

**remove_entity** - enables you to remove an entity from the environment. It takes strings to identify the entity.

**Shell command:**
```shell
rosservice call /toaster_simu/remove_entity "id: ''
type: ''
ownerId: ''"}
```

**set_entity_keyboard** - enables you to set any entity to keyboard mode so that it can be controlled by keyboard. It takes a sting with the id of the entity you want to control.

**Shell command:**
```shell
rosservice call /toaster_simu/set_entity_keyboard "id: ''"
```

**set_entity_pose:** - enables you to set position of any existing entity. The three first parameters `id, ownerId, type` are strings meant to identify the entity whose position is to be changed. The last parameter is the pose in which we want to put the entity with the type `geometry\_msgs/Pose` from ros.

**Shell command:**
```shell
\textit{ rosservice call /toaster\_simu/set\_entity\_pose "{id: '', 
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
    w: 0.0"}" }
 ```
 



## Limitations and improvements
The current version of toaster\_simu allows to quickly add or remove any kind of entity from the environment. However, as it is meant to be a tool for quick testing, it doesn't provide a simulation of perception as a simulator would do but directly add a "perceived" world state. Nevertheless, TOASTER is also compatible with inputs from the robotic open-source software MORSE.

An other limitation is that an agent can be added with his joints, however for now, the joint position is not updated with the agent's one (see [issue 19](https://github.com/Greg8978/toaster/issues/19)).