# TOASTER WIKI

## What is TOASTER?
To understand the environment, the robot needs not only to perceive it but also to reason on the relationships between the different elements and the evolution of the properties describing the world state. This understanding of what happens around it is key in order for the robot to take appropriate decisions according to the situation. TOASTER (Tracking Of Agents and Spatio-TEmporal Reasoning), is a generic multi-sensor **situation assessment** framework for **human-robot interaction** able to maintain a world state, generate symbolic facts, track human motion and maintain a belief state for the robot as well as an **estimate of the belief state of its human partners.** The framework consists in several components, each with its specific spatial and temporal reasoning. 

TOASTER is a framework composed by several ROS modules and based on C++ library: `toaster-lib`. The goal of TOASTER is to shrink the gap between sensors high-level data output and decision layer. This is done by combining the data concerning 3 entity types: human, robot and object.

## What will I find in this wiki?
This wiki contains a user manual for each component and a brief description.

### Installation
[instructions](https://github.com/Greg8978/toaster/wiki/Installation)

[scripts for testing](https://github.com/Greg8978/toaster/wiki/tools)

### Modules descriptions:
[toaster_msgs](https://github.com/Greg8978/toaster/wiki/toaster_msgs)

[toaster_simu](https://github.com/Greg8978/toaster/wiki/toaster_simu)

[PDG](https://github.com/Greg8978/toaster/wiki/PDG)

[area_manager](https://github.com/Greg8978/toaster/wiki/area_manager)

[agent_monitor](https://github.com/Greg8978/toaster/wiki/agent_monitor)

[move3d_facts](https://github.com/Greg8978/toaster/wiki/move3d_facts)

[belief_manager](https://github.com/Greg8978/toaster/wiki/belief_manager)

[database_manager](https://github.com/Greg8978/toaster/wiki/database_manager)

### Other
[toaster-lib](https://github.com/Greg8978/toaster/wiki/toaster-lib)

[FAQ](https://github.com/Greg8978/toaster/wiki/FAQ)




If you can't find an answer in this wiki, you can open a new issue [here](https://github.com/Greg8978/toaster/issues), or ask your question at the mailing list [toaster-users@laas.fr](https://sympa.laas.fr/sympa/info/toaster-users) if you are a user and [toaster-dev@laas.fr](https://sympa.laas.fr/sympa/info/toaster-dev) if you are developing new features or wish to contribute to the TOASTER project.

![](http://i.imgur.com/7DTlww0.png)

## How to install TOASTER?
**Install toaster-lib**

To install toaster, you need first to install toaster-lib.
To do so, go to your installation folder, and do :

```shell
> git clone https://github.com/Greg8978/toaster-lib.git
> cd toaster-lib
> mkdir build
> cd build
> cmake ..
```

_Note: You may have to specify the boost headers directory, and you may want to specify the prefix for the install path. To do so, you can either use the gui with the command ccmake ... Press "t" key to toggle._
_Once you finished configuring, quit the cmake gui by generating the files. To do so, press "c" then "g" key._
_Alternatively you can directly specify this with the command:_

```shell
> cmake -DCMAKE_INSTALL_PREFIX=MY_INSTALL_PREFIX -DBoost_INCLUDE_DIR=MY_BOOST_INCLUDE_PATH/include ..
> make install
```

After installing `toaster-lib`, you will need to add `TOASTERLIB_DIR` variable to your `env`:

```shell
> export TOASTERLIB_DIR=MY_INSTALL_PREFIX
```

Alternatively, you may want to put this variable into your `.bashrc` 

**Install toaster**

Once you installed toaster-lib, you will need to install toaster.
To do so, go to your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
You may need first to get catkin [cmake_module](http://wiki.ros.org/cmake_modules). Then use the following commands :

```shell
> cd ${CATKIN_PATH}/src
> git clone https://github.com/Greg8978/toaster.git
> cd ..
> catkin_make
```