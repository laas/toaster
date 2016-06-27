toaster-lib is a C++ library which defines data structures used in TOASTER. 
The structures are:
**Entity**: this class defines a physical element of the world. It can be an agent or a joint or an object.
**Agent**: this class defines an entity which has a set of joints. This agent can be a Human or a Robot.
**Human**: this class represents a human agent.
**Robot**: this class represents a robotic agent.
**Joint**: this class defines an entity which belongs to an agent.
**Object**: this class defines an object. This object can be a MovableObjet or a StaticObject.
**MovableObject**: this class defines objects that may move during the interaction.
**StaticObject**: this class defines objects that can't move.
**Area**: this class defines an area. An area is a defined location of the environment.
**PolygonArea**: this class defines a polygonal area.
**CircleArea**: this class defines a circular area.

The attributes and links between these class are shown in the UML diagram at the figure below.
These key concepts (data structures) are used by TOASTER to represent the world state and to perform geometrical computations in order to obtain situation awareness for human-robot interaction. Apart from this, toaster-lib defines frequently used mathematical functions and aims to work with object-oriented approach.

![](https://writelatex.s3.amazonaws.com/rztjkrqdrypx/uploads/1505/6294078/1.png)