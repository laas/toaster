
## Description

A fact is a symbolic representation of a truth concerning the environment. A fact is a vector composed of several fields such as:

**Subject**: the entity on which the property applies (e.g.: `Red_Mug`, `Human1`, `Pr2`, `Human_Right_Hand`).

**Property**: the property linked to the `Subject` (e.g.: `IsOn`, `IsFull`, `IsMoving`, `IsPointing`, `canSee`).

**Target**: the property may link the entity-subject with an entity-target. As an example if an entity `BOOK` is on an entity `TABLE`, `BOOK` will be the subject while `TABLE` will be the target.

**PropertyType**: this parameter defines the category in which the property falls (e.g.: `position`, `state`, `motion`, `posture`, `affordance`). Using this parameter, if an external module adds a fact, it is still possible to know which kind of property it is.

**Value**: a property may have a value linked to it. As an example, the property `IsFull` or `IsMoving` may have the value `TRUE` or `FALSE` (if we want to have a close world representation). As another example, if we represent the distance between joints, such as robot's hand to human head, the `Value` parameter could be set to `DANGER`, `CLOSE` or `FAR`, or even holds a numerical value.
In some situations, to represent a lack of knowledge on a property and the awareness of this lack, the fact's value can also be set to `unknown`.

**Confidence**: this figure between 0 and 1 represents the reliability of the fact. This can be linked to sensor reliability or to the property computation itself.

**Time**: the time at which the fact was computed.

**FactObservability**: the probability that a human would acquire the awareness of the fact if he sees the Subject of the fact.


As an example, the vector:
`Subject = Bob_Right_Hand, Property = IsMovingToward, Target = Red_Book, PropertyType = motion, Confidence = 0.8, time = 145571646570, FactObservability = 0.7`
 represents the fact that, at the given time, the hand (`Bob_Right_Hand`) of Bob is going toward a book (`Red_Book`) with a confidence of `0.8`. 
The following sections describe spatial and temporal reasoning components that generate these kind of facts.


## Facts computation

The fact computation is detailed in the page of each module.
To sum up, we give below a table which sums up where facts are computed.

