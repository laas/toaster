# Init for spencer project

#pdg set-up

# No spark (branch genom /mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: true, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: true,
#  spencerRobot: false, vimanObject: true, sparkObject: false, sparkFact: false}"

# spark (branch genom / mardi_dev2)
rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: false, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: true, om2mObject: true, gazeboObject: false}" 

# no genom (branch master)
#rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: true, pr2Robot: true,
#  spencerRobot: false}" 



#area manager setup

#############
### AREA ###
#############
rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Readingroom'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 2.3, y: 9.1, z: 0.0}
    - {x: 9.4, y: 9.1, z: 0.0}
    - {x: 9.4, y: 5.1, z: 0.0}
    - {x: 2.3, y: 5.1, z: 0.0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Bedroom'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 2.3, y: 9.1, z: 0.0}
    - {x: 6.4, y: 9.1, z: 0.0}
    - {x: 6.4, y: 13.2, z: 0.0}
    - {x: 2.3, y: 13.2, z: 0.0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Kitchen'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 0.0, y: 5.0, z: 0.0}
    - {x: 0.0, y: 9.0, z: 0.0}
    - {x: 2.3, y: 9.0, z: 0.0}
    - {x: 2.3, y: 5.0, z: 0.0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Kitchen'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 0.0, y: 5.0, z: 0.0}
    - {x: 0.0, y: 9.0, z: 0.0}
    - {x: 2.3, y: 9.0, z: 0.0}
    - {x: 2.3, y: 5.0, z: 0.0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"


rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Bed'
  myOwner: 'Bed'
  areaType: 'bed_area'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 0.5, y: 1.0, z: 0.0}
    - {x: -0.5, y: 1.0, z: 0.0}
    - {x: -0.5, y: -1.0, z: 0.0}
    - {x: 0.5, y: -1.0, z: 0.0}
  zmin: 0.0
  zmax: 0.5
  enterHysteresis: 0.01
  leaveHysteresis: 0.3
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'robot_interaction'
  myOwner: 'PR2_ROBOT'
  areaType: ''
  factType: 'interaction'
  entityType: 'agents'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 2.0
  height: 0.0
  poly:
    points:
    - {x: 0, y: -1, z: 0}
    - {x: 2, y: -2, z: 0}
    - {x: 2, y: 2, z: 0}
    - {x: 0, y: 1, z: 0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"
  
# IoT part:

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'LivingRoom'
  myOwner: ''
  areaType: 'room'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 6.5, y: 9.1, z: 0.0}
    - {x: 6.5, y: 13.2, z: 0.0}
    - {x: 9.4, y: 13.2, z: 0.0}
    - {x: 9.4, y: 9.1, z: 0.0}
  zmin: 0.0
  zmax: 2.0
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"



###############
### OBJECTS ###
###############
  

rosservice call /toaster_simu/add_entity "id: 'Readingroom_table'
name: 'Readingroom_table'
type: 'object'
ownerId: ''"

rosservice call /toaster_simu/set_entity_pose "id: 'Readingroom_table'
ownerId: ''
type: 'object'
pose:
  position:
    x: 9.0
    y: 6.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0" 

rosservice call /toaster_simu/add_entity "id: 'Readingroom_table2'
name: 'Readingroom_table2'
type: 'object'
ownerId: ''"

rosservice call /toaster_simu/set_entity_pose "id: 'Readingroom_table2'
ownerId: ''
type: 'object'
pose:
  position:
    x: 9.0
    y: 8.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0" 

   
rosservice call /toaster_simu/add_entity "id: 'Bed'
name: 'Bed'
type: 'object'
ownerId: ''"

rosservice call /toaster_simu/set_entity_pose "id: 'Bed'
ownerId: ''
type: 'object'
pose:
  position:
    x: 3.5
    y: 11.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Livingroom_low_table'
name: 'Livingroom_low_table'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_low_table'
ownerId: ''
type: 'object'
pose:
  position:
    x: 7.2
    y: 12.9
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"


rosservice call /toaster_simu/add_entity "id: 'Livingroom_table'
name: 'Livingroom_table'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_table'
ownerId: ''
type: 'object'
pose:
  position:
    x: 8.9
    y: 9.9
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"


rosservice call /toaster_simu/add_entity "id: 'Livingroom_small_shelf'
name: 'Livingroom_small_shelf'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_small_shelf'
ownerId: ''
type: 'object'
pose:
  position:
    x: 6.7
    y: 11.4
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Livingroom_shelf_1'
name: 'Livingroom_shelf_1'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_shelf_1'
ownerId: ''
type: 'object'
pose:
  position:
    x: 6.6
    y: 10.7
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Livingroom_shelf_2'
name: 'Livingroom_shelf_2'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_shelf_2'
ownerId: ''
type: 'object'
pose:
  position:
    x: 7.9
    y: 9.4
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"


rosservice call /toaster_simu/add_entity "id: 'Livingroom_tele_1'
name: 'Livingroom_tele_1'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_tele_1'
ownerId: ''
type: 'object'
pose:
  position:
    x: 6.6
    y: 10.7
    z: 1.06
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Livingroom_tele_2'
name: 'Livingroom_tele_2'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_tele_2'
ownerId: ''
type: 'object'
pose:
  position:
    x: 7.9
    y: 9.4
    z: 1.06
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"



rosservice call /area_manager/publish_all_areas "{}"

