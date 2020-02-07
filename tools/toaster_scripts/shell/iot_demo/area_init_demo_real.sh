# Init for spencer project

#pdg set-up

# No spark (branch genom /mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: true, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: true,
#  spencerRobot: false, vimanObject: true, sparkObject: false, sparkFact: false}"

# spark (branch genom / mardi_dev2)
rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: true, toasterSimuHuman: false, pr2Robot: false, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: true, om2mObject: true, gazeboObject: false, mocapObject: false}" 

# no genom (branch master)
#rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: true, pr2Robot: true,
#  spencerRobot: false}" 




#############
### AREA ###
#############

# IoT part:

rosservice call /area_manager/add_area "myArea:
  id: 2
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
  zmax: 2.5
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: ['']
  upcomingEntities_: ['']
  leavingEntities_: ['']"

rosservice call /area_manager/add_area "myArea:
  id: 3
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
    - {x: 2.35, y: 9.1, z: 0.0}
    - {x: 6.3, y: 9.1, z: 0.0}
    - {x: 6.3, y: 13.2, z: 0.0}
    - {x: 2.35, y: 13.2, z: 0.0}
  zmin: 0.0
  zmax: 2.5
  enterHysteresis: 0.01
  leaveHysteresis: 0.01
  insideEntities_: ['']
  upcomingEntities_: ['']
  leavingEntities_: ['']"


###############
### OBJECTS ###
###############
   
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
    y: 11.4
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
    x: 6.7
    y: 12.95
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"


rosservice call /toaster_simu/add_entity "id: 'Livingroom_black_table'
name: 'Livingroom_black_table'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Livingroom_black_table'
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
    y: 11.8
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

rosservice call /toaster_simu/add_entity "id: 'Bedroom_console'
name: 'Bedroom_console'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Bedroom_console'
ownerId: ''
type: 'object'
pose:
  position:
    x: 6.0
    y: 11.6
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Bedroom_chest'
name: 'Bedroom_chest'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Bedroom_chest'
ownerId: ''
type: 'object'
pose:
  position:
    x: 3.9
    y: 12.9
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.5"

rosservice call /toaster_simu/add_entity "id: 'Bedroom_big_cupboard'
name: 'Bedroom_big_cupboard'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Bedroom_big_cupboard'
ownerId: ''
type: 'object'
pose:
  position:
    x: 4.4
    y: 9.4
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"

rosservice call /toaster_simu/add_entity "id: 'Bedroom_cupboard1'
name: 'Bedroom_cupboard1'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Bedroom_cupboard1'
ownerId: ''
type: 'object'
pose:
  position:
    x: 3.3
    y: 9.4
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0"

rosservice call /toaster_simu/add_entity "id: 'Bedroom_cupboard3'
name: 'Bedroom_cupboard3'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: 'Bedroom_cupboard3'
ownerId: ''
type: 'object'
pose:
  position:
    x: 5.2
    y: 12.9
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.5"

