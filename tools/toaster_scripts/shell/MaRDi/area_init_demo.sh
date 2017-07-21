# Init for spencer project

#pdg set-up

# No spark (branch genom /mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: true, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: true,
#  spencerRobot: false, vimanObject: true, sparkObject: false, sparkFact: false}"

# spark (branch genom / mardi_dev2)
rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false,
  toasterSimuHuman: true, pr2Robot: false, spencerRobot: false, toasterSimuRobot: true,
  toasterSimuObject: true, arObject: true, om2mObject: true, gazeboObject: true}" 

# no genom (branch master)
#rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: true, pr2Robot: true,
#  spencerRobot: false}" 



#area manager setup

#############
### AREA ###
#############
rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Livingroom'
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
    - {x: 2.3, y: 9.0, z: 0.0}
    - {x: 9.4, y: 9.0, z: 0.0}
    - {x: 9.4, y: 5.0, z: 0.0}
    - {x: 2.3, y: 5.0, z: 0.0}
    - {x: 2.3, y: 9.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
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
    - {x: 2.3, y: 9.0, z: 0.0}
    - {x: 6.5, y: 9.0, z: 0.0}
    - {x: 6.5, y: 13.0, z: 0.0}
    - {x: 2.3, y: 13.0, z: 0.0}
    - {x: 2.3, y: 9.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
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
    - {x: 0.0, y: 5.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"


rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Drone_Area'
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
    - {x: 0.0, y: 0.0, z: 0.0}
    - {x: 0.0, y: 5.0, z: 0.0}
    - {x: 9.4, y: 5.0, z: 0.0}
    - {x: 9.4, y: 0.0, z: 0.0}
    - {x: 0.0, y: 0.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"


rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Livingroom_table'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 8.5, y: 5.0, z: 0.0}
    - {x: 8.5, y: 9.0, z: 0.0}
    - {x: 9.5, y: 9.0, z: 0.0}
    - {x: 9.5, y: 5.0, z: 0.0}
    - {x: 8.5, y: 5.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Kitchen_table'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 1.5, y: 6.8, z: 0.0}
  ray: 1.0
  height: 0.0
  poly:
    points:
    - {x: 0.0, y: 0.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Bedroom_bedsidetable'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 3.6, y: 1.8, z: 0.0}
  ray: 1.0
  height: 0.0
  poly:
    points:
    - {x: 0.0, y: 0.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Bed'
  myOwner: 'bed'
  areaType: 'bed_area'
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 2.5, y: 10.5, z: 0.0}
    - {x: 2.5, y: 11.5, z: 0.0}
    - {x: 4.5, y: 11.5, z: 0.0}
    - {x: 4.5, y: 10.5, z: 0.0}
    - {x: 2.5, y: 10.5, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Bedroom_shelf'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 3.0, y: 4.7, z: 0.0}
  ray: 1.0
  height: 0.0
  poly:
    points:
    - {x: 0.0, y: 0.0, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'interaction'
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
    - {x: 0, y: -1, z: 0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"
  
  
# IoT part:

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
    - {x: 6.5, y: 9.3, z: 0.0}
    - {x: 6.5, y: 13.3, z: 0.0}
    - {x: 9.4, y: 13.3, z: 0.0}
    - {x: 9.4, y: 9.3, z: 0.0}
    - {x: 6.5, y: 9.3, z: 0.0}
  zmin: 0.0
  zmax: 0.0
  enterHysteresis: 0.0
  leaveHysteresis: 0.0
  insideEntities_: [0]
  upcomingEntities_: [0]
  leavingEntities_: [0]"



###############
### OBJECTS ###
###############

#rosservice call /toaster_simu/add_entity "id: '1'
#name: 'Livingroom_table'
#type: 'object'
#ownerId: ''"

#rosservice call /toaster_simu/set_entity_pose "id: '1'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 9.0
#    y: 6.0
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 1.0
#    w: 0.0" 

#rosservice call /toaster_simu/add_entity "id: '2'
#name: 'Livingroom_table2'
#type: 'object'
#ownerId: ''"

#rosservice call /toaster_simu/set_entity_pose "id: '2'
#ownerId: ''
#type: 'object'
#pose:

#  position:
#    x: 9.0
#    y: 8.0
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 1.0
#    w: 0.0" 

rosservice call /toaster_simu/add_entity "id: '3'
name: 'FAUTEUIL_1'
type: 'object'
ownerId: ''" 

rosservice call /toaster_simu/set_entity_pose "id: '3'
ownerId: ''
type: 'object'
pose:
  position:
    x: 4.5
    y: 8.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.5" 
   
rosservice call /toaster_simu/add_entity "id: '4'
name: 'bed'
type: 'object'
ownerId: ''"

rosservice call /toaster_simu/set_entity_pose "id: '4'
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

#rosservice call /toaster_simu/add_entity "id: '5'
#name: 'IKEA_chair_STEFAN'
#type: 'object'
#ownerId: ''" 

#rosservice call /toaster_simu/set_entity_pose "id: '5'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 8.5
#    y: 10.0
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.5
#    w: 0.5" 

#rosservice call /toaster_simu/add_entity "id: '6'
#name: 'IKEA_console_HEMNES'
#type: 'object'
#ownerId: ''" 
 
#rosservice call /toaster_simu/set_entity_pose "id: '6'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 7.0
#    y: 3.0
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.0
#    w: 0.5" 

#rosservice call /toaster_simu/add_entity "id: '7'
#name: 'IKEA_low_table_LACK_1'
#type: 'object'
#ownerId: ''" 
 
#rosservice call /toaster_simu/set_entity_pose "id: '7'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 7.0
#    y: 13.0
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.0
#    w: 0.5" 


#rosservice call /toaster_simu/add_entity "id: '8'
#name: 'basic_desk'
#type: 'object'
#ownerId: ''" 
 
#rosservice call /toaster_simu/set_entity_pose "id: '8'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 9.0
#    y: 10.5
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.5
#    w: 0.5"

rosservice call /toaster_simu/add_entity "id: '9'
name: 'IKEA_chair_1'
type: 'object'
ownerId: ''" 
 
rosservice call /toaster_simu/set_entity_pose "id: '9'
ownerId: ''
type: 'object'
pose:
  position:
    x: 7.8
    y: 12.6
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 0.5" 

rosservice call /toaster_simu/add_entity "id: '10'
name: 'IKEA_chair_2'
type: 'object'
ownerId: ''" 
 
rosservice call /toaster_simu/set_entity_pose "id: '10'
ownerId: ''
type: 'object'
pose:
  position:
    x: 6.9
    y: 11.8
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.5
    w: 0.5" 

#rosservice call /toaster_simu/add_entity "id: '11'
#name: 'Carpet'
#type: 'object'
#ownerId: ''" 
 
#rosservice call /toaster_simu/set_entity_pose "id: '11'
#ownerId: ''
#type: 'object'
#pose:
#  position:
#    x: 5.5
#    y: 6.5
#    z: 0.0
#  orientation:
#    x: 0.0
#    y: 0.0
#    z: 0.5
#    w: 0.5" 


rosservice call /area_manager/publish_all_areas "{}"

