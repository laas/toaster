# Init for spencer project

#pdg set-up

# No spark (branch genom /mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: true, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: true,
#  spencerRobot: false, vimanObject: true, sparkObject: false, sparkFact: false}"

# spark (branch genom / mardi_dev2)
rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: false, adreamMocapHuman: false, toasterSimuRobot: true, arObject: false,
  spencerRobot: false, toasterSimuHuman: true, toasterSimuObject: true, arObject: true, om2mObject: true}"

# no genom (branch master)
#rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: true, pr2Robot: true,
#  spencerRobot: false}" 



#area manager setup
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
  name: 'Livingroom_coffeetable'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 3.84, y: 7.1, z: 0.0}
  ray: 1.3
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
  name: 'Livingroom_table'
  myOwner: ''
  areaType: 'support'
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 7.5, y: 6.1, z: 0.0}
  ray: 1.5
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
    
rosservice call /area_manager/publish_all_areas

