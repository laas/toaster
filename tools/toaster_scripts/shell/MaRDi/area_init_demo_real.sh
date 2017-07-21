# Init for spencer project

#pdg set-up

# No spark (branch genom /mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: true, niutHuman: false, groupHuman: false, mocapHuman: false, pr2Robot: true,
#  spencerRobot: false, vimanObject: true, sparkObject: false, sparkFact: false}"

# spark (branch genom / mardi_dev2)
#rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: false, adreamMocapHuman: false, toasterSimuHuman: false, pr2Robot: true, spencerRobot: false, toasterSimuRobot: false, toasterSimuObject: true, arObject: true, om2mObject: false, gazeboObject: false}" 

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

rosservice call /area_manager/publish_all_areas "{}"

