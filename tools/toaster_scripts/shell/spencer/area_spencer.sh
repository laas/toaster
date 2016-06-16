# Init for spencer project

#pdg set-up
rosservice call /pdg/manage_stream "{morseHuman: false, niutHuman: false, groupHuman: false, mocapHuman: true, humanCentroid: true,
  pr2Robot: false, spencerRobot: true, vimanObject: false, sparkObject: false}"



#area manager setup
rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'sides'
  myOwner: 1
  areaType: 'situation'
  factType: 'density'
  entityType: 'humans'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: 0.2, y: 2, z: 0.0}
    - {x: 0.2, y: -2, z: 0.0}
    - {x: -1, y: -2, z: 0.0}
    - {x: -1, y: 2, z: 0.0}
    - {x: 0.2, y: 2, z: 0.0}

  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 2
  name: 'behind'
  myOwner: 1
  areaType: 'situation'
  factType: 'density'
  entityType: 'humans'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: -1, y: 3, z: 0.0}
    - {x: -1, y: -3, z: 0.0}
    - {x: -3.0, y: -3, z: 0.0}
    - {x: -3.0, y: 3, z: 0.0}
    - {x: -1, y: 3, z: 0.0}
  insideEntities: [0]"

rosservice call /area_manager/add_area "myArea:
  id: 3
  name: 'rear'
  myOwner: 1
  areaType: 'situation'
  factType: 'density'
  entityType: 'humans'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: -3, y: 3, z: 0.0}
    - {x: -3, y: -3, z: 0.0}
    - {x: -4.0, y: -3, z: 0.0}
    - {x: -4.0, y: 3, z: 0.0}
    - {x: -3, y: 3, z: 0.0}
  insideEntities: [0]"

#agent monitor setup
rosservice call /agent_monitor/add_agent "id: 152                                                                                  
name: 'Human_Centroid'" 

