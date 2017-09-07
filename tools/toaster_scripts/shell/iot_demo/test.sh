rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'test'
  myOwner: ''
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  height: 0.0
  poly:
    points:
    - {x: 0.85, y: -0.5, z: 0.0}
    - {x: 0.85, y: 0.5, z: 0.0}
    - {x: -0.85, y: 0.5, z: 0.0}
    - {x: -0.85, y: -0.5, z: 0.0}
  zmin: -2.0
  zmax: 2.0
  enterHysteresis: 0.05
  leaveHysteresis: 0.05
  insideEntities_: ['']
  upcomingEntities_: ['']
  leavingEntities_: ['']"

rosservice call /area_manager/add_area "myArea:
  id: 0
  name: 'Tiede'
  myOwner: ''
  areaType: ''
  factType: ''
  entityType: 'entities'
  isCircle: true
  center: {x: 8.0, y: 8.0, z: 0.0}
  ray: 1.5
  height: 0.0
  poly:
    points:
    - {x: 0.85, y: -0.5, z: 0.0}
  zmin: -1.0
  zmax: 1.0
  enterHysteresis: 0.05
  leaveHysteresis: 0.05
  insideEntities_: ['']
  upcomingEntities_: ['']
  leavingEntities_: ['']"

