rosservice call /area_manager/add_area "myArea:
  id: 1
  name: 'pushing'
  myOwner: 'spencer'
  areaType: ''
  factType: ''
  entityType: 'human'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: -2.0, y: 1.0, z: 0.0}
    - {x: 0.0, y: 0.0, z: 0.0}
    - {x: 2.0, y: 1.0, z: 0.0}
    - {x: 2.5, y: -1.0, z: 0.0}
    - {x: -2.5, y: -1.0, z: 0.0}
    - {x: -2.0, y: 0.0, z: 0.0}
  insideEntities: [0]" 

rosservice call /area_manager/add_area "myArea:
  id: 2
  name: 'normal'
  myOwner: 'spencer'
  areaType: ''
  factType: ''
  entityType: 'human'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: -2.5, y: -1.0, z: 0.0}
    - {x: 2.5, y: -1.0, z: 0.0}
    - {x: 2.5, y: -2.5, z: 0.0}
    - {x: -2.5, y: -2.5, z: 0.0}
    - {x: -2.5, y: -1.0, z: 0.0}
  insideEntities: [0]" 

rosservice call /area_manager/add_area "myArea:
  id: 3
  name: 'slow'
  myOwner: 'spencer'
  areaType: ''
  factType: ''
  entityType: 'human'
  isCircle: false
  center: {x: 0.0, y: 0.0, z: 0.0}
  ray: 0.0
  poly:
    points:
    - {x: -2.5, y: -2.5, z: 0.0}
    - {x: 2.5, y: -2.5, z: 0.0}
    - {x: 2.5, y: -4.5, z: 0.0}
    - {x: -2.5, y: -4.5, z: 0.0}
    - {x: -2.5, y: -2.5, z: 0.0}
  insideEntities: [0]" 

