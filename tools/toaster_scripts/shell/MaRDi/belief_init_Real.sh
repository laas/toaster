#HERAKLES_HUMAN1
rosservice call /belief_manager/add_fact "fact: {property: 'isAt', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'HERAKLES_HUMAN1', targetId: 'Livingroom_coffeetable', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"


rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'HERAKLES_HUMAN1', targetId: 'Livingroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"


#Locations
rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'Livingroom_coffeetable', targetId: 'Livingroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'Livingroom_table', targetId: 'Livingroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

#rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
#  subjectId: 'Bedroom_chest', targetId: 'Bedroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
#  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'Bedroom_shelf', targetId: 'Bedroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

#rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
#  subjectId: 'Bedroom_console', targetId: 'Bedroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
#  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'Bedroom_bedsidetable', targetId: 'Bedroom', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'Kitchen_table', targetId: 'Kitchen', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

#rosservice call /belief_manager/add_fact "fact: {property: 'isIn', propertyType: 'state', subProperty: '', subjectId: 0, targetId: 0,
#  subjectId: 'Kitchen_table', targetId: 'Kitchen', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
#  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"


#Object type
rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'LOTR_TAPE', targetId: 'DVD', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'WALLE_TAPE', targetId: 'DVD', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'GREY_TAPE', targetId: 'DVD', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'HANDLE_BOX', targetId: 'TOOL_BOX', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'BLACK_CUBE', targetId: 'BOX', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'GREEN_CUBE2', targetId: 'BOX', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'RED_CUBE2', targetId: 'BOX', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Type', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'BLUE_CUBE', targetId: 'BOX', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"



#object color
rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'GREY_TAPE', targetId: 'Grey', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'LOTR_TAPE', targetId: 'Black', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'WALLE_TAPE', targetId: 'White', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'BLACK_CUBE', targetId: 'Black', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'BLUE_CUBE2', targetId: 'Blue', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'RED_CUBE2', targetId: 'Red', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Color', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'GREEN_CUBE2', targetId: 'Green', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"


#object title
rosservice call /belief_manager/add_fact "fact: {property: 'Title', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'GREY_TAPE', targetId: 'Dorian Grey', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Title', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'LOTR_TAPE', targetId: 'Le seigneur des anneaux', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Title', propertyType: 'staticProperty', subProperty: '', subjectId: 0, targetId: 0,
  subjectId: 'WALLE_TAPE', targetId: 'Wally', subjectOwnerId: "", targetOwnerId: "", valueType: true, factObservability: 0.0,
  doubleValue: 0.0, stringValue: '', confidence: 0.0, time: 0, timeStart: 0, timeEnd: 0}"


