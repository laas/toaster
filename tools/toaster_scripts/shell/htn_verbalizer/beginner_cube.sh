
rosservice call /belief_manager/add_fact "fact: {property: 'ApplyFirstOperations', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Blue_Cube-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Handle', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Blue_Cube--',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Apply', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Cubes-Polish-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'expert', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'ApplyOperation', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Cubes-Polish-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'ApplyFirstOperations', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Red_Cube-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'ApplyOperation', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Cubes-Glue-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Handle', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'Cubes-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'BuildStack', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'StackArea-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'beginner', confidence: 0.0, time: 0}"



rosservice call /htn_verbalizer/set_present_agents "names:
  - 'HERAKLES_HUMAN1'" 


rosservice call /htn_verbalizer/switch_policy 


