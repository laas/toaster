
rosservice call /belief_manager/add_fact "fact: {property: 'Move', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'ACCESSKIT-Kitchen_table-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'new', confidence: 0.0, time: 0}"

rosservice call /belief_manager/add_fact "fact: {property: 'Put', propertyType: 'knowledge', subProperty: '', subjectId: 'HERAKLES_HUMAN1', targetId: 'ACCESSKIT-Kitchen_table-',
  subjectOwnerId: '', targetOwnerId: '', valueType: false, factObservability: 0.0,
  doubleValue: 0.0, stringValue: 'new', confidence: 0.0, time: 0}"

rosservice call /htn_verbalizer/set_present_agents "names:
- 'HERAKLES_HUMAN1'" 


rosservice call /htn_verbalizer/switch_policy 


