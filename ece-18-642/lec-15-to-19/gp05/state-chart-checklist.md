# Statechart peer review checklist

Modify, update, add, edit as seems appropriate

1. Initialization arc
1. Each state has unique identifier and a state name (e.g. "S5. INIT")
1. Each arc has a guard condition stated with a unique identifier (e.g., "A7. bump==true")
1. Each arc corresponds to at least one arc on at least one Sequence Diagram
1. No side effects on arcs
1. Each state has all side effects identified as being set to a value
1. No use of boolean variables that could instead be expressed as states
1. Each state has at least one entrance and one exit (except any intentional "death states" to handle system failures).
