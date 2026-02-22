```mermaid
stateDiagram-v2

[*] --> Idle
Idle --> DetectCube
DetectCube --> QRRead
QRRead --> IdentifyColor
IdentifyColor --> PlanPick
PlanPick --> ExecutePick
ExecutePick --> PlanPlace
PlanPlace --> ExecutePlace
ExecutePlace --> Complete
Complete --> [*]
```
