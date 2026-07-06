---
type: "query"
date: "2026-07-02T09:38:17.990759+00:00"
question: "Why does NavigationController connect Core Navigation Controller to UserMovementTracker Operations, POI & Waypoint Management, Geospatial Location Tracking, UI & UX Panels, Waypoint & Route Detection, Geospatial Location Tracking, Drift Correction, Waypoint & Route Detection?"
contributor: "graphify"
outcome: "useful"
source_nodes: ["NavigationController", "UserMovementTracker", "WaypointProximityDetector", "WrongWayDetector", "DriftCorrector"]
---

# Q: Why does NavigationController connect Core Navigation Controller to UserMovementTracker Operations, POI & Waypoint Management, Geospatial Location Tracking, UI & UX Panels, Waypoint & Route Detection, Geospatial Location Tracking, Drift Correction, Waypoint & Route Detection?

## Answer

NavigationController is the central orchestrator (degree 71) in NavigationController.cs. In its Update loop, UpdateWaypointNavigation() coordinates several subsystems: UserMovementTracker (via UpdateUserMovement), WrongWayDetector (via UpdateWrongWayDetection), TurnClassifier (via UpdateDynamicTurnReclassification), WaypointProximityDetector (via UpdateWaypointProximity), TurnPOISpawner, and DriftCorrector (by subscribing to OnExtremeDriftDetected). This structure links movement tracking, route validation, UI spawner feedback, and geospatial tracking into a single cohesive control loop.

## Outcome

- Signal: useful

## Source Nodes

- NavigationController
- UserMovementTracker
- WaypointProximityDetector
- WrongWayDetector
- DriftCorrector