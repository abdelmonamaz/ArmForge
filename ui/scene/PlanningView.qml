import QtQuick
import QtQuick3D
import ArmForge

// Phase 4 — 3-D visualization for the planner: obstacle spheres (translucent
// red), taught waypoint markers (amber), and the planned end-effector trail
// (small green dots, one per RRT-Connect path step).
//
// Repeater3D (not Repeater) — Model/Node delegates live in the 3-D scene graph.
Node {
    id: root

    component MarkerMat: PrincipledMaterial {
        roughness: 0.3
        metalness: 0.1
    }

    // ── Obstacles ──────────────────────────────────────────────
    Repeater3D {
        model: PlanningController.obstacles
        Model {
            source: "#Sphere"
            position: modelData.center
            // "#Sphere" has radius 50 in scene units — scale to match obstacle radius.
            scale: Qt.vector3d(modelData.radius / 50, modelData.radius / 50, modelData.radius / 50)
            materials: PrincipledMaterial {
                baseColor: "#ef4444"
                roughness: 0.6
                metalness: 0.0
                opacity: 0.28
                alphaMode: PrincipledMaterial.Blend
            }
        }
    }

    // ── Waypoint markers ───────────────────────────────────────
    Repeater3D {
        model: PlanningController.waypoints
        Model {
            source: "#Sphere"
            position: modelData.ee
            scale: Qt.vector3d(0.34, 0.34, 0.34)
            materials: MarkerMat { baseColor: "#f59e0b"; emissiveFactor: Qt.vector3d(0.12, 0.07, 0.0) }
        }
    }

    // ── Planned end-effector trail ─────────────────────────────
    Repeater3D {
        model: PlanningController.pathPoints
        Model {
            source: "#Sphere"
            position: modelData
            scale: Qt.vector3d(0.16, 0.16, 0.16)
            materials: MarkerMat { baseColor: "#22c55e"; emissiveFactor: Qt.vector3d(0.0, 0.1, 0.02) }
        }
    }
}
