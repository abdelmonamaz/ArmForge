import QtQuick3D
import ArmForge

// 3-D IK target gizmo: sphere + axis markers.
// Positioned at RobotController.targetPosition (Qt3D scene units).
// Drag logic lives in SceneView — this component is purely visual.
Node {
    id: root

    visible: RobotController.ikEnabled
    position: RobotController.targetPosition

    // ── Main sphere ────────────────────────────────────────────
    Model {
        objectName: "gizmoSphere"
        source: "#Sphere"
        scale: Qt.vector3d(0.52, 0.52, 0.52)
        pickable: true
        materials: PrincipledMaterial {
            baseColor:      RobotController.ikConverged ? "#22c55e" : "#ef4444"
            roughness:      0.15
            metalness:      0.55
            emissiveFactor: RobotController.ikConverged
                            ? Qt.vector3d(0.0, 0.12, 0.0)
                            : Qt.vector3d(0.12, 0.0, 0.0)
        }
    }

    // ── X axis — red ───────────────────────────────────────────
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(80, 0, 0)
        scale: Qt.vector3d(0.04, 1.6, 0.04)
        eulerRotation: Qt.vector3d(0, 0, -90)
        materials: PrincipledMaterial { baseColor: "#ef4444"; roughness: 0.6; metalness: 0.0 }
    }

    // ── Y axis — green ─────────────────────────────────────────
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, 80, 0)
        scale: Qt.vector3d(0.04, 1.6, 0.04)
        materials: PrincipledMaterial { baseColor: "#22c55e"; roughness: 0.6; metalness: 0.0 }
    }

    // ── Z axis — blue ──────────────────────────────────────────
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, 0, 80)
        scale: Qt.vector3d(0.04, 1.6, 0.04)
        eulerRotation: Qt.vector3d(90, 0, 0)
        materials: PrincipledMaterial { baseColor: "#3b82f6"; roughness: 0.6; metalness: 0.0 }
    }
}
