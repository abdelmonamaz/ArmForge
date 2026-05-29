import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import ArmForge

View3D {
    id: view3d

    environment: SceneEnvironment {
        clearColor: "#0d1117"
        backgroundMode: SceneEnvironment.Color
        antialiasingMode: SceneEnvironment.MSAA
        antialiasingQuality: SceneEnvironment.High
        tonemapMode: SceneEnvironment.TonemapModeLinear
    }

    Node {
        id: originNode
        position: Qt.vector3d(0, 250, 0)
        eulerRotation: Qt.vector3d(-25, 45, 0)

        PerspectiveCamera {
            id: mainCamera
            position: Qt.vector3d(0, 0, 1400)
            clipNear: 1
            clipFar: 20000
        }
    }

    DirectionalLight {
        eulerRotation: Qt.vector3d(-45, 30, 0)
        brightness: 3.0
        color: Qt.rgba(1.0, 0.95, 0.88, 1.0)
        castsShadow: true
        shadowMapQuality: Light.ShadowMapQualityMedium
    }
    DirectionalLight {
        eulerRotation: Qt.vector3d(-10, -90, 0)
        brightness: 1.5
        color: Qt.rgba(0.5, 0.7, 1.0, 1.0)
        castsShadow: false
    }
    DirectionalLight {
        eulerRotation: Qt.vector3d(20, 160, 0)
        brightness: 1.0
        color: Qt.rgba(0.7, 0.85, 1.0, 1.0)
        castsShadow: false
    }
    DirectionalLight {
        eulerRotation: Qt.vector3d(-90, 0, 0)
        brightness: 0.6
        color: Qt.rgba(0.9, 0.9, 1.0, 1.0)
        castsShadow: false
    }

    Model {
        source: "#Rectangle"
        scale: Qt.vector3d(30, 30, 1)
        eulerRotation: Qt.vector3d(-90, 0, 0)
        castsShadows: false
        receivesShadows: true
        materials: PrincipledMaterial {
            baseColor: "#161d28"
            roughness: 0.9
            metalness: 0.0
        }
    }

    RobotView { id: robotView }

    GizmoView {}

    PlanningView {}

    Connections {
        target: RobotController
        function onIkEnabledChanged() {
            if (RobotController.ikEnabled)
                RobotController.setTargetFromScene(RobotController.endEffectorPosition)
        }
    }

    Loader {
        id: orbitLoader
        anchors.fill: parent
        active: !gizmoDrag.dragging
        sourceComponent: Component {
            OrbitCameraController {
                anchors.fill: parent
                origin: originNode
                camera: mainCamera
            }
        }
    }

    // ── Gizmo drag handler ─────────────────────────────────────
    // Left-click on sphere → ray-plane intersection on Y=gizmo plane
    //   The gizmo follows the mouse cursor exactly in 3D space.
    // Right-click anywhere → Y axis (screen-up = world-up)
    MouseArea {
        id: gizmoDrag
        anchors.fill: parent
        propagateComposedEvents: true
        acceptedButtons: Qt.LeftButton | Qt.RightButton

        property bool dragging:   false
        property real lastY_px:   0          // last screen Y for right-drag
        property real planeY:     0          // Y height of the XZ drag plane
        property var  prevXZ:     null       // last {x,z} world intersection

        // Unproject a screen point onto the horizontal plane Y=planeY.
        // Returns {x,z} world coordinates, or null if the ray is parallel/behind.
        function unprojectXZ(screenX, screenY, targetY) {
            const mat  = mainCamera.sceneTransform
            // Camera world position = col 3 of sceneTransform
            const ox = mat.m14, oy = mat.m24, oz = mat.m34

            // NDC [-1,1] — screen Y is flipped
            const nx = (2.0 * screenX / view3d.width)  - 1.0
            const ny = 1.0 - (2.0 * screenY / view3d.height)

            // Ray in camera local space (vertical FOV, -Z forward)
            const tanH   = Math.tan(mainCamera.fieldOfView * Math.PI / 360.0)
            const aspect = view3d.width / view3d.height
            const lx = nx * tanH * aspect
            const ly = ny * tanH
            const lz = -1.0

            // Rotate to world space using camera rotation (mat cols 0-2)
            const wx = lx*mat.m11 + ly*mat.m12 + lz*mat.m13
            const wy = lx*mat.m21 + ly*mat.m22 + lz*mat.m23
            const wz = lx*mat.m31 + ly*mat.m32 + lz*mat.m33

            // Intersect ray with horizontal plane Y=targetY
            if (Math.abs(wy) < 1e-6) return null     // ray parallel to plane
            const t = (targetY - oy) / wy
            if (t < 0) return null                    // plane behind camera

            return { x: ox + t*wx, z: oz + t*wz }
        }

        onPressed: function(mouse) {
            if (!RobotController.ikEnabled) {
                mouse.accepted = false
                return
            }
            if (mouse.button === Qt.RightButton) {
                dragging    = true
                lastY_px    = mouse.y
                mouse.accepted = true
                console.log("[GIZMO] RIGHT-PRESS  target.y=",
                            RobotController.targetPosition.y.toFixed(1))
                return
            }
            var hit = view3d.pick(mouse.x, mouse.y)
            var hitName = (hit.objectHit ? hit.objectHit.objectName : "<none>")
            console.log("[GIZMO] LEFT-PRESS  hit=", hitName)
            if (hit.objectHit && hit.objectHit.objectName === "gizmoSphere") {
                planeY   = RobotController.targetPosition.y
                prevXZ   = unprojectXZ(mouse.x, mouse.y, planeY)
                dragging = true
                mouse.accepted = true
                const t = RobotController.targetPosition
                console.log("[GIZMO] XZ drag start  planeY=", planeY.toFixed(1),
                            " initXZ=(", (prevXZ ? prevXZ.x.toFixed(1) : "?"),
                            (prevXZ ? prevXZ.z.toFixed(1) : "?"), ")",
                            " target=(", t.x.toFixed(1), t.y.toFixed(1), t.z.toFixed(1), ")")
                console.log("[GIZMO] cam.sceneTransform col3/pos: ",
                            mainCamera.sceneTransform.m14.toFixed(1),
                            mainCamera.sceneTransform.m24.toFixed(1),
                            mainCamera.sceneTransform.m34.toFixed(1))
                console.log("[GIZMO] cam.fieldOfView=", mainCamera.fieldOfView,
                            " view size=", view3d.width.toFixed(0), "x", view3d.height.toFixed(0))
            } else {
                mouse.accepted = false
            }
        }

        onPositionChanged: function(mouse) {
            if (!dragging) return

            if (mouse.buttons & Qt.RightButton) {
                const dy = mouse.y - lastY_px
                lastY_px   = mouse.y

                // Same depth-based scale as the XZ unproject, so Y feels consistent.
                const mat  = mainCamera.sceneTransform
                const t    = RobotController.targetPosition
                const depth = Math.sqrt(
                    (t.x - mat.m14) * (t.x - mat.m14) +
                    (t.y - mat.m24) * (t.y - mat.m24) +
                    (t.z - mat.m34) * (t.z - mat.m34))
                const tanH  = Math.tan(mainCamera.fieldOfView * Math.PI / 360.0)
                const scale = 2.0 * depth * tanH / view3d.height

                const deltaY = -dy * scale
                console.log("[GIZMO] Y-DRAG  dy=", dy.toFixed(2),
                            " depth=", depth.toFixed(0),
                            " scale=", scale.toFixed(3),
                            " Δy=", deltaY.toFixed(2))
                RobotController.offsetTarget(Qt.vector3d(0, deltaY, 0))
            } else {
                const curr = unprojectXZ(mouse.x, mouse.y, planeY)
                if (!curr || !prevXZ) return

                const dx3d = curr.x - prevXZ.x
                const dz3d = curr.z - prevXZ.z
                prevXZ = curr

                console.log("[GIZMO] XZ-DRAG  screen=(", (mouse.x - lastY_px).toFixed(1), ")",
                            " Δworld=(", dx3d.toFixed(1), dz3d.toFixed(1), ")",
                            " newTarget.xz=(", (RobotController.targetPosition.x+dx3d).toFixed(1),
                            (RobotController.targetPosition.z+dz3d).toFixed(1), ")")
                RobotController.offsetTarget(Qt.vector3d(dx3d, 0, dz3d))
            }
        }

        onReleased: function(mouse) {
            dragging = false
            prevXZ   = null
            mouse.accepted = false
            const t = RobotController.targetPosition
            console.log("[GIZMO] RELEASED  finalTarget=(",
                        t.x.toFixed(1), t.y.toFixed(1), t.z.toFixed(1), ")")
        }
    }
}
