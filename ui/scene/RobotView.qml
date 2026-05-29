import QtQuick3D
import ArmForge

Node {
    id: robotRoot

    // Scene-space position of the last joint (used to seed the IK gizmo).
    readonly property var eeScenePosition: j6.scenePosition

    component LinkMat: PrincipledMaterial {
        baseColor: "#5c7e9e"
        roughness: 0.45
        metalness: 0.35
    }
    component JointMat: PrincipledMaterial {
        baseColor: "#2d6fd4"
        roughness: 0.20
        metalness: 0.50
    }

    // ── BASE PLATFORM ─────────────────────────────────────────
    Model {
        source: "#Cylinder"
        position: Qt.vector3d(0, 15, 0)
        scale: Qt.vector3d(1.6, 0.3, 1.6)
        materials: LinkMat {
            baseColor: "#2a3d52"
        }
    }

    // ── JOINT 1 — base pan (Y) ────────────────────────────────
    Node {
        id: j1
        position: Qt.vector3d(0, 119, 0)
        eulerRotation.y: RobotController.j1Yaw

        Model {
            source: "#Cylinder"
            position: Qt.vector3d(0, -44.5, 0)
            scale: Qt.vector3d(0.7, 0.89, 0.7)
            materials: LinkMat {}
        }
        Model {
            source: "#Sphere"
            scale: Qt.vector3d(0.56, 0.56, 0.56)
            materials: JointMat {}
        }

        // ── JOINT 2 — shoulder lift (Z) ───────────────────────
        Node {
            id: j2
            eulerRotation.z: RobotController.j2Roll

            Model {
                source: "#Cylinder"
                position: Qt.vector3d(212.5, 0, 0)
                scale: Qt.vector3d(0.44, 4.25, 0.44)
                eulerRotation: Qt.vector3d(0, 0, -90)
                materials: LinkMat {}
            }
            Model {
                source: "#Sphere"
                scale: Qt.vector3d(0.44, 0.44, 0.44)
                materials: JointMat {}
            }

            // ── JOINT 3 — elbow (Z) ───────────────────────────
            Node {
                id: j3
                position: Qt.vector3d(425, 0, 0)
                eulerRotation.z: RobotController.j3Roll

                Model {
                    source: "#Cylinder"
                    position: Qt.vector3d(196, 0, 0)
                    scale: Qt.vector3d(0.36, 3.92, 0.36)
                    eulerRotation: Qt.vector3d(0, 0, -90)
                    materials: LinkMat {}
                }
                Model {
                    source: "#Sphere"
                    scale: Qt.vector3d(0.44, 0.44, 0.44)
                    materials: JointMat {}
                }

                // ── JOINT 4 — wrist roll (X) ──────────────────
                Node {
                    id: j4
                    position: Qt.vector3d(392, 0, 0)
                    eulerRotation.x: RobotController.j4Roll

                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(0, 0, 54.5)
                        scale: Qt.vector3d(0.3, 1.09, 0.3)
                        eulerRotation: Qt.vector3d(90, 0, 0)
                        materials: LinkMat {}
                    }
                    Model {
                        source: "#Sphere"
                        scale: Qt.vector3d(0.36, 0.36, 0.36)
                        materials: JointMat {
                            baseColor: "#2563eb"
                        }
                    }

                    // ── JOINT 5 — wrist pitch (Y) — fléchit le poignet latéralement
                    Node {
                        id: j5
                        position: Qt.vector3d(0, 0, 109)
                        eulerRotation.y: RobotController.j5Roll

                        Model {
                            source: "#Cylinder"
                            position: Qt.vector3d(0, 0, 47.5)
                            scale: Qt.vector3d(0.24, 0.95, 0.24)
                            eulerRotation: Qt.vector3d(90, 0, 0)
                            materials: LinkMat {}
                        }
                        Model {
                            source: "#Sphere"
                            scale: Qt.vector3d(0.30, 0.30, 0.30)
                            materials: JointMat {
                                baseColor: "#3b82f6"
                            }
                        }

                        // ── JOINT 6 — wrist roll 2 (X) — rotation finale de l'outil
                        Node {
                            id: j6
                            position: Qt.vector3d(0, 0, 95)
                            eulerRotation.x: RobotController.j6Roll

                            Model {
                                source: "#Sphere"
                                scale: Qt.vector3d(0.24, 0.24, 0.24)
                                materials: JointMat {
                                    baseColor: "#60a5fa"
                                }
                            }
                            Model {
                                source: "#Cylinder"
                                position: Qt.vector3d(0, 0, 41)
                                scale: Qt.vector3d(0.18, 0.82, 0.18)
                                eulerRotation: Qt.vector3d(90, 0, 0)
                                materials: PrincipledMaterial {
                                    baseColor: "#f97316"
                                    roughness: 0.25
                                    metalness: 0.7
                                    emissiveFactor: Qt.vector3d(0.1, 0.04, 0.0)
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
