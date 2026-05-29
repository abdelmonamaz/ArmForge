import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ArmForge

Rectangle {
    id: root
    color: "#080d14"

    readonly property var qProps: [
        { get: () => RobotController.q0, set: v => RobotController.q0 = v },
        { get: () => RobotController.q1, set: v => RobotController.q1 = v },
        { get: () => RobotController.q2, set: v => RobotController.q2 = v },
        { get: () => RobotController.q3, set: v => RobotController.q3 = v },
        { get: () => RobotController.q4, set: v => RobotController.q4 = v },
        { get: () => RobotController.q5, set: v => RobotController.q5 = v }
    ]

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 16
        spacing: 12

        // ── Header ─────────────────────────────────────────────
        Text {
            text: "JOG"
            color: "#3b82f6"
            font.pixelSize: 13
            font.bold: true
            font.letterSpacing: 3
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

        // ── IK toggle ──────────────────────────────────────────
        Rectangle {
            Layout.fillWidth: true
            height: 34
            radius: 5
            color:        RobotController.ikEnabled ? "#0d2b14" : "#0d1420"
            border.color: RobotController.ikEnabled ? "#22c55e" : "#1e3a5f"
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin:  10
                anchors.rightMargin: 10

                Text {
                    text: "IK MODE"
                    color: RobotController.ikEnabled ? "#22c55e" : "#334155"
                    font.pixelSize: 11
                    font.bold: true
                    font.letterSpacing: 2
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    width: 28; height: 16; radius: 8
                    color: RobotController.ikEnabled ? "#22c55e" : "#1e293b"
                    Rectangle {
                        width: 12; height: 12; radius: 6
                        color: "white"
                        x: RobotController.ikEnabled ? parent.width - width - 2 : 2
                        anchors.verticalCenter: parent.verticalCenter
                        Behavior on x { NumberAnimation { duration: 120 } }
                    }
                }
            }
            MouseArea {
                anchors.fill: parent
                onClicked: RobotController.ikEnabled = !RobotController.ikEnabled
            }
        }

        // ── IK status + target ─────────────────────────────────
        ColumnLayout {
            visible: RobotController.ikEnabled
            Layout.fillWidth: true
            spacing: 4

            Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

            Text {
                text: "TARGET (m)"
                color: "#22c55e"
                font.pixelSize: 9
                font.bold: true
                font.letterSpacing: 2
            }

            ColumnLayout {
                Layout.fillWidth: true
                spacing: 2

                Repeater {
                    model: ["X", "Y", "Z"]
                    Row {
                        spacing: 8
                        Text {
                            text: modelData
                            color: index === 0 ? "#f87171"
                                 : index === 1 ? "#4ade80"
                                 :               "#60a5fa"
                            font.pixelSize: 10
                            font.bold: true
                            font.family: "monospace"
                            width: 12
                        }
                        Text {
                            text: {
                                const t = RobotController.targetPosition
                                const v = index === 0 ? t.x : index === 1 ? t.y : t.z
                                return (v / 1000).toFixed(3)
                            }
                            color: "#94a3b8"
                            font.pixelSize: 10
                            font.family: "monospace"
                        }
                    }
                }
            }

            Text {
                text: RobotController.ikConverged ? "● CONVERGED" : "● UNREACHABLE"
                color: RobotController.ikConverged ? "#22c55e" : "#ef4444"
                font.pixelSize: 10
                font.bold: true
                font.family: "monospace"
            }

            // ── Trajectory duration ──────────────────────────────
            RowLayout {
                Layout.fillWidth: true
                spacing: 6

                Text {
                    text: "TRAJ"
                    color: "#22c55e"
                    font.pixelSize: 9
                    font.bold: true
                    font.letterSpacing: 1
                    font.family: "monospace"
                }

                Slider {
                    id: durSlider
                    Layout.fillWidth: true
                    from: 0.0; to: 3.0
                    value: RobotController.trajectoryDuration
                    onMoved: RobotController.trajectoryDuration = value

                    background: Rectangle {
                        x: durSlider.leftPadding
                        y: durSlider.topPadding + durSlider.availableHeight / 2 - height / 2
                        width: durSlider.availableWidth; height: 3; radius: 1
                        color: "#0d2b14"
                        Rectangle {
                            width: durSlider.visualPosition * parent.width
                            height: parent.height; radius: 1
                            color: "#166534"
                        }
                    }
                    handle: Rectangle {
                        x: durSlider.leftPadding + durSlider.visualPosition * (durSlider.availableWidth - width)
                        y: durSlider.topPadding  + durSlider.availableHeight / 2 - height / 2
                        width: 12; height: 12; radius: 6
                        color: "#22c55e"; border.color: "#4ade80"; border.width: 1
                    }
                }

                Text {
                    text: durSlider.value.toFixed(1) + "s"
                    color: "#94a3b8"
                    font.pixelSize: 9
                    font.family: "monospace"
                    width: 28
                }
            }

            Text {
                text: "L-drag = XZ · R-drag = Y"
                color: "#1e3a5f"
                font.pixelSize: 9
                font.family: "monospace"
            }

            Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }
        }

        // ── Joint sliders ──────────────────────────────────────
        Repeater {
            model: [
                { label: "J1", name: "Base Pan"     },
                { label: "J2", name: "Shoulder"     },
                { label: "J3", name: "Elbow"        },
                { label: "J4", name: "Wrist Roll"   },
                { label: "J5", name: "Wrist Pitch"  },
                { label: "J6", name: "Wrist Roll 2" }
            ]

            delegate: ColumnLayout {
                Layout.fillWidth: true
                spacing: 4

                readonly property int jointIndex: index

                RowLayout {
                    spacing: 6
                    Text {
                        text: modelData.label
                        color: RobotController.ikEnabled ? "#1e3a5f" : "#3b82f6"
                        font.pixelSize: 11
                        font.bold: true
                        font.family: "monospace"
                    }
                    Text {
                        text: modelData.name
                        color: "#64748b"
                        font.pixelSize: 10
                        Layout.fillWidth: true
                    }
                    Text {
                        id: degLabel
                        text: "  0°"
                        color: RobotController.ikEnabled ? "#334155" : "#94a3b8"
                        font.pixelSize: 11
                        font.family: "monospace"
                        horizontalAlignment: Text.AlignRight
                        width: 44
                    }
                }

                Slider {
                    id: jogSlider
                    Layout.fillWidth: true
                    from: -180
                    to: 180
                    value: 0
                    enabled: !RobotController.ikEnabled

                    onValueChanged: {
                        degLabel.text = (value >= 0 ? " " : "") + Math.round(value) + "°"
                        if (!RobotController.ikEnabled)
                            root.qProps[jointIndex].set(value)
                    }

                    // Sync slider display to IK-computed angles.
                    Connections {
                        target: RobotController
                        function onJointsChanged() {
                            if (RobotController.ikEnabled) {
                                const v = root.qProps[jointIndex].get()
                                jogSlider.value = v
                            }
                        }
                    }

                    background: Rectangle {
                        x: jogSlider.leftPadding
                        y: jogSlider.topPadding + jogSlider.availableHeight / 2 - height / 2
                        width: jogSlider.availableWidth
                        height: 4
                        radius: 2
                        color: "#1a2535"
                        Rectangle {
                            width: jogSlider.visualPosition * parent.width
                            height: parent.height
                            radius: 2
                            color: RobotController.ikEnabled ? "#1e3a5f" : "#2563eb"
                        }
                    }
                    handle: Rectangle {
                        x: jogSlider.leftPadding +
                           jogSlider.visualPosition * (jogSlider.availableWidth - width)
                        y: jogSlider.topPadding +
                           jogSlider.availableHeight / 2 - height / 2
                        width: 14; height: 14; radius: 7
                        color: RobotController.ikEnabled ? "#1e3a5f" : "#3b82f6"
                        border.color: RobotController.ikEnabled ? "#334155" : "#93c5fd"
                        border.width: 1
                    }
                }
            }
        }

        Item { Layout.fillHeight: true }

        Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

        RowLayout {
            Layout.fillWidth: true
            Text {
                text: RobotController.ikEnabled ? "Phase 3 — IK+Control" : "Phase 3 — FK"
                color: RobotController.ikEnabled ? "#166534" : "#1e3a5f"
                font.pixelSize: 9
                font.family: "monospace"
            }
            Item { Layout.fillWidth: true }
            Text { text: "ArmForge v0.3"; color: "#1e3a5f"; font.pixelSize: 9; font.family: "monospace" }
        }
    }
}
