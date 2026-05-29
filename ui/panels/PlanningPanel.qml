import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import ArmForge

// Phase 4 — teach waypoints, plan an obstacle-avoiding path (RRT-Connect),
// replay it through RobotController. Pure orchestration — the algorithm
// lives in AF_Core (RrtConnectPlanner), execution in RobotController.followPath().
Rectangle {
    id: root
    color: "#080d14"

    function currentConfigDeg() {
        return [RobotController.q0, RobotController.q1, RobotController.q2,
                RobotController.q3, RobotController.q4, RobotController.q5]
    }

    component PanelButton: Rectangle {
        id: btn
        property string label: ""
        property bool   enabled_: true
        signal clicked()

        height: 26
        radius: 4
        color:        area.pressed ? "#1e3a5f" : (enabled_ ? "#0d1f33" : "#0a121d")
        border.color: enabled_ ? "#2563eb" : "#1e293b"
        border.width: 1

        Text {
            anchors.centerIn: parent
            text: btn.label
            color: btn.enabled_ ? "#60a5fa" : "#334155"
            font.pixelSize: 10
            font.bold: true
            font.letterSpacing: 1
            font.family: "monospace"
        }
        MouseArea {
            id: area
            anchors.fill: parent
            enabled: btn.enabled_
            onClicked: btn.clicked()
        }
    }

    // The panel's content (waypoints + plan/run + GPU benchmark) can exceed
    // the space allotted to PlanningPanel — wrap it in a Flickable with a
    // visible scrollbar so every section/button stays reachable.
    Flickable {
        id: scrollArea
        anchors.fill: parent
        anchors.margins: 16
        contentWidth: width
        contentHeight: content.implicitHeight
        clip: true
        boundsBehavior: Flickable.StopAtBounds

        ScrollBar.vertical: ScrollBar {
            policy: ScrollBar.AsNeeded
            width: 6
        }

        ColumnLayout {
            id: content
            width: scrollArea.width
            spacing: 8

            Text {
                text: "PLANNING"
            color: "#3b82f6"
            font.pixelSize: 13
            font.bold: true
            font.letterSpacing: 3
            Layout.alignment: Qt.AlignHCenter
        }

        Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

        // ── Waypoints ────────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            spacing: 6

            Text {
                text: "WAYPOINTS · " + PlanningController.waypoints.length
                color: "#94a3b8"
                font.pixelSize: 9
                font.bold: true
                font.letterSpacing: 1
                font.family: "monospace"
                Layout.fillWidth: true
            }
            PanelButton {
                label: "TEACH"
                Layout.preferredWidth: 60
                onClicked: PlanningController.addWaypoint(root.currentConfigDeg())
            }
            PanelButton {
                label: "CLEAR"
                enabled_: PlanningController.waypoints.length > 0
                Layout.preferredWidth: 56
                onClicked: PlanningController.clearWaypoints()
            }
        }

        ListView {
            Layout.fillWidth: true
            Layout.preferredHeight: Math.min(72, model.length * 18)
            clip: true
            model: PlanningController.waypoints
            delegate: RowLayout {
                width: ListView.view.width
                spacing: 6
                Text {
                    text: "WP" + (index + 1)
                    color: "#4ade80"
                    font.pixelSize: 9
                    font.family: "monospace"
                    width: 28
                }
                Text {
                    text: "(" + (modelData.ee.x/1000).toFixed(2) + ", "
                              + (modelData.ee.y/1000).toFixed(2) + ", "
                              + (modelData.ee.z/1000).toFixed(2) + ") m"
                    color: "#64748b"
                    font.pixelSize: 9
                    font.family: "monospace"
                    Layout.fillWidth: true
                }
                Text {
                    text: "✕"
                    color: "#ef4444"
                    font.pixelSize: 10
                    MouseArea { anchors.fill: parent; onClicked: PlanningController.removeWaypoint(index) }
                }
            }
        }

        Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

        // ── GPU planner toggle ───────────────────────────────────
        // When on (and CUDA is available), plan() runs CudaRrtPlanner —
        // same RRT-Connect strategy, but each candidate segment is validated
        // in one batched GPU kernel launch instead of N sequential CPU checks.
        Rectangle {
            Layout.fillWidth: true
            height: 28
            radius: 5
            visible: PlanningController.cudaAvailable
            color:        PlanningController.useGpuPlanner ? "#0d2b14" : "#0d1420"
            border.color: PlanningController.useGpuPlanner ? "#22c55e" : "#1e3a5f"
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.leftMargin:  10
                anchors.rightMargin: 10

                Text {
                    text: "PLANIFIER SUR GPU"
                    color: PlanningController.useGpuPlanner ? "#22c55e" : "#334155"
                    font.pixelSize: 9
                    font.bold: true
                    font.letterSpacing: 1
                    font.family: "monospace"
                }
                Item { Layout.fillWidth: true }
                Rectangle {
                    width: 28; height: 16; radius: 8
                    color: PlanningController.useGpuPlanner ? "#22c55e" : "#1e293b"
                    Rectangle {
                        width: 12; height: 12; radius: 6
                        color: "white"
                        x: PlanningController.useGpuPlanner ? parent.width - width - 2 : 2
                        anchors.verticalCenter: parent.verticalCenter
                        Behavior on x { NumberAnimation { duration: 120 } }
                    }
                }
            }
            MouseArea {
                anchors.fill: parent
                onClicked: PlanningController.useGpuPlanner = !PlanningController.useGpuPlanner
            }
        }

        // ── Plan / run ───────────────────────────────────────────
        RowLayout {
            Layout.fillWidth: true
            spacing: 6

            PanelButton {
                label: PlanningController.planning ? "PLANNING…" : "PLAN PATH"
                enabled_: PlanningController.waypoints.length > 0 && !PlanningController.planning
                Layout.fillWidth: true
                onClicked: PlanningController.plan(root.currentConfigDeg())
            }
            PanelButton {
                label: RobotController.pathExecuting ? "RUNNING…" : "RUN PATH"
                enabled_: PlanningController.lastPlanSuccess
                          && PlanningController.pathConfigsDeg.length > 0
                          && !RobotController.pathExecuting
                Layout.fillWidth: true
                onClicked: RobotController.followPath(PlanningController.pathConfigsDeg,
                                                       RobotController.trajectoryDuration)
            }
        }

        Text {
            visible: PlanningController.pathPoints.length > 0
            text: PlanningController.lastPlanSuccess
                  ? ("● PATH FOUND — " + PlanningController.pathPoints.length + " steps · "
                     + PlanningController.lastPlanIterations + " iter")
                  : ("● NO PATH — " + PlanningController.lastPlanIterations + " iter")
            color: PlanningController.lastPlanSuccess ? "#22c55e" : "#ef4444"
            font.pixelSize: 10
            font.bold: true
            font.family: "monospace"
        }

        Text {
            text: "Spheres = obstacles · trail = planned EE path"
            color: "#1e3a5f"
            font.pixelSize: 9
            font.family: "monospace"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

        Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

        // ── Phase 5 — CPU vs GPU collision-check benchmark ───────
        // Validates the same batch of random configs through a sequential
        // CPU loop and a single GPU kernel launch (af::cuda::batchValidate),
        // timing each. Falls back gracefully when CUDA isn't available.
        Text {
            text: "GPU BENCHMARK"
            color: "#3b82f6"
            font.pixelSize: 11
            font.bold: true
            font.letterSpacing: 2
            Layout.alignment: Qt.AlignHCenter
        }

        RowLayout {
            Layout.fillWidth: true
            spacing: 6

            Text {
                text: "SAMPLES · " + benchSlider.value.toFixed(0)
                color: "#94a3b8"
                font.pixelSize: 9
                font.bold: true
                font.letterSpacing: 1
                font.family: "monospace"
                Layout.fillWidth: true
            }
            PanelButton {
                label: "RUN"
                Layout.preferredWidth: 50
                onClicked: PlanningController.runBenchmark(Math.round(benchSlider.value))
            }
        }

        Slider {
            id: benchSlider
            Layout.fillWidth: true
            from: 1000
            to: 100000
            stepSize: 1000
            value: 20000
        }

        ColumnLayout {
            visible: PlanningController.benchmarkRan
            Layout.fillWidth: true
            spacing: 2

            Text {
                text: "CPU (séquentiel) · " + PlanningController.benchmarkSampleCount
                      + " configs · " + PlanningController.benchmarkCpuMs.toFixed(2) + " ms"
                color: "#fbbf24"
                font.pixelSize: 10
                font.family: "monospace"
            }
            Text {
                visible: PlanningController.cudaAvailable
                text: "GPU (1 lancement) · " + PlanningController.benchmarkGpuMs.toFixed(2)
                      + " ms · accélération ×" + PlanningController.benchmarkSpeedup.toFixed(1)
                color: "#4ade80"
                font.pixelSize: 10
                font.bold: true
                font.family: "monospace"
            }
        }

        Text {
            visible: !PlanningController.cudaAvailable
            text: "⚠ CUDA indisponible (compilé sans support GPU, ou aucun device détecté) — comparaison limitée au CPU"
            color: "#f87171"
            font.pixelSize: 9
            font.family: "monospace"
            wrapMode: Text.WordWrap
            Layout.fillWidth: true
        }

            Item { Layout.fillHeight: true }
        }
    }
}
