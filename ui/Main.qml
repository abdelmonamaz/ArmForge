import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

Window {
    id: root
    width: 1400
    height: 860
    minimumWidth: 900
    minimumHeight: 600
    visible: true
    title: "ArmForge — 6-DOF Digital Twin"
    color: "#0d1117"

    SplitView {
        anchors.fill: parent
        orientation: Qt.Horizontal

        // Draggable divider — lets you widen the right panel to see its
        // buttons/labels comfortably instead of the old fixed 300 px width.
        handle: Rectangle {
            implicitWidth: 6
            color: SplitHandle.pressed ? "#2563eb"
                 : (SplitHandle.hovered ? "#3b82f6" : "#1e293b")

            Column {
                anchors.centerIn: parent
                spacing: 3
                Repeater {
                    model: 3
                    Rectangle {
                        width: 2; height: 2; radius: 1
                        color: "#64748b"
                    }
                }
            }
        }

        SceneView {
            SplitView.fillWidth: true
            SplitView.minimumWidth: 400
        }

        ColumnLayout {
            SplitView.preferredWidth: 300
            SplitView.minimumWidth:   260
            SplitView.maximumWidth:   560
            spacing: 0

            JogPanel {
                Layout.fillWidth:  true
                Layout.fillHeight: true
            }

            Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

            ChartPanel {
                Layout.fillWidth:       true
                Layout.preferredHeight: 150
            }

            Rectangle { height: 1; color: "#1e293b"; Layout.fillWidth: true }

            PlanningPanel {
                Layout.fillWidth:       true
                Layout.preferredHeight: 260
            }
        }
    }
}
