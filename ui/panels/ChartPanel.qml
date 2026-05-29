import QtQuick
import ArmForge

Rectangle {
    id: root
    color: "#080d14"

    // Ring buffer: array of 6-element arrays, one per jointsChanged signal
    property var history: []

    readonly property int  kSamples: 300   // 6 s at 50 Hz
    readonly property var  kColors:  ["#f87171","#fbbf24","#4ade80","#60a5fa","#a78bfa","#f472b6"]
    readonly property var  kLabels:  ["J1","J2","J3","J4","J5","J6"]

    Connections {
        target: RobotController
        function onJointsChanged() {
            const s = [
                RobotController.q0, RobotController.q1, RobotController.q2,
                RobotController.q3, RobotController.q4, RobotController.q5
            ]
            if (root.history.length >= root.kSamples)
                root.history.shift()
            root.history.push(s)
            chart.requestPaint()
        }
    }

    // ── Header ────────────────────────────────────────────────────
    Row {
        id: header
        anchors.top:   parent.top
        anchors.left:  parent.left
        anchors.right: parent.right
        anchors.margins: 6
        height: 18
        spacing: 0

        Text {
            text: "JOINT HISTORY"
            color: "#3b82f6"
            font.pixelSize: 9
            font.bold: true
            font.letterSpacing: 2
            anchors.verticalCenter: parent.verticalCenter
            width: parent.width * 0.45
        }

        // Legend
        Row {
            spacing: 8
            anchors.verticalCenter: parent.verticalCenter

            Repeater {
                model: root.kLabels
                Row {
                    spacing: 3
                    Rectangle {
                        width: 7; height: 7; radius: 2
                        color: root.kColors[index]
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: modelData
                        color: "#64748b"
                        font.pixelSize: 8
                        font.family: "monospace"
                        anchors.verticalCenter: parent.verticalCenter
                    }
                }
            }
        }
    }

    // ── Canvas oscilloscope ───────────────────────────────────────
    Canvas {
        id: chart
        anchors.top:    header.bottom
        anchors.bottom: parent.bottom
        anchors.left:   parent.left
        anchors.right:  parent.right
        anchors.topMargin:    2
        anchors.bottomMargin: 4
        anchors.leftMargin:   4
        anchors.rightMargin:  4

        onPaint: {
            const ctx = getContext("2d")
            const w = width, h = height
            ctx.clearRect(0, 0, w, h)

            // Background
            ctx.fillStyle = "#0d1117"
            ctx.fillRect(0, 0, w, h)

            // Horizontal grid at 0°, ±90°, ±180°
            const gridAngles = [-180, -90, 0, 90, 180]
            for (let gi = 0; gi < gridAngles.length; ++gi) {
                const a = gridAngles[gi]
                const y = h * (1.0 - (a + 180) / 360.0)
                ctx.strokeStyle = (a === 0) ? "#1e3a5f" : "#111827"
                ctx.lineWidth   = (a === 0) ? 1.0 : 0.5
                ctx.beginPath()
                ctx.moveTo(22, y)
                ctx.lineTo(w,  y)
                ctx.stroke()

                ctx.fillStyle  = "#1e3a5f"
                ctx.font       = "8px monospace"
                ctx.textAlign  = "right"
                ctx.fillText(a + "°", 20, y + 3)
            }

            // Joint curves
            const n = root.history.length
            if (n < 2) return

            for (let j = 0; j < 6; ++j) {
                ctx.strokeStyle = root.kColors[j]
                ctx.lineWidth   = 1.2
                ctx.beginPath()
                for (let i = 0; i < n; ++i) {
                    const x = 22 + (w - 22) * i / (root.kSamples - 1)
                    const y = h * (1.0 - (root.history[i][j] + 180) / 360.0)
                    if (i === 0) ctx.moveTo(x, y)
                    else         ctx.lineTo(x, y)
                }
                ctx.stroke()
            }
        }
    }
}
