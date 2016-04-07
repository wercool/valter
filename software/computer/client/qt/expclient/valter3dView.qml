import QtQuick 2.0
import QtCanvas3D 1.0

import "valter3d.js" as GLCode


Canvas3D
{
    id: valter3DCanvas
    anchors.fill: parent
    focus: true
    width: 640
    height: 480


    property int orbitControlState: 0 //0 - no control, 1 - orbit camera, 2 - pan camera

    property double mouseX: 0.0;
    property double mouseY: 0.0;

    onInitializeGL: GLCode.initializeGL(valter3DCanvas)
    onPaintGL: GLCode.paintGL(valter3DCanvas)
    onResizeGL: GLCode.resizeGL(valter3DCanvas)

    function setValterGroupRotationY(angle)
    {
        GLCode.setValterGroupRotationY(angle)
    }

    function setLink1ZAngle(angle)
    {
        GLCode.setLink1ZAngle(angle)
    }

    function setLink2ZAngle(angle)
    {
        GLCode.setLink2ZAngle(angle)
    }

    function setManTiltZAngle(angle)
    {
        GLCode.setManTiltZAngle(angle)
    }

    function updateLabels()
    {
        valterGroupPosition.text = "valterGroup.pos [x, y, x] = " + GLCode.valterGroup.position.x + ", " + GLCode.valterGroup.position.y + ", " + GLCode.valterGroup.position.z;
        valterGroupRotation.text = "valterGroup.rot [x, y, x] = " + GLCode.valterGroup.rotation.x + ", " + GLCode.valterGroup.rotation.y + ", " + GLCode.valterGroup.rotation.z;
    }

    MouseArea
    {
        id: mouseArea1
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons;

        property double wheelDelta: 0

        onMouseXChanged:
        {
            valter3DCanvas.mouseX = mouseX;
        }
        onMouseYChanged:
        {
            valter3DCanvas.mouseY = mouseY;
        }

        onPressed:
        {
            if (mouse.button == Qt.RightButton)
            {
                GLCode.handleMouseDownPan(mouseX, mouseY);
                orbitControlState = 2;
            }
            if (mouse.button == Qt.MiddleButton)
            {
                GLCode.handleMouseDownRotate(mouseX, mouseY);
                orbitControlState = 1;
            }
        }

        onReleased:
        {
            orbitControlState = 0;
        }
        onWheel:
        {
            wheelDelta = wheel.angleDelta.y / 500.0;
            if (wheelDelta > 0)
            {
                GLCode.dollyIn();
            }
            else
            {
                GLCode.dollyOut();
            }
        }

        Text
        {
            id: valterGroupPosition
            x: 5
            y: 5
            text: qsTr("valterGroup.pos [x, y, x] = ")
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignTop
            textFormat: Text.PlainText
            font.pixelSize: 10
        }

        Text
        {
            id: valterGroupRotation
            x: 5
            y: 20
            text: qsTr("valterGroup.rot [x, y, x] = ")
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignTop
            textFormat: Text.PlainText
            font.pixelSize: 10
        }

        Text {
            id: axisXLabel
            x: 600
            y: 5
            color: "#ff0000"
            text: qsTr("X")
            anchors.right: parent.right
            anchors.rightMargin: 30
            font.bold: true
            font.pixelSize: 10
            textFormat: Text.PlainText
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignTop
        }

        Text {
            id: axisYLabel
            x: 610
            y: 5
            color: "#00ff00"
            text: qsTr("Y")
            anchors.right: parent.right
            anchors.rightMargin: 20
            font.pixelSize: 10
            textFormat: Text.PlainText
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignTop
            font.bold: true
        }

        Text {
            id: axisZLabel
            x: 620
            y: 5
            color: "#0000ff"
            text: qsTr("Z")
            anchors.right: parent.right
            anchors.rightMargin: 10
            font.pixelSize: 10
            textFormat: Text.PlainText
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignTop
            font.bold: true
        }
    }

    Keys.onPressed:
    {
        if (event.key === Qt.Key_R)
        {
            console.log("Reset");
        }
    }

    Keys.onReleased:
    {
        if (event.key === Qt.Key_Control)
        {
        }
    }
}
