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

    property double xRot: -90.0
    property double yRot: 30.0
    property double distance: 5.0

    property bool mousePressed: true
    property bool panCamera: false

    property double mouseX: 0.0;
    property double mouseY: 0.0;
    property double mousePrevX: 0.0;
    property double mousePrevY:  0.0;

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

        property int previousY: 0
        property int previousX: 0

        onMouseXChanged:
        {
            // Do not rotate if we don't have previous value
            if (previousY !== 0)
                valter3DCanvas.yRot += mouseY - previousY
            previousY = mouseY
            // Limit the rotation to -90...90 degrees
            if (valter3DCanvas.yRot > 90)
                valter3DCanvas.yRot = 90
            if (valter3DCanvas.yRot < -90)
                valter3DCanvas.yRot = -90

            valter3DCanvas.mouseX = mouseX;
        }
        onMouseYChanged:
        {
            // Do not rotate if we don't have previous value
            if (previousX !== 0)
            {
                valter3DCanvas.xRot += mouseX - previousX
            }

            previousX = mouseX

            // Wrap the rotation around
            if (valter3DCanvas.xRot > 180)
                valter3DCanvas.xRot -= 360
            if (valter3DCanvas.xRot < -180)
                valter3DCanvas.xRot += 360

            valter3DCanvas.mouseY = mouseY;
        }

        onPressed:
        {
            if (mouse.button == Qt.RightButton)
            {
                panCamera = true;
            }

            mousePressed = true;

            valter3DCanvas.mouseX = mouseX;
            valter3DCanvas.mouseY = mouseY;
            valter3DCanvas.mousePrevX = 0;
            valter3DCanvas.mousePrevY = 0;

            GLCode.mousePointerXYToSceneXZ(valter3DCanvas);
        }

        onReleased:
        {
            // Reset previous mouse positions to avoid rotation jumping
            previousX = 0
            previousY = 0
            valter3DCanvas.mousePrevX = 0;
            valter3DCanvas.mousePrevY = 0;
            mousePressed = false;
            panCamera = false;
        }
        onWheel:
        {
            valter3DCanvas.distance -= wheel.angleDelta.y / 500.0
            // Limit the distance to 0.5...10
            if (valter3DCanvas.distance < 0.5)
                valter3DCanvas.distance = 0.5
            if (valter3DCanvas.distance > 10)
                valter3DCanvas.distance = 10
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
            color: "#0000ff"
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
            color: "#00ff00"
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
        if (event.key === Qt.Key_Control)
        {
            //panCamera = true;
        }
        if (event.key === Qt.Key_R)
        {
            distance = 5.0;
            GLCode.zeroVector.x = 0;
            GLCode.zeroVector.y = 1.2;
            GLCode.zeroVector.z = 0;
        }
    }

    Keys.onReleased:
    {
        if (event.key === Qt.Key_Control)
        {
            //panCamera = false;
        }
    }
}
