import QtQuick 2.0
import QtCanvas3D 1.0

import "valter3d.js" as Valter3D


Canvas3D
{
    id: valter3DCanvas
    anchors.fill: parent
    focus: true
    width: 640
    height: 480


    property int mouseButtonPressed: 0 //1 - no control (left mouse button), 2 - orbit camera (middle mouse button), 3 - pan camera (right mouse button)

    property double mouseX: 0.0;
    property double mouseY: 0.0;

    onInitializeGL: Valter3D.initializeGL(valter3DCanvas)
    onPaintGL: Valter3D.paintGL(valter3DCanvas)
    onResizeGL: Valter3D.resizeGL(valter3DCanvas)

    function setValterTrunkRotationY(angle)
    {
        Valter3D.setValterTrunkRotationY(angle)
    }

    function setValterBodyRotationZ(angle)
    {
        Valter3D.setValterBodyRotationZ(angle)
    }

    function setLink1ZAngle(angle)
    {
        Valter3D.setLink1ZAngle(angle)
    }

    function setLink2ZAngle(angle)
    {
        Valter3D.setLink2ZAngle(angle)
    }

    function setManTiltZAngle(angle)
    {
        Valter3D.setManTiltZAngle(angle)
    }

    function updateLabels()
    {
        //valterGroupPosition.text = "valterGroup.pos [x, y, x] = " + Valter3D.valterGroup.position.x + ", " + Valter3D.valterGroup.position.y + ", " + Valter3D.valterGroup.position.z;
        //valterGroupRotation.text = "valterGroup.rot [x, y, x] = " + Valter3D.valterGroup.rotation.x + ", " + Valter3D.valterGroup.rotation.y + ", " + Valter3D.valterGroup.rotation.z;
    }


    Keys.onPressed:
    {
        switch(event.key)
        {
            case Qt.Key_W:
                Valter3D.setValterGroupPositionDxDz(0.01, 0.01);
            break;
            case Qt.Key_S:
                Valter3D.setValterGroupPositionDxDz(-0.01, -0.01);
            break;
            case Qt.Key_A:
                Valter3D.setValterGroupRotationDy(0.01);
            break;
            case Qt.Key_D:
                Valter3D.setValterGroupRotationDy(- 0.01);
            break;
            case Qt.Key_R:
                Valter3D.initOrbitControl();
            break;
            case Qt.Key_Control:
                Valter3D.ctrlPressed = true;
            break;
        }
    }

    Keys.onReleased:
    {
        if (event.key === Qt.Key_Control)
        {
            Valter3D.ctrlPressed = false;
        }
    }

    MouseArea
    {
        id: mouseArea1
        anchors.fill: parent
        acceptedButtons: Qt.AllButtons;
        hoverEnabled: true;

        property double wheelDelta: 0

        onPositionChanged:
        {
            Valter3D.setMouse(valter3DCanvas, mouseX, mouseY);
            Valter3D.setMouseCameraRaycaster(valter3DCanvas, false);
        }

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
            if (mouse.button == Qt.LeftButton)
            {
                mouseButtonPressed = 1;
                Valter3D.setMouseCameraRaycaster(valter3DCanvas, true);
            }
            if (mouse.button == Qt.RightButton)
            {
                Valter3D.handleMouseDownPan(mouseX, mouseY);
                mouseButtonPressed = 2;
            }
            if (mouse.button == Qt.MiddleButton)
            {
                Valter3D.handleMouseDownRotate(mouseX, mouseY);
                mouseButtonPressed = 3;
            }
        }

        onReleased:
        {
            mouseButtonPressed = 0;
            if (Valter3D.selectedManipulationObject)
            {
                Valter3D.objectManipulationPlane.position.copy( Valter3D.interceptedObject.position );
                Valter3D.selectedManipulationObject = null;
            }
            Valter3D.headTarget.position.copy(Valter3D.headEndEffectorHelperPosition);
        }
        onWheel:
        {
            wheelDelta = wheel.angleDelta.y / 500.0;
            if (wheelDelta > 0)
            {
                Valter3D.dollyIn();
            }
            else
            {
                Valter3D.dollyOut();
            }
        }

        /*-----------------------------------------------GUI OBJECTS--------------------------------------------*/
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
}
