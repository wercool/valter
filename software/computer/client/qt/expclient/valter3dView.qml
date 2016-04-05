import QtQuick 2.0
import QtCanvas3D 1.0

import "valter3d.js" as GLCode

Canvas3D
{
    id: valter3DCanvas
    anchors.fill: parent

    property double xRot: -90.0
    property double yRot: 0.0
    property double distance: 1.5

    onInitializeGL: GLCode.initializeGL(valter3DCanvas)
    onPaintGL: GLCode.paintGL(valter3DCanvas)
    onResizeGL: GLCode.resizeGL(valter3DCanvas)

    function setSpeed(speed)
    {
        GLCode.setSpeed(speed);
    }

    function setGroupRotationX(angle)
    {
        GLCode.setGroupRotationX(angle)
    }

    MouseArea
    {
        anchors.fill: parent

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
        }
        onMouseYChanged:
        {
            // Do not rotate if we don't have previous value
            if (previousX !== 0)
                valter3DCanvas.xRot += mouseX - previousX
            previousX = mouseX
            // Wrap the rotation around
            if (valter3DCanvas.xRot > 180)
                valter3DCanvas.xRot -= 360
            if (valter3DCanvas.xRot < -180)
                valter3DCanvas.xRot += 360
        }
        onReleased:
        {
            // Reset previous mouse positions to avoid rotation jumping
            previousX = 0
            previousY = 0
        }
        onWheel:
        {
            valter3DCanvas.distance -= wheel.angleDelta.y / 1000.0
            // Limit the distance to 0.5...10
            if (valter3DCanvas.distance < 0.5)
                valter3DCanvas.distance = 0.5
            if (valter3DCanvas.distance > 10)
                valter3DCanvas.distance = 10
        }
    }
}
