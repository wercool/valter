import QtQuick 2.0
import QtCanvas3D 1.0

import "valter3d.js" as GLCode

Canvas3D
{
    id: valter3DCanvas
    anchors.fill: parent

    onInitializeGL: GLCode.initializeGL(valter3DCanvas)
    onPaintGL: GLCode.paintGL(valter3DCanvas)
    onResizeGL: GLCode.resizeGL(valter3DCanvas)

    function setSpeed(speed)
    {
        GLCode.setSpeed(speed);
    }
}
