import QtQuick 2.0
import QtCanvas3D 1.0

Item {
    Canvas3D {
        id: valter3DCanvas
        anchors.fill: parent
        focus: true
        property var gl

        onInitializeGL: {
            gl = valter3DCanvas.getContext("experimental-webgl");

            // Setup clear color to be a random color
            gl.clearColor(Math.random(), Math.random(), Math.random(), 1.0);

            // Setup viewport
            gl.viewport(0, 0, valter3DCanvas.width * valter3DCanvas.devicePixelRatio, valter3DCanvas.height * valter3DCanvas.devicePixelRatio);
        }

        onPaintGL: {
            // Clear background to current clear color
            gl.clear(gl.COLOR_BUFFER_BIT);
        }

        onResizeGL: {
            var pixelRatio = valter3DCanvas.devicePixelRatio;
            valter3DCanvas.pixelSize = Qt.size(valter3DCanvas.width * pixelRatio, valter3DCanvas.height * pixelRatio);
            if (gl)
                gl.viewport(0, 0, valter3DCanvas.width * valter3DCanvas.devicePixelRatio, valter3DCanvas.height * valter3DCanvas.devicePixelRatio);
        }
    }

}
