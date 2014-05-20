package app;

import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;

import org.opencv.core.Core;

public class OpenCVApplicationController
{

    @FXML
    ImageView frontCameraImageView;
    @FXML
    Button startFrontCameraStreamBtn;

    String frontCameraMJPGStreamURL = "http://109.87.34.156:8081/?action=stream&dummyparam=dummy.mjpg";

    MJPGStreamGrabber frontCameraMJPGStream;

    public OpenCVApplicationController()
    {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    @FXML
    void initialize()
    {
        frontCameraMJPGStream = new MJPGStreamGrabber(frontCameraMJPGStreamURL, frontCameraImageView);
        fxmlComponentHandlersInit();
    }

    void close()
    {
        frontCameraMJPGStream.stopCapture();
    }

    void fxmlComponentHandlersInit()
    {
        startFrontCameraStreamBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (!frontCameraMJPGStream.isCapturing)
                {
                    frontCameraMJPGStream.startCapture();
                    startFrontCameraStreamBtn.setText("Stop Front Camera Stream");
                } else
                {
                    frontCameraMJPGStream.stopCapture();
                    startFrontCameraStreamBtn.setText("Start Front Camera Stream");
                }
            }
        });
    }
}
