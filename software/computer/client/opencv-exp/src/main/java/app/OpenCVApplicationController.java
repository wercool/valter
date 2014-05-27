package app;

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.embed.swing.SwingFXUtils;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;

import org.opencv.core.Core;

import com.github.sarxos.webcam.Webcam;
import com.github.sarxos.webcam.WebcamResolution;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.EncodeHintType;
import com.google.zxing.MultiFormatReader;
import com.google.zxing.NotFoundException;
import com.google.zxing.Result;
import com.google.zxing.client.j2se.BufferedImageLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;

public class OpenCVApplicationController
{

    @FXML
    ImageView frontCameraImageView;
    @FXML
    Button startFrontCameraStreamBtn;
    @FXML
    ComboBox<Webcam> videoDeviceListComboBox;
    @FXML
    Button connectWebCamBtn;
    @FXML
    ImageView webCamImageView;
    @FXML
    Label qrCodeDecodedLabel;

    String frontCameraMJPGStreamURL = "http://109.87.34.156:8080/?action=stream&dummyparam=dummy.mjpg";

    MJPGStreamGrabber frontCameraMJPGStream;

    List<Webcam> videoDeviceList;
    Webcam selectedVideoDevice;
    Thread webCamFrameOutputThread;
    BufferedImage webCamImage;

    public OpenCVApplicationController()
    {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    @FXML
    void initialize()
    {
        frontCameraMJPGStream = new MJPGStreamGrabber(frontCameraMJPGStreamURL, frontCameraImageView);
        fxmlComponentHandlersInit();
        initWebCamQRCodeDetection();
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

    void initWebCamQRCodeDetection()
    {
        videoDeviceList = Webcam.getWebcams();
        ObservableList<Webcam> videoDeviceObservableList = FXCollections.observableList(videoDeviceList);
        if (videoDeviceList.size() > 0)
        {
            videoDeviceListComboBox.setItems(videoDeviceObservableList);
            videoDeviceListComboBox.getSelectionModel().select(Webcam.getDefault());
        } else
        {
            videoDeviceListComboBox.getSelectionModel().selectFirst();
        }

        connectWebCamBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                selectedVideoDevice = videoDeviceListComboBox.getValue();
                connectDisconnectVideoDevice();
            }
        });
    }

    void connectDisconnectVideoDevice()
    {
        if (videoDeviceListComboBox.getValue() instanceof Webcam)
        {
            if (selectedVideoDevice != null)
            {
                if (selectedVideoDevice.isOpen())
                {
                    selectedVideoDevice.close();
                    connectWebCamBtn.setText("Connect");
                } else
                {
                    Dimension[] nonStandardResolutions = new Dimension[]
                        { WebcamResolution.VGA.getSize(), WebcamResolution.SVGA.getSize(), WebcamResolution.HD720.getSize() };

                    selectedVideoDevice.setCustomViewSizes(nonStandardResolutions);
                    selectedVideoDevice.setViewSize(WebcamResolution.VGA.getSize());

                    selectedVideoDevice.open();
                    connectWebCamBtn.setText("Disconnect");

                    webCamFrameOutputThread = new Thread()
                    {
                        @Override
                        public void run()
                        {
                            redrawWebCamImageView();
                        }
                    };
                    webCamFrameOutputThread.start();
                }
            }
        }
    }

    void redrawWebCamImageView()
    {
        Map<EncodeHintType, ErrorCorrectionLevel> hintMap = new HashMap<EncodeHintType, ErrorCorrectionLevel>();
        hintMap.put(EncodeHintType.ERROR_CORRECTION, ErrorCorrectionLevel.L);

        while (selectedVideoDevice != null && selectedVideoDevice.isOpen())
        {
            webCamImage = selectedVideoDevice.getImage();
            Image frame = null;
            frame = SwingFXUtils.toFXImage(webCamImage, null);
            webCamImageView.setImage(frame);

            BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(new BufferedImageLuminanceSource(webCamImage)));
            try
            {
                final Result qrCodeResult = new MultiFormatReader().decode(binaryBitmap);
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        qrCodeDecodedLabel.setText("QR Code Decoded: " + qrCodeResult.getText());
                    }
                });
            } catch (NotFoundException e)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        qrCodeDecodedLabel.setText("QR Code Decoded: ");
                    }
                });
                //e.printStackTrace();
            }
        }
    }
}
