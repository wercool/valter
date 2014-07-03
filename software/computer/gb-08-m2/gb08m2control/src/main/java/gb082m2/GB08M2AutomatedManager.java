package gb082m2;

import java.awt.image.BufferedImage;

import javafx.application.Platform;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.shape.Rectangle;

import org.opencv.core.Mat;

import utils.JPEGFrameGrabber;
import application.MainAppController;

public class GB08M2AutomatedManager
{
    public SLAMTask slamTask;

    public static double startROIx = 0;
    public static double startROIy = 0;
    public static double endROIx = 0;
    public static double endROIy = 0;

    public static boolean patternTracking = false;

    FrontCameraCVVizualisationTask frontCameraCVVizualisationTask;

    public GB08M2AutomatedManager()
    {
        slamTask = new SLAMTask();
    }

    public void deinitialize()
    {
        if (frontCameraCVVizualisationTask != null)
        {
            frontCameraCVVizualisationTask.stop();
        }
    }

    public void startFrontCameraCVVizualisation()
    {
        frontCameraCVVizualisationTask = new FrontCameraCVVizualisationTask();
    }

    public void stopFrontCameraCVVizualisation()
    {
        if (frontCameraCVVizualisationTask != null)
        {
            frontCameraCVVizualisationTask.stop();
        }
    }

    public class FrontCameraCVVizualisationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;

        public FrontCameraCVVizualisationTask()
        {
            new Thread(this).start();
        }

        public void stop()
        {
            isStopped = true;
        }

        public void pause()
        {
            isPaused = true;
        }

        public boolean isPaused()
        {
            return isPaused;
        }

        public void resume()
        {
            isPaused = false;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                try
                {

                    if (!patternTracking)
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                double ROIWidth = endROIx - startROIx;
                                double ROIHeight = endROIy - startROIy;
                                if (ROIWidth > 0 && ROIHeight > 0)
                                {
                                    Rectangle ROI = new Rectangle(startROIx, startROIy, ROIWidth, ROIHeight);
                                    ROI.setStrokeWidth(2);
                                    ROI.setStroke(Color.RED);
                                    ROI.setFill(Color.TRANSPARENT);
                                    ROI.setMouseTransparent(true);
                                    GB08M2.getInstance().setFrontCameraROIRectangle(ROI);
                                    MainAppController.frontCameraAutomatedControlOverlayPane.getChildren().clear();
                                    MainAppController.frontCameraAutomatedControlOverlayPane.getChildren().addAll(GB08M2.getInstance().getFrontCameraROIRectangle());

                                    if (GB08M2.getInstance().getFrontCameraROIRectangle().getWidth() > 0 && GB08M2.getInstance().getFrontCameraROIRectangle().getHeight() > 0)
                                    {
                                        Mat ROIMat = GB08M2.getInstance().getFrontCameraROIMat();
                                        if (ROIMat != null)
                                        {
                                            if (ROIMat.width() > 0 && ROIMat.height() > 0)
                                            {
                                                BufferedImage selectedROIBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROIMat);
                                                Image ROIImage = SwingFXUtils.toFXImage(selectedROIBufferedImage, null);
                                                MainAppController.frontCameraAutomatedROIImageView.setFitWidth(ROIImage.getWidth());
                                                MainAppController.frontCameraAutomatedROIImageView.setFitHeight(ROIImage.getHeight());
                                                MainAppController.frontCameraAutomatedROIImageView.setImage(ROIImage);
                                                MainAppController.frontCameraAutomatedROIImageView.setCache(false);
                                            }
                                        }
                                    }
                                }
                            }
                        });
                    }
                    else
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                if (MainAppController.frontCameraAutomatedControlOverlayPane.getChildren().contains(GB08M2.getInstance().getFrontCameraROIRectangle()))
                                {
                                    MainAppController.frontCameraAutomatedControlOverlayPane.getChildren().remove(GB08M2.getInstance().getFrontCameraROIRectangle());
                                }
                            }
                        });
                    }

                    BufferedImage frameBufferedImage = JPEGFrameGrabber.matToBufferedImage(GB08M2.getInstance().getFrontCameraMat());
                    Image frame = SwingFXUtils.toFXImage(frameBufferedImage, null);
                    MainAppController.frontCameraAutomatedControlImageView.setImage(frame);
                    MainAppController.frontCameraAutomatedControlImageView.setCache(false);

                    Thread.sleep(50);

                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }

    }
}
