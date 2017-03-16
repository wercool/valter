package gb082m2;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javafx.application.Platform;
import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.scene.shape.Line;
import javafx.scene.shape.Polyline;
import javafx.scene.shape.Rectangle;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import utils.JPEGFrameGrabber;
import application.MainAppController;

public class GB08M2AutomatedManager
{
    public SLAMTask slamTask;

    public static double startROIx = 0;
    public static double startROIy = 0;
    public static double endROIx = 0;
    public static double endROIy = 0;

    public static double curMouseX = 0;
    public static double curMouseY = 0;

    static boolean patternTracking = false;

    public static Line horizontLine;
    public static Line navigationLineLeft;
    public static Line navigationLineRight;
    public static ArrayList<Point> ROIDisplacementTrajectoryPoints;
    static Point ROIMatchPrevLoc;
    public static Polyline ROIMatchDisplacementPolyLine;
    public static Rectangle detectedROIRectangle;

    static FrontCameraCVVizualisationTask frontCameraCVVizualisationTask;
    FrontCameraNavigationVizualisationTask frontCameraNavigationVizualisationTask;

    public GB08M2AutomatedManager()
    {
        horizontLine = new Line(0, 0, 0, 0);
        horizontLine.setStroke(Color.YELLOW);
        horizontLine.setStrokeWidth(2);
        horizontLine.setOpacity(0.25);
        MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(horizontLine);

        navigationLineLeft = new Line(0, 0, 0, 0);
        navigationLineLeft.setStroke(Color.AQUAMARINE);
        navigationLineLeft.setStrokeWidth(1);
        MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(navigationLineLeft);

        navigationLineRight = new Line(0, 0, 0, 0);
        navigationLineRight.setStroke(Color.AQUAMARINE);
        navigationLineRight.setStrokeWidth(1);
        MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(navigationLineRight);

        ROIMatchDisplacementPolyLine = new Polyline();
        ROIMatchDisplacementPolyLine.setStroke(Color.BLUE);
        MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(ROIMatchDisplacementPolyLine);

        detectedROIRectangle = new Rectangle();
        detectedROIRectangle.setStroke(Color.LIME);
        detectedROIRectangle.setStrokeWidth(2);
        detectedROIRectangle.setFill(Color.TRANSPARENT);
        MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(detectedROIRectangle);

        ROIDisplacementTrajectoryPoints = new ArrayList<Point>();

        slamTask = new SLAMTask();
    }

    public void deinitialize()
    {
        stopFrontCameraCVVizualisation();
        stopFrontCameraNavigationVizualisationTask();
    }

    public void startFrontCameraVizualisation()
    {
        frontCameraCVVizualisationTask = new FrontCameraCVVizualisationTask();
        frontCameraNavigationVizualisationTask = new FrontCameraNavigationVizualisationTask();
    }

    public void stopFrontCameraCVVizualisation()
    {
        if (frontCameraCVVizualisationTask != null)
        {
            frontCameraCVVizualisationTask.stop();
        }
    }

    public void stopFrontCameraNavigationVizualisationTask()
    {
        if (frontCameraNavigationVizualisationTask != null)
        {
            frontCameraNavigationVizualisationTask.stop();
        }
    }

    public static synchronized boolean isPatternTracking()
    {
        return patternTracking;
    }

    public static synchronized void setPatternTracking(boolean patternTracking)
    {
        if (!patternTracking)
        {
            ROIDisplacementTrajectoryPoints.clear();
            ROIMatchPrevLoc = null;
        }
        GB08M2AutomatedManager.patternTracking = patternTracking;
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

                                    MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().remove(5, MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().size());
                                    MainAppController.instance.frontCameraAutomatedControlOverlayPane.getChildren().add(GB08M2.getInstance().getFrontCameraROIRectangle());

                                    if (GB08M2.getInstance().getFrontCameraROIRectangle().getWidth() > 0 && GB08M2.getInstance().getFrontCameraROIRectangle().getHeight() > 0)
                                    {
                                        Mat ROIMat = GB08M2.getInstance().getFrontCameraROIMat();
                                        Mat ROIGrayscaleMat = GB08M2.getInstance().getFrontCameraROIGrayscaleWithSURFMat();
                                        if (ROIMat != null && ROIGrayscaleMat != null)
                                        {
                                            if (ROIMat.width() > 0 && ROIMat.height() > 0)
                                            {
                                                BufferedImage selectedROIBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROIMat);
                                                Image ROIImage = SwingFXUtils.toFXImage(selectedROIBufferedImage, null);
                                                MainAppController.instance.frontCameraAutomatedROIImageView.setFitWidth(ROIImage.getWidth());
                                                MainAppController.instance.frontCameraAutomatedROIImageView.setFitHeight(ROIImage.getHeight());
                                                MainAppController.instance.frontCameraAutomatedROIImageView.setImage(ROIImage);
                                                MainAppController.instance.frontCameraAutomatedROIImageView.setCache(false);

                                                BufferedImage selectedROIGrayscakeBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROIGrayscaleMat);
                                                Image ROIGrayscaleImage = SwingFXUtils.toFXImage(selectedROIGrayscakeBufferedImage, null);
                                                MainAppController.instance.frontCameraAutomatedROIGrayscaleImageView.setFitWidth(ROIGrayscaleImage.getWidth());
                                                MainAppController.instance.frontCameraAutomatedROIGrayscaleImageView.setFitHeight(ROIGrayscaleImage.getHeight());
                                                MainAppController.instance.frontCameraAutomatedROIGrayscaleImageView.setImage(ROIGrayscaleImage);
                                                MainAppController.instance.frontCameraAutomatedROIGrayscaleImageView.setCache(false);
                                            }
                                        }
                                    }
                                }
                            }
                        });
                        if (GB08M2.getInstance().getFrontCameraROIRectangle() != null)
                        {
                            GB08M2.getInstance().getFrontCameraROIRectangle().setVisible(true);
                        }
                    } else
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                if (GB08M2.getInstance().getFrontCameraROIRectangle() != null)
                                {
                                    Mat ROIDetectedMat = GB08M2.getInstance().getFrontCameraROIDetectedMat();
                                    Mat ROIDetectedGrayscaleMat = GB08M2.getInstance().getFrontCameraROIDetectedGrayscaleMat();
                                    if (ROIDetectedMat != null && ROIDetectedGrayscaleMat != null)
                                    {
                                        if (ROIDetectedMat.width() > 0 && ROIDetectedMat.height() > 0)
                                        {
                                            BufferedImage selectedROIBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROIDetectedMat);
                                            Image ROIDetectedImage = SwingFXUtils.toFXImage(selectedROIBufferedImage, null);
                                            MainAppController.instance.frontCameraAutomatedROIDetectedImageView.setFitWidth(ROIDetectedImage.getWidth());
                                            MainAppController.instance.frontCameraAutomatedROIDetectedImageView.setFitHeight(ROIDetectedImage.getHeight());
                                            MainAppController.instance.frontCameraAutomatedROIDetectedImageView.setImage(ROIDetectedImage);
                                            MainAppController.instance.frontCameraAutomatedROIDetectedImageView.setCache(false);

                                            BufferedImage selectedROIGrayscaleBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROIDetectedGrayscaleMat);
                                            Image ROIDetectedGrayscaleImage = SwingFXUtils.toFXImage(selectedROIGrayscaleBufferedImage, null);
                                            MainAppController.instance.frontCameraAutomatedROIDetectedGrayscaleImageView.setFitWidth(ROIDetectedGrayscaleImage.getWidth());
                                            MainAppController.instance.frontCameraAutomatedROIDetectedGrayscaleImageView.setFitHeight(ROIDetectedGrayscaleImage.getHeight());
                                            MainAppController.instance.frontCameraAutomatedROIDetectedGrayscaleImageView.setImage(ROIDetectedGrayscaleImage);
                                            MainAppController.instance.frontCameraAutomatedROIDetectedGrayscaleImageView.setCache(false);

                                            Mat frontCameraSURFMatchesMat = GB08M2.getInstance().getFrontCameraSURFMatchesMat();
                                            if (frontCameraSURFMatchesMat != null)
                                            {
                                                BufferedImage frontCameraSURFMatchesMatBufferedImage = JPEGFrameGrabber.matToBufferedImage(frontCameraSURFMatchesMat);
                                                Image frontCameraSURFMatchesMatImage = SwingFXUtils.toFXImage(frontCameraSURFMatchesMatBufferedImage, null);
                                                MainAppController.instance.frontCameraAutomatedSURFMatchesImageView.setFitWidth(frontCameraSURFMatchesMatImage.getWidth());
                                                MainAppController.instance.frontCameraAutomatedSURFMatchesImageView.setFitHeight(frontCameraSURFMatchesMatImage.getHeight());
                                                MainAppController.instance.frontCameraAutomatedSURFMatchesImageView.setImage(frontCameraSURFMatchesMatImage);
                                                MainAppController.instance.frontCameraAutomatedSURFMatchesImageView.setCache(false);
                                            }
                                            
                                            Mat ROISURFMatchesWithTemplateMat = GB08M2.getInstance().getFrontCameraDetectedROISURFMatchesWithSelectedTemplateMat();
                                            if (ROISURFMatchesWithTemplateMat != null)
                                            {
                                                BufferedImage ROISURFMatchesWithTemplateMatBufferedImage = JPEGFrameGrabber.matToBufferedImage(ROISURFMatchesWithTemplateMat);
                                                Image ROISURFMatchesWithTemplateMatImage = SwingFXUtils.toFXImage(ROISURFMatchesWithTemplateMatBufferedImage, null);
                                                MainAppController.instance.frontCameraAutomatedROIMatchWithTemplareImageView.setFitWidth(ROISURFMatchesWithTemplateMatImage.getWidth());
                                                MainAppController.instance.frontCameraAutomatedROIMatchWithTemplareImageView.setFitHeight(ROISURFMatchesWithTemplateMatImage.getHeight());
                                                MainAppController.instance.frontCameraAutomatedROIMatchWithTemplareImageView.setImage(ROISURFMatchesWithTemplateMatImage);
                                                MainAppController.instance.frontCameraAutomatedROIMatchWithTemplareImageView.setCache(false);
                                            }

                                            Point ROIMatchLoc = GB08M2.getInstance().getROIMatchLoc();

                                            detectedROIRectangle.setX(ROIMatchLoc.x);
                                            detectedROIRectangle.setY(ROIMatchLoc.y);
                                            detectedROIRectangle.setWidth(ROIDetectedGrayscaleImage.getWidth());
                                            detectedROIRectangle.setHeight(ROIDetectedGrayscaleImage.getHeight());
                                        }
                                    }

                                    GB08M2.getInstance().getFrontCameraROIRectangle().setVisible(false);
                                }
                            }
                        });
                    }

                    BufferedImage frameBufferedImage = JPEGFrameGrabber.matToBufferedImage(GB08M2.getInstance().getFrontCameraMat());
                    Image frame = SwingFXUtils.toFXImage(frameBufferedImage, null);
                    MainAppController.instance.frontCameraAutomatedControlImageView.setFitWidth(frame.getWidth());
                    MainAppController.instance.frontCameraAutomatedControlImageView.setFitHeight(frame.getHeight());
                    MainAppController.instance.frontCameraAutomatedControlImageView.setImage(frame);
                    MainAppController.instance.frontCameraAutomatedControlImageView.setCache(false);

                    horizontLine.setEndX(MainAppController.instance.frontCameraAutomatedControlImageView.getFitWidth());
                    horizontLine.setStartY(MainAppController.instance.frontCameraAutomatedControlImageView.getFitHeight() / 2.5);
                    horizontLine.setEndY(horizontLine.getStartY());

                    Thread.sleep(50);

                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    public class FrontCameraNavigationVizualisationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;

        public FrontCameraNavigationVizualisationTask()
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
                    navigationLineLeft.setStartX((MainAppController.instance.frontCameraAutomatedControlImageView.getFitWidth() / 2) - 100);
                    navigationLineRight.setStartX((MainAppController.instance.frontCameraAutomatedControlImageView.getFitWidth() / 2) + 100);
                    navigationLineLeft.setStartY(MainAppController.instance.frontCameraAutomatedControlImageView.getFitHeight());
                    navigationLineRight.setStartY(MainAppController.instance.frontCameraAutomatedControlImageView.getFitHeight());

                    if (MainAppController.instance.frontCameraAutomatedControlImageView.getImage() != null)
                    {
                        if (curMouseX < MainAppController.instance.frontCameraAutomatedControlImageView.getImage().getWidth() && curMouseY < MainAppController.instance.frontCameraAutomatedControlImageView.getImage().getHeight())
                        {
                            navigationLineLeft.setEndX(curMouseX);
                            navigationLineRight.setEndX(curMouseX);
                            navigationLineLeft.setEndY(curMouseY);
                            navigationLineRight.setEndY(curMouseY);
                        }
                    }

                    if (isPatternTracking())
                    {
                        if (GB08M2.getInstance().getROIMatchLoc() != null)
                        {
                            if (ROIMatchPrevLoc == null)
                            {
                                ROIMatchPrevLoc = GB08M2.getInstance().getROIMatchLoc();
                            }
                            if (ROIMatchPrevLoc.x != GB08M2.getInstance().getROIMatchLoc().x || ROIMatchPrevLoc.y != GB08M2.getInstance().getROIMatchLoc().y)
                            {
                                ROIMatchPrevLoc = GB08M2.getInstance().getROIMatchLoc();
                                ROIDisplacementTrajectoryPoints.add(GB08M2.getInstance().getROIMatchLoc());
                                //System.out.println("ROI location has changed [" + GB08M2.getInstance().getROIMatchLoc() + "]");
                            }
                            ROIMatchDisplacementPolyLine.getPoints().clear();

                            //                            for (Point p : ROIDisplacementTrajectoryPoints)
                            //                            {
                            //                                if (p != null)
                            //                                {
                            //                                    ROIMatchDisplacementPolyLine.getPoints().addAll(new Double[] { p.x, p.y });
                            //                                }
                            //                            }
                        }
                    }

                    Thread.sleep(50);

                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }

    }
}
