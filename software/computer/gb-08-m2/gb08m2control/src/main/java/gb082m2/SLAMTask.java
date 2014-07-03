package gb082m2;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.paint.Color;
import javafx.scene.shape.Arc;
import javafx.scene.shape.ArcType;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import javafx.scene.transform.Rotate;
import application.MainAppController;

public class SLAMTask
{
    final static int slamResultsCanvasWidth = 2000;
    final static int slamResultsCanvasHeight = 2000;
    List<Line> gridLines = new ArrayList<Line>();

    static Canvas slamResultsCanvas;
    static GraphicsContext slamResultsCanvasGraphicsContext;

    final static ImageView robot = new ImageView();
    static double robotWidth;
    static double robotWidthSQR;
    static double robotLength;
    static double robotCenterShiftX;
    static double robotCenterShiftY;

    static int startXPosition;
    static int startYPosition;

    static double teta = 0;
    static double dteta = 0;

    static Circle centerMarker;
    static Line scanLine;
    static Circle circumcircle;

    static double realCircumcircleRadius;

    static Arc distanceScannerArea;

    public SLAMresultsVisualizationTask slamResultsVisualizationTask;

    public SLAMTask()
    {
        MainAppController.automatedControlSLAMPane.setMinWidth(slamResultsCanvasWidth);
        MainAppController.automatedControlSLAMPane.setMinHeight(slamResultsCanvasHeight);

        slamResultsCanvas = new Canvas(slamResultsCanvasWidth, slamResultsCanvasHeight);
        slamResultsCanvas.cacheProperty().set(false);
        slamResultsCanvasGraphicsContext = slamResultsCanvas.getGraphicsContext2D();
        slamResultsCanvasGraphicsContext.setFill(Color.BLACK);

        MainAppController.automatedControlSLAMPane.getChildren().add(slamResultsCanvas);

        robot.setImage(new Image("application/robot.png"));
        startXPosition = slamResultsCanvasWidth / 2;
        startYPosition = slamResultsCanvasHeight / 2;
        drawSLAMGrid();

        centerMarker = new Circle(startXPosition, startYPosition, 5);
        centerMarker.setFill(Color.RED);
        MainAppController.automatedControlSLAMPane.getChildren().add(centerMarker);

        MainAppController.automatedControlSLAMPane.getChildren().add(robot);
        robotWidth = robot.getImage().getWidth();
        robotWidthSQR = Math.pow(robotWidth, 2);
        robotLength = robot.getImage().getHeight();
        robotCenterShiftX = robotWidth / 2;
        robotCenterShiftY = robotLength / 2;

        realCircumcircleRadius = Math.floor(Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)) / 2);

        robot.setX(startXPosition - robotCenterShiftX);
        robot.setY(startYPosition - robotCenterShiftY);

        circumcircle = new Circle(startXPosition, startYPosition, 40);
        circumcircle.setFill(Color.TRANSPARENT);
        circumcircle.setStroke(Color.GREEN);
        circumcircle.getStrokeDashArray().addAll(2d);
        MainAppController.automatedControlSLAMPane.getChildren().add(circumcircle);

        distanceScannerArea = new Arc(startXPosition, startYPosition - robotCenterShiftY, 50, 50, 25, 130);
        distanceScannerArea.setType(ArcType.ROUND);
        distanceScannerArea.setStroke(Color.TRANSPARENT);
        distanceScannerArea.setFill(Color.GREY);
        distanceScannerArea.setOpacity(0.2);
        MainAppController.automatedControlSLAMPane.getChildren().add(distanceScannerArea);

        scanLine = new Line(startXPosition, startYPosition - robotCenterShiftY, startXPosition, startYPosition - robotCenterShiftY - 10);
        scanLine.getStrokeDashArray().addAll(3d);
        scanLine.setOpacity(0.5);
        MainAppController.automatedControlSLAMPane.getChildren().add(scanLine);

        slamResultsVisualizationTask = new SLAMresultsVisualizationTask();
    }

    public void drawSLAMGrid()
    {
        for (int i = 0; i < gridLines.size(); i++)
        {
            MainAppController.automatedControlSLAMPane.getChildren().remove(gridLines.get(i));
        }
        gridLines.clear();
        int gridY = 0;
        while (gridY < slamResultsCanvasHeight)
        {
            Line gridLine = new Line(0, gridY, slamResultsCanvasWidth, gridY);
            gridLine.setStroke(Color.GREEN);
            gridLine.setStrokeWidth(0.1);
            gridLines.add(gridLine);
            MainAppController.automatedControlSLAMPane.getChildren().add(gridLine);
            gridY += 10;
        }
        int gridX = 0;
        while (gridX < slamResultsCanvasWidth)
        {
            Line gridLine = new Line(gridX, 0, gridX, slamResultsCanvasHeight);
            gridLine.setStroke(Color.GREEN);
            gridLine.setStrokeWidth(0.1);
            gridLines.add(gridLine);
            MainAppController.automatedControlSLAMPane.getChildren().add(gridLine);
            gridX += 10;
        }
    }

    public static void setRobotCenterXYDisplacement(double dx, double dy)
    {
        double centerX = circumcircle.getCenterX() - dx * Math.sin(teta);
        double centerY = circumcircle.getCenterY() - dy * Math.cos(teta);

        slamResultsCanvasGraphicsContext.setStroke(Color.BROWN);
        slamResultsCanvasGraphicsContext.setLineWidth(2);
        slamResultsCanvasGraphicsContext.strokeLine(circumcircle.getCenterX(), circumcircle.getCenterY(), centerX, centerY);

        circumcircle.setCenterX(centerX);
        circumcircle.setCenterY(centerY);

        robot.getTransforms().clear();
        robot.setX(circumcircle.getCenterX() - robotCenterShiftX);
        robot.setY(circumcircle.getCenterY() - robotCenterShiftY);
        robot.getTransforms().add(new Rotate(-teta * 180 / Math.PI, circumcircle.getCenterX(), circumcircle.getCenterY(), 0, Rotate.Z_AXIS));

        distanceScannerArea.getTransforms().clear();
        distanceScannerArea.setCenterX(circumcircle.getCenterX());
        distanceScannerArea.setCenterY(circumcircle.getCenterY() - robotLength / 2);
        distanceScannerArea.getTransforms().add(new Rotate(-teta * 180 / Math.PI, circumcircle.getCenterX(), circumcircle.getCenterY(), 0, Rotate.Z_AXIS));
    }

    public static double getRobotX()
    {
        if (robot != null)
        {
            return robot.getX();
        } else
        {
            return startXPosition;
        }
    }

    public static double getRobotY()
    {
        if (robot != null)
        {
            return robot.getY();
        } else
        {
            return startYPosition;
        }
    }

    public static class SLAMresultsVisualizationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;
        volatile double prevDistanceScannerAngle;
        volatile int prevLeftEncoderTicks = 0;
        volatile int prevRightEncoderTicks = 0;

        public SLAMresultsVisualizationTask()
        {
            new Thread(this).start();
        }

        public void pause()
        {
            isPaused = true;
        }

        public void resume()
        {
            isPaused = false;
        }

        @Override
        synchronized public void run()
        {
            while (MainAppController.isActive && !isStopped)
            {
                try
                {
                    if (!isPaused)
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                if (prevLeftEncoderTicks == 0)
                                {
                                    prevLeftEncoderTicks = GB08M2.getInstance().getLeftEncoderTicks();
                                }
                                if (prevRightEncoderTicks == 0)
                                {
                                    prevRightEncoderTicks = GB08M2.getInstance().getRightEncoderTicks();
                                }

                                if (GB08M2.getInstance().getLeftEncoderTicks() != prevLeftEncoderTicks || GB08M2.getInstance().getRightEncoderTicks() != prevRightEncoderTicks)
                                {
                                    double nl = (GB08M2.getInstance().getLeftEncoderTicks() - prevLeftEncoderTicks) * 0.87;
                                    double nr = (GB08M2.getInstance().getRightEncoderTicks() - prevRightEncoderTicks) * 0.87;
                                    double dn = (nl + nr) / 2;
                                    double al = Math.asin(nl / Math.sqrt(Math.pow(nl, 2) + robotWidthSQR)) * 180 / Math.PI;
                                    double ar = Math.asin(nr / Math.sqrt(Math.pow(nr, 2) + robotWidthSQR)) * 180 / Math.PI;

                                    dteta = 0;

                                    //rotation about center
                                    if ((GB08M2.getInstance().getLeftMotorsDirection() == "forward" && GB08M2.getInstance().getRightMotorsDirection() == "backward") || (GB08M2.getInstance().getLeftMotorsDirection() == "backward" && GB08M2.getInstance().getRightMotorsDirection() == "forward"))
                                    {
                                        dteta = al + ar;
                                        //cw rotation
                                        if (GB08M2.getInstance().getLeftMotorsDirection() == "forward" && GB08M2.getInstance().getRightMotorsDirection() == "backward")
                                        {
                                            teta -= dteta * Math.PI / 180;
                                        }
                                        //ccw rotation
                                        if (GB08M2.getInstance().getLeftMotorsDirection() == "backward" && GB08M2.getInstance().getRightMotorsDirection() == "forward")
                                        {
                                            teta += dteta * Math.PI / 180;
                                        }
                                        if (nl >= nr)
                                        {
                                            setRobotCenterXYDisplacement(nl - nr, nl - nr);
                                        } else if (nl < nr)
                                        {
                                            setRobotCenterXYDisplacement(nr - nl, nr - nl);
                                        }
                                    } else
                                    //one side movement
                                    {
                                        if (GB08M2.getInstance().getLeftMotorsDirection() == "forward" || GB08M2.getInstance().getRightMotorsDirection() == "forward")
                                        {
                                            if (nl >= nr)
                                            {
                                                dteta = al - ar;
                                                if (GB08M2.getInstance().getLeftMotorsDirection() == GB08M2.getInstance().getRightMotorsDirection())
                                                {
                                                    teta += dteta * Math.PI / 180;
                                                } else
                                                {
                                                    teta -= dteta * Math.PI / 180;
                                                }
                                            } else if (nr > nl)
                                            {
                                                dteta = ar - al;
                                                if (GB08M2.getInstance().getLeftMotorsDirection() == GB08M2.getInstance().getRightMotorsDirection())
                                                {
                                                    teta -= dteta * Math.PI / 180;
                                                } else
                                                {
                                                    teta += dteta * Math.PI / 180;
                                                }
                                            }
                                            setRobotCenterXYDisplacement(dn, dn);
                                        }

                                        if (GB08M2.getInstance().getLeftMotorsDirection() == "backward" || GB08M2.getInstance().getRightMotorsDirection() == "backward")
                                        {
                                            if (nl >= nr)
                                            {
                                                dteta = al - ar;
                                                if (GB08M2.getInstance().getLeftMotorsDirection() == GB08M2.getInstance().getRightMotorsDirection())
                                                {
                                                    teta -= dteta * Math.PI / 180;
                                                } else
                                                {
                                                    teta += dteta * Math.PI / 180;
                                                }
                                            } else if (nr > nl)
                                            {
                                                dteta = ar - al;
                                                if (GB08M2.getInstance().getLeftMotorsDirection() == GB08M2.getInstance().getRightMotorsDirection())
                                                {
                                                    teta += dteta * Math.PI / 180;
                                                } else
                                                {
                                                    teta -= dteta * Math.PI / 180;
                                                }
                                            }
                                            setRobotCenterXYDisplacement(-dn, -dn);
                                        }
                                    }

                                    prevLeftEncoderTicks = GB08M2.getInstance().getLeftEncoderTicks();
                                    prevRightEncoderTicks = GB08M2.getInstance().getRightEncoderTicks();
                                }

                                double curDistanceScannerAngle = GB08M2.getInstance().getDistanceScannerPositionAngle();
                                if (curDistanceScannerAngle > 0)
                                {
                                    int distance = GB08M2.getInstance().getDistanceScannerDistance_cm();
                                    scanLine.setVisible(true);
                                    double curScanLineAngle = (-115 - curDistanceScannerAngle) * Math.PI / 180 + teta;
                                    double startX = circumcircle.getCenterX() - (robotLength / 2) * Math.sin(teta);
                                    double startY = circumcircle.getCenterY() - (robotLength / 2) * Math.cos(teta);
                                    double endX = startX + distance * Math.sin(curScanLineAngle);
                                    double endY = startY + distance * Math.cos(curScanLineAngle);
                                    scanLine.setStartX(startX);
                                    scanLine.setStartY(startY);
                                    scanLine.setEndX(endX);
                                    scanLine.setEndY(endY);

                                    if (curDistanceScannerAngle != prevDistanceScannerAngle)
                                    {
                                        prevDistanceScannerAngle = GB08M2.getInstance().getDistanceScannerPositionAngle();
                                        if (distance <= 50)
                                        {
                                            slamResultsCanvasGraphicsContext.fillRect(endX, endY, 2, 2);
                                        }
                                    }
                                } else
                                {
                                    scanLine.setVisible(false);
                                }
                            }
                        });
                        Thread.sleep(GB08M2.distanceScannerPositioningDelay);
                    } else
                    {
                        Thread.sleep(200);
                    }
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
