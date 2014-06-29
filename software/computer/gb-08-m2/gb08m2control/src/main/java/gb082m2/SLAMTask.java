package gb082m2;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
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
    static double robotLength;
    static double robotCenterShiftX;
    static double robotCenterShiftY;

    static int startXPosition;
    static int startYPosition;

    static Line scanLine;

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

        Circle center = new Circle(startXPosition, startYPosition, 5);
        center.setFill(Color.RED);
        MainAppController.automatedControlSLAMPane.getChildren().add(center);

        MainAppController.automatedControlSLAMPane.getChildren().add(robot);
        robotWidth = robot.getImage().getWidth();
        robotLength = robot.getImage().getHeight();
        robotCenterShiftX = robotWidth / 2;
        robotCenterShiftY = robotLength / 2;

        robot.setX(startXPosition - robotCenterShiftX);
        robot.setY(startYPosition - robotCenterShiftY);

        scanLine = new Line(startXPosition, startYPosition - robotCenterShiftY, startXPosition, startYPosition - robotCenterShiftY - 10);
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

    public static double getRobotCenterX()
    {
        return robot.getX();
    }

    public static double getRobotCenterY()
    {
        return robot.getY();
    }

    public static void setRobotCenterXY(double x, double y)
    {
        robot.setX(x);
        robot.setY(y);
    }

    public static void setRobotCenterXYDisplacement(double dx, double dy)
    {
        robot.setX(robot.getX() + dx);
        robot.setY(robot.getY() + dy);
    }

    public static double getRobotDistanceScannerX()
    {
        return robot.getX() + robotCenterShiftX;
    }

    public static double getRobotDistanceScannerY()
    {
        return robot.getY();
    }

    public static class SLAMresultsVisualizationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;
        volatile double prevAngle;
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

                                if (GB08M2.getInstance().getLeftEncoderTicks() > prevLeftEncoderTicks || GB08M2.getInstance().getRightEncoderTicks() > prevRightEncoderTicks)
                                {
                                    if (GB08M2.getInstance().getLeftMotorsDirection() == "forward" && GB08M2.getInstance().getRightMotorsDirection() == "forward")
                                    {
                                        setRobotCenterXYDisplacement(0, -((GB08M2.getInstance().getLeftEncoderTicks() - prevLeftEncoderTicks) * 0.87));
                                    }
                                    if (GB08M2.getInstance().getLeftMotorsDirection() == "backward" && GB08M2.getInstance().getRightMotorsDirection() == "backward")
                                    {
                                        setRobotCenterXYDisplacement(0, ((GB08M2.getInstance().getLeftEncoderTicks() - prevLeftEncoderTicks) * 0.87));
                                    }

                                    if (GB08M2.getInstance().getLeftEncoderTicks() > prevLeftEncoderTicks)
                                    {
                                        prevLeftEncoderTicks = GB08M2.getInstance().getLeftEncoderTicks();
                                    }
                                    if (GB08M2.getInstance().getRightEncoderTicks() > prevRightEncoderTicks)
                                    {
                                        prevRightEncoderTicks = GB08M2.getInstance().getRightEncoderTicks();
                                    }
                                }

                                //robot.getTransforms().add(new Rotate(1, startXPosition - robotCenterShiftX, startYPosition - robotCenterShiftY, 0, Rotate.Z_AXIS));
                                int distance = GB08M2.getInstance().getDistanceScannerDistance_cm();
                                double curAngle = GB08M2.getInstance().getDistanceScannerPositionAngle();
                                double curScanLineAngle = (-115 - curAngle) * Math.PI / 180;
                                double endX = getRobotDistanceScannerX() + (distance / 2) * Math.sin(curScanLineAngle);
                                double endY = getRobotDistanceScannerY() + (distance / 2) * Math.cos(curScanLineAngle);
                                scanLine.setStartX(getRobotDistanceScannerX());
                                scanLine.setStartY(getRobotDistanceScannerY());
                                scanLine.setEndX(endX);
                                scanLine.setEndY(endY);
                                if (curAngle != prevAngle)
                                {
                                    prevAngle = GB08M2.getInstance().getDistanceScannerPositionAngle();
                                    if (distance < 50)
                                    {
                                        slamResultsCanvasGraphicsContext.fillRect(endX, endY, 2, 2);
                                    }
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
