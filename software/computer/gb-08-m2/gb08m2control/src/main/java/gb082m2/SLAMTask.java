package gb082m2;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import javafx.application.Platform;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.paint.Color;
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
    static double robotLength;
    static double robotCenterShiftX;
    static double robotCenterShiftY;

    static int startXPosition;
    static int startYPosition;

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

    public static class SLAMresultsVisualizationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;


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
                                Random rn = new Random();
                                int x = Math.abs(rn.nextInt() % slamResultsCanvasWidth);
                                int y = Math.abs(rn.nextInt() % slamResultsCanvasHeight);
                                slamResultsCanvasGraphicsContext.fillRect(x, y, 2, 2);
                                //robot.setX(x - robotCenterShiftX);
                                //robot.setY(y - robotCenterShiftY);
                                //robot.setRotate(20);
                                robot.getTransforms().add(new Rotate(1, startXPosition, startYPosition, 0, Rotate.Z_AXIS));
                            }
                        });
                        Thread.sleep(1);
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
