package gb082m2;

import java.util.Random;

import javafx.application.Platform;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import application.MainAppController;

public class SLAMTask
{
    public SLAMresultsVisualizationTask slamResultsVisualizationTask;

    public SLAMTask()
    {
        slamResultsVisualizationTask = new SLAMresultsVisualizationTask();
    }

    public static class SLAMresultsVisualizationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;
        boolean isPaused = true;
        Canvas slamResultsCanvas;
        GraphicsContext slamResultsCanvasGraphicsContext;
        final int slamResultsCanvasWidth = 2000;
        final int slamResultsCanvasHeight = 2000;
        //TODO: temporary
        Circle circle;

        public SLAMresultsVisualizationTask()
        {
            new Thread(this).start();
            MainAppController.automatedControlSLAMPane.setMinWidth(slamResultsCanvasWidth);
            MainAppController.automatedControlSLAMPane.setMinHeight(slamResultsCanvasHeight);

            slamResultsCanvas = new Canvas(slamResultsCanvasWidth, slamResultsCanvasHeight);
            slamResultsCanvas.cacheProperty().set(false);
            slamResultsCanvasGraphicsContext = slamResultsCanvas.getGraphicsContext2D();
            slamResultsCanvasGraphicsContext.setFill(Color.BLACK);

            circle = new Circle(30);
            circle.setFill(Color.RED);
            MainAppController.automatedControlSLAMPane.getChildren().add(slamResultsCanvas);
            MainAppController.automatedControlSLAMPane.getChildren().add(circle);
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
                                circle.setCenterX(x);
                                circle.setCenterY(y);
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
