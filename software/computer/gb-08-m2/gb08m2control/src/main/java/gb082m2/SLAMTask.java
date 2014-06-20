package gb082m2;

import java.util.Random;

import javafx.application.Platform;
import javafx.scene.shape.Rectangle;
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
        //TODO: temporary
        public static int num = 0;
        Rectangle point;

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
                        Random rn = new Random();
                        int x = Math.abs(rn.nextInt() % 2000);
                        int y = Math.abs(rn.nextInt() % 2000);
                        point = new Rectangle(x, y, 1, 1);
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                MainAppController.automatedControlSLAMPane.getChildren().add(point);
                            }
                        });
                        System.out.println(num++);
                        Thread.sleep(25);
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
