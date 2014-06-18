package gb082m2;

import javafx.application.Platform;
import application.MainAppController;

public class GB08M2AutomatedManager
{
    class DistanceReadingsVisualizationTask implements Runnable
    {
        Thread thread;

        public DistanceReadingsVisualizationTask()
        {
            new Thread(this).start();
        }

        @Override
        synchronized public void run()
        {
            while (MainAppController.isActive)
            {
                try
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {

                        }
                    });
                    Thread.sleep(25);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
