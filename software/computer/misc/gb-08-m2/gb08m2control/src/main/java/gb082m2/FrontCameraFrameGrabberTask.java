package gb082m2;

import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;

import javax.imageio.ImageIO;

import application.MainAppController;
import javafx.application.Platform;

public class FrontCameraFrameGrabberTask implements Runnable
{
    Thread thread;
    URL frontCameraFramesURL;
    boolean isStopped = false;
    boolean isPaused = true;

    public FrontCameraFrameGrabberTask()
    {
        try
        {
            frontCameraFramesURL = new URL("http://" + GB08M2.getInstance().getHostname() + ":" + String.valueOf(GB08M2.getInstance().getFrontCameraPort()) + "/?action=snapshot");
        } catch (MalformedURLException e)
        {
            e.printStackTrace();
        }
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
                if (!isPaused)
                {
                    GB08M2.getInstance().setFrontCameraFrameBufferedImage(ImageIO.read(frontCameraFramesURL));
                }

                Thread.sleep(500);

            } catch (IOException | InterruptedException e)
            {
                e.printStackTrace();
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                		MainAppController.instance.frontCameraImageViewrearCameraImageViewIndicator.setProgress(-1);
                    }
                });
            }
        }
    }

}
