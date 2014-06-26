package gb082m2;

public class DistanceScannerTask implements Runnable
{
    Thread thread;

    volatile boolean isStopped = false;

    public DistanceScannerTask()
    {
        if (GB08M2.getInstance().getDistanceScannerPositionAngle() >= 65)
        {
            GB08M2.getInstance().distanceScannerDirection = true;
        } else
        {
            GB08M2.getInstance().distanceScannerDirection = false;
        }
        thread = new Thread(this);
    }

    public void start()
    {
        thread.start();
    }

    public void stop()
    {
        isStopped = true;
    }

    public boolean isRunning()
    {
        return !isStopped;
    }

    @Override
    public void run()
    {
        while (!isStopped)
        {
            double curDistanceScannerPositonAngle = GB08M2.getInstance().getDistanceScannerPositionAngle();

            if (curDistanceScannerPositonAngle == 130)
            {
                GB08M2.getInstance().distanceScannerDirection = false;
            }
            if (curDistanceScannerPositonAngle == 0)
            {
                GB08M2.getInstance().distanceScannerDirection = true;
            }

            if (GB08M2.getInstance().distanceScannerDirection)
            {
                curDistanceScannerPositonAngle += 0.5;
            } else
            {
                curDistanceScannerPositonAngle -= 0.5;
            }

            GB08M2.getInstance().setDistanceScannerPositionAngle(curDistanceScannerPositonAngle, true);

            try
            {
                Thread.sleep(GB08M2.distanceScannerPositioningDelay);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }

            GB08M2.getInstance().retrieveDistanceScannerDistance();
        }
        thread.interrupt();
    }
}
