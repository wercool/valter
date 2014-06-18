package gb082m2;


public class DistanceScannerTask implements Runnable
{
    Thread thread;

    volatile boolean isStopped = false;

    public DistanceScannerTask()
    {
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

    @Override
    public void run()
    {
        while (!isStopped)
        {
            GB08M2.getInstance().setDistanceScannerPositionAngle(GB08M2.getInstance().getDistanceScannerPositionAngle() + 1);
            try
            {
                Thread.sleep(GB08M2.distanceScannerPositioningDelay);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        thread.interrupt();
    }
}
