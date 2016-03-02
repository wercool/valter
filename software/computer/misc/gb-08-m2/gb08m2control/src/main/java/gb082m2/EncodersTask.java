package gb082m2;


public class EncodersTask implements Runnable
{
    Thread thread;

    volatile boolean isStopped = false;

    public EncodersTask()
    {
        GB08M2.getInstance().enableEncoders();
        thread = new Thread(this);
    }

    public void start()
    {
        thread.start();
    }

    public void stop()
    {
        GB08M2.getInstance().disableEncoders();
        isStopped = true;
    }

    @Override
    public void run()
    {
        while (!isStopped)
        {
            GB08M2.getInstance().retrieveLeftEncoderTicks();
            GB08M2.getInstance().retrieveRightEncoderTicks();
            try
            {
                Thread.sleep(GB08M2.encoderReadingsDelay);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        GB08M2.getInstance().disableEncoders();
        thread.interrupt();
    }
}
