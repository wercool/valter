package gb082m2;

public class GB08M2ManualControlManager
{
    public GB08M2ManualControlManager()
    {
    }

    public void moveForward(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            new MoveForwardTask(dutyLeft, dutyRight).start();
        }
    }

    class MoveForwardTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        int targetDutyLeft;
        int targetDutyRight;

        volatile int curDutyLeft;
        volatile int curDutyRight;

        public MoveForwardTask(int dutyLeft, int dutyRight)
        {
            thread = new Thread(this);

            this.targetDutyLeft = dutyLeft;
            this.targetDutyRight = dutyRight;

            curDutyLeft = GB08M2.getInstance().getLeftDuty();
            curDutyRight = GB08M2.getInstance().getRightDuty();
        }

        public void start()
        {
            thread.start();
        }

        public void stop()
        {
            isStopped = true;
            thread.interrupt();
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                if (curDutyLeft < targetDutyLeft || curDutyRight < targetDutyRight)
                {
                    if (curDutyLeft < targetDutyLeft)
                    {
                        GB08M2.getInstance().setFrontLeftMotorDuty(curDutyLeft);
                        GB08M2.getInstance().setRearLeftMotorDuty(curDutyLeft);

                        GB08M2.getInstance().setLeftDuty(curDutyLeft);

                        curDutyLeft++;
                    }

                    if (curDutyRight < targetDutyRight)
                    {
                        curDutyRight++;
                    }

                    try
                    {
                        Thread.sleep(25);
                    } catch (InterruptedException e)
                    {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                } else
                {
                    this.stop();
                }
            }
        }
    }
}
