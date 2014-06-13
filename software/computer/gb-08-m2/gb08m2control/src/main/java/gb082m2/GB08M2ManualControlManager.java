package gb082m2;

public class GB08M2ManualControlManager
{
    boolean decceleration = false;

    public GB08M2ManualControlManager()
    {}

    public void moveForward(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("forward");
            GB08M2.getInstance().setRightMotorsDirection("forward");
            new MovePairTask(dutyLeft, dutyRight).start();
        }
    }

    public void decelerate()
    {
        if (GB08M2.instance.isInitialized())
        {
            new DecelerationTask().start();
        }
    }

    class DecelerationTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        volatile int curDutyLeft;
        volatile int curDutyRight;

        public DecelerationTask()
        {
            decceleration = true;

            thread = new Thread(this);

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
                if (curDutyLeft > 0 || curDutyRight > 0)
                {
                    if (curDutyRight > 0)
                    {
                        GB08M2.getInstance().setFrontRightMotorDuty(curDutyRight);
                        GB08M2.getInstance().setRearRightMotorDuty(curDutyRight);

                        GB08M2.getInstance().setRightDuty(curDutyRight);

                        curDutyRight--;
                    }

                    if (curDutyLeft > 0)
                    {
                        GB08M2.getInstance().setFrontLeftMotorDuty(curDutyLeft);
                        GB08M2.getInstance().setRearLeftMotorDuty(curDutyLeft);

                        GB08M2.getInstance().setLeftDuty(curDutyLeft);

                        curDutyLeft--;
                    }

                    try
                    {
                        Thread.sleep(5);
                    } catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
                } else
                {
                    this.stop();
                }
            }
        }
    }

    class MovePairTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        int targetDutyLeft;
        int targetDutyRight;

        volatile int curDutyLeft;
        volatile int curDutyRight;

        public MovePairTask(int dutyLeft, int dutyRight)
        {
            decceleration = false;

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
            while (!isStopped && !decceleration)
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
                        GB08M2.getInstance().setFrontRightMotorDuty(curDutyRight);
                        GB08M2.getInstance().setRearRightMotorDuty(curDutyRight);

                        GB08M2.getInstance().setRightDuty(curDutyRight);

                        curDutyRight++;
                    }

                    try
                    {
                        Thread.sleep(25);
                    } catch (InterruptedException e)
                    {
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
