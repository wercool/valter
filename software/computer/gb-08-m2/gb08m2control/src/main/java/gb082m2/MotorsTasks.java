package gb082m2;

public class MotorsTasks
{
    static boolean decelerationLeft = false;
    static boolean decelerationRight = false;

    public static class LeftMotorsAccelerationTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        int targetDutyLeft;

        volatile int curDutyLeft;

        public LeftMotorsAccelerationTask(int dutyLeft)
        {
            decelerationLeft = false;

            thread = new Thread(this);

            this.targetDutyLeft = dutyLeft;

            curDutyLeft = GB08M2.getInstance().getLeftDuty();
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
            while (!isStopped && !decelerationLeft)
            {
                if (curDutyLeft < targetDutyLeft)
                {
                    if (curDutyLeft < targetDutyLeft)
                    {
                        GB08M2.getInstance().setFrontLeftMotorDuty(curDutyLeft);
                        GB08M2.getInstance().setRearLeftMotorDuty(curDutyLeft);

                        GB08M2.getInstance().setLeftDuty(curDutyLeft);

                        GB08M2.getInstance().retrieveFrontLeftMotorCurrent();

                        curDutyLeft++;
                    }

                    try
                    {
                        Thread.sleep(GB08M2.accelerationStepDelay);
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

    public static class RightMotorsAccelerationTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        int targetDutyRight;

        volatile int curDutyRight;

        public RightMotorsAccelerationTask(int dutyRight)
        {
            decelerationRight = false;

            thread = new Thread(this);

            this.targetDutyRight = dutyRight;

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
            while (!isStopped && !decelerationRight)
            {
                if (curDutyRight < targetDutyRight)
                {
                    if (curDutyRight < targetDutyRight)
                    {
                        GB08M2.getInstance().setFrontRightMotorDuty(curDutyRight);
                        GB08M2.getInstance().setRearRightMotorDuty(curDutyRight);

                        GB08M2.getInstance().setRightDuty(curDutyRight);

                        curDutyRight++;
                    }

                    try
                    {
                        Thread.sleep(GB08M2.accelerationStepDelay);
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

    public static class LeftMotorsDecelerationTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        volatile int curDutyLeft;

        public LeftMotorsDecelerationTask()
        {
            decelerationLeft = true;

            thread = new Thread(this);

            curDutyLeft = GB08M2.getInstance().getLeftDuty();
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
                if (curDutyLeft > 0)
                {
                    if (curDutyLeft > 0)
                    {
                        GB08M2.getInstance().setFrontLeftMotorDuty(curDutyLeft);
                        GB08M2.getInstance().setRearLeftMotorDuty(curDutyLeft);

                        GB08M2.getInstance().setLeftDuty(curDutyLeft);

                        GB08M2.getInstance().setFrontLeftMotorCurrent(0);

                        curDutyLeft--;
                    }

                    try
                    {
                        Thread.sleep(GB08M2.decelerationStepDelay);
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

    public static class RightMotorsDecelerationTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        volatile int curDutyRight;

        public RightMotorsDecelerationTask()
        {
            decelerationRight = true;

            thread = new Thread(this);

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
                if (curDutyRight > 0)
                {
                    if (curDutyRight > 0)
                    {
                        GB08M2.getInstance().setFrontRightMotorDuty(curDutyRight);
                        GB08M2.getInstance().setRearRightMotorDuty(curDutyRight);

                        GB08M2.getInstance().setRightDuty(curDutyRight);

                        curDutyRight--;
                    }

                    try
                    {
                        Thread.sleep(GB08M2.decelerationStepDelay);
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
