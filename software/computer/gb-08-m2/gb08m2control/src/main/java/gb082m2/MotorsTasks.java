package gb082m2;


public class MotorsTasks
{
    static boolean decelerationLeft = false;
    static boolean decelerationRight = false;

    static LeftMotorsCurrentReadingTask leftMotorsCurrentReadingTask;
    static RightMotorsCurrentReadingTask rightMotorsCurrentReadingTask;

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

            if (leftMotorsCurrentReadingTask != null)
            {
                leftMotorsCurrentReadingTask.stop();
            }
            leftMotorsCurrentReadingTask = new LeftMotorsCurrentReadingTask();
            leftMotorsCurrentReadingTask.start();
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
            while (!isStopped && !decelerationLeft)
            {
                if (curDutyLeft < targetDutyLeft)
                {
                    if (curDutyLeft < targetDutyLeft)
                    {
                        GB08M2.getInstance().setFrontLeftMotorDuty(curDutyLeft);
                        GB08M2.getInstance().setRearLeftMotorDuty(curDutyLeft);

                        GB08M2.getInstance().setLeftDuty(curDutyLeft);

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
            thread.interrupt();
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

            if (rightMotorsCurrentReadingTask != null)
            {
                rightMotorsCurrentReadingTask.stop();
            }
            rightMotorsCurrentReadingTask = new RightMotorsCurrentReadingTask();
            rightMotorsCurrentReadingTask.start();
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
            thread.interrupt();
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
            if (leftMotorsCurrentReadingTask != null)
            {
                leftMotorsCurrentReadingTask.stop();
            }
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
            thread.interrupt();
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
            if (rightMotorsCurrentReadingTask != null)
            {
                rightMotorsCurrentReadingTask.stop();
            }
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
            thread.interrupt();
        }
    }

    public static class LeftMotorsCurrentReadingTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        public LeftMotorsCurrentReadingTask()
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
                GB08M2.getInstance().retrieveFrontLeftMotorCurrent();
                GB08M2.getInstance().retrieveRearLeftMotorCurrent();
                try
                {
                    Thread.sleep(GB08M2.currentReadingsStepDelay);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
            try
            {
                Thread.sleep(GB08M2.currentReadingsStepDelay * 2);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            GB08M2.getInstance().setFrontLeftMotorCurrent(0);
            GB08M2.getInstance().setRearLeftMotorCurrent(0);
            thread.interrupt();
        }
    }

    public static class RightMotorsCurrentReadingTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        public RightMotorsCurrentReadingTask()
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
                GB08M2.getInstance().retrieveFrontRightMotorCurrent();
                GB08M2.getInstance().retrieveRearRightMotorCurrent();

                try
                {
                    Thread.sleep(GB08M2.currentReadingsStepDelay);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
            try
            {
                Thread.sleep(GB08M2.currentReadingsStepDelay * 2);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
            GB08M2.getInstance().setFrontRightMotorCurrent(0);
            GB08M2.getInstance().setRearRightMotorCurrent(0);
            thread.interrupt();
        }
    }
}
