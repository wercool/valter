package gb082m2;

import javafx.application.Platform;
import application.MainAppController;

public class GB08M2ManualControlManager
{
    EncodersTask encodersTask;

    public void deinitialize()
    {
        stopEncodersReadings();
    }

    public GB08M2ManualControlManager()
    {
        new MotorsDutyVisualizationTask();
        new MotorsCurrentVisualizationTask();
        new EncodersVisualizationTask();
    }

    //Motors
    public void moveBothMotorsForward(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("forward");
            GB08M2.getInstance().setRightMotorsDirection("forward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void moveBothMotorsBackward(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("backward");
            GB08M2.getInstance().setRightMotorsDirection("backward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void turnLeft(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("backward");
            GB08M2.getInstance().setRightMotorsDirection("forward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void turnRight(int dutyLeft, int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("forward");
            GB08M2.getInstance().setRightMotorsDirection("backward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void moveLeftMotorsForward(int dutyLeft)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("forward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
        }
    }

    public void moveRightMotorsForward(int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setRightMotorsDirection("forward");
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void moveLeftMotorsBackward(int dutyLeft)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setLeftMotorsDirection("backward");
            new MotorsTasks.LeftMotorsAccelerationTask(dutyLeft).start();
        }
    }

    public void moveRightMotorsBackward(int dutyRight)
    {
        if (GB08M2.instance.isInitialized())
        {
            GB08M2.getInstance().setRightMotorsDirection("backward");
            new MotorsTasks.RightMotorsAccelerationTask(dutyRight).start();
        }
    }

    public void decelerateBothMotors()
    {
        if (GB08M2.instance.isInitialized())
        {
            new MotorsTasks.LeftMotorsDecelerationTask().start();
            new MotorsTasks.RightMotorsDecelerationTask().start();
        }
    }

    public void decelerateLeftMotors()
    {
        if (GB08M2.instance.isInitialized())
        {
            new MotorsTasks.LeftMotorsDecelerationTask().start();
        }
    }

    public void decelerateRightMotors()
    {
        if (GB08M2.instance.isInitialized())
        {
            new MotorsTasks.RightMotorsDecelerationTask().start();
        }
    }

    //Encoders
    public void startEncodersReadings()
    {
        if (GB08M2.instance.isInitialized())
        {
            if (encodersTask != null)
            {
                if (!encodersTask.isStopped)
                {
                    encodersTask.stop();
                }
            }
            encodersTask = new EncodersTask();
            encodersTask.start();
        }
    }

    public void stopEncodersReadings()
    {
        if (GB08M2.instance.isInitialized())
        {
            if (encodersTask != null)
            {
                encodersTask.stop();
            }
        }
    }

    //Visualization
    class MotorsDutyVisualizationTask implements Runnable
    {
        Thread thread;

        public MotorsDutyVisualizationTask()
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
                            MainAppController.leftMotorsDutyProgressBar.setProgress((double) GB08M2.getInstance().getLeftDuty() / 99);
                            MainAppController.rightMotorsDutyProgressBar.setProgress((double) GB08M2.getInstance().getRightDuty() / 99);
                        }
                    });
                    Thread.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    class MotorsCurrentVisualizationTask implements Runnable
    {
        Thread thread;

        public MotorsCurrentVisualizationTask()
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
                            MainAppController.frontLeftMotorCurrentProgressBar.setProgress((double) GB08M2.getInstance().getFrontLeftMotorCurrent() / GB08M2.maxADCMotorCurrentValue);
                            MainAppController.frontRightMotorCurrentProgressBar.setProgress((double) GB08M2.getInstance().getFrontRightMotorCurrent() / GB08M2.maxADCMotorCurrentValue);
                            MainAppController.rearLeftMotorCurrentProgressBar.setProgress((double) GB08M2.getInstance().getRearLeftMotorCurrent() / GB08M2.maxADCMotorCurrentValue);
                            MainAppController.rearRightMotorCurrentProgressBar.setProgress((double) GB08M2.getInstance().getRearRightMotorCurrent() / GB08M2.maxADCMotorCurrentValue);
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

    class EncodersVisualizationTask implements Runnable
    {
        Thread thread;
        boolean isStopped = false;

        public EncodersVisualizationTask()
        {
            new Thread(this).start();
        }

        public void stop()
        {
            isStopped = true;
        }

        @Override
        synchronized public void run()
        {
            while (MainAppController.isActive && !isStopped)
            {
                try
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            MainAppController.leftEncoderLabel.setText("Left: " + String.valueOf(GB08M2.getInstance().getLeftEncoderTicks()));
                            MainAppController.rightEncoderLabel.setText("Right: " + String.valueOf(GB08M2.getInstance().getRightEncoderTicks()));
                        }
                    });
                    Thread.sleep(GB08M2.encoderReadingsDelay);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    public static class DistanceMeterVisualizationTask implements Runnable
    {
        Thread thread;
        boolean singleMeasurement = false;
        boolean isStopped = false;

        public DistanceMeterVisualizationTask(boolean singleMeasurement)
        {
            this.singleMeasurement = singleMeasurement;
            new Thread(this).start();
        }

        @Override
        synchronized public void run()
        {
            while (MainAppController.isActive && !isStopped)
            {
                try
                {
                    if (this.singleMeasurement)
                    {
                        isStopped = true;
                        Thread.sleep(GB08M2.distanceScannerPositioningDelay);
                    }
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            MainAppController.distanceScannerValueLabel.setText("Distance: (" + String.valueOf(GB08M2.getInstance().getDistanceScannerDistance()) + ") = " + String.valueOf(GB08M2.getInstance().getDistanceScannerDistance_cm()) + " cm" + (GB08M2.getInstance().getDistanceScannerDistance_cm() > 150 ? " = blank" : ""));
                            MainAppController.distanceScannerPositionLabel.setText("Position: (" + String.valueOf(GB08M2.getInstance().getDistanceScannerPosition() + ") = " + GB08M2.getInstance().getDistanceScannerPositionAngle() + "°"));
                        }
                    });
                    Thread.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
