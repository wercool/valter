package valter;

import app.MainWindowController;

interface CommandRunnable
{
    void execute();

    void cancel();

    void stop();
}

public class PLATFORM_CONTROL_P1
{

    private MainWindowController mainWindowController = null;
    private static PLATFORM_CONTROL_P1 instance = new PLATFORM_CONTROL_P1();

    CDCDevice cdcDevice;

    Thread moveForwardThread = null;
    MoveForward moveForwardRunnable = null;

    public PLATFORM_CONTROL_P1()
    {
        System.out.println("PLATFORM_CONTROL_P1()");
    }

    public static PLATFORM_CONTROL_P1 getInstance()
    {
        return instance;
    }

    public void initialize()
    {
        moveForwardRunnable = new MoveForward(this);
        moveForwardThread = new Thread(moveForwardRunnable);
        moveForwardThread.start();
    }

    public void setMainController(MainWindowController mainWindowController)
    {
        this.mainWindowController = mainWindowController;
    }

    public CDCDevice getCdcDevice()
    {
        return cdcDevice;
    }

    public void setCdcDevice(CDCDevice cdcDevice)
    {
        this.cdcDevice = cdcDevice;
    }

    public void stopExecutionOfAllCommads()
    {
        moveForwardRunnable.stop();
        moveForwardThread.interrupt();
    }

    public void executeCommand(String cmd)
    {
        if (cdcDevice != null)
        {
            if (cdcDevice.getDeviceConnected())
            {
                switch (cmd)
                {
                case "FORWARD EXECUTE":
                    moveForwardRunnable.execute();
                    break;
                case "FORWARD CANCEL":
                    moveForwardRunnable.cancel();
                    break;
                case "STOP ALL":
                    moveForwardRunnable.cancel();
                    break;
                }
            } else
            {
                MainWindowController.showTooltip(mainWindowController.mainAppObject.stage, mainWindowController.forwardBtn, "CDC Device disconnected", null);
                mainWindowController.logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device disconnected");
            }
        } else
        {
            MainWindowController.showTooltip(mainWindowController.mainAppObject.stage, mainWindowController.forwardBtn, "CDC Device is not assigned", null);
            mainWindowController.logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device is not assigned");
        }
    }

    //Execution threads
    private class MoveForward implements Runnable, CommandRunnable
    {
        MainWindowController mainWindowController;
        PLATFORM_CONTROL_P1 platform_control_p1;
        private volatile boolean isAccelerating = false;
        private volatile boolean isDecelerating = false;
        private volatile boolean isStopped = false;
        int curDuty = 1;

        public MoveForward(PLATFORM_CONTROL_P1 platform_control_p1)
        {
            this.mainWindowController = platform_control_p1.mainWindowController;
            this.platform_control_p1 = platform_control_p1;
            System.out.println("MoveForward()");
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                int setDuty = (int) mainWindowController.platformMotorsDuty.getValue();
                int acceleration = (int) mainWindowController.platformMotorsAсceleration.getValue();
                int deceleration = (int) mainWindowController.platformMotorsDeceleration.getValue();
                if (isAccelerating)
                {
                    if (curDuty + acceleration < setDuty)
                    {
                        curDuty += acceleration;
                    } else
                    {
                        curDuty = setDuty;
                        isAccelerating = false;
                    }
                    System.out.println("MoveForward is Accelerating [" + "Δ " + acceleration + " (" + curDuty + " → " + setDuty + ")]");
                }
                if (isDecelerating)
                {
                    if (curDuty - deceleration > 1)
                    {
                        curDuty -= deceleration;
                    } else
                    {
                        curDuty = 1;
                        isDecelerating = false;
                    }
                    System.out.println("MoveForward is Decelerating [" + "Δ " + deceleration + " (" + curDuty + " → 1)]");
                }
                this.platform_control_p1.cdcDevice.writeData("SETLEFTMOTORPWMDUTY#" + curDuty);
                this.platform_control_p1.cdcDevice.writeData("SETRIGHTMOTORPWMDUTY#" + curDuty);

                try
                {
                    Thread.sleep(100);
                } catch (InterruptedException e)
                {
                    //e.printStackTrace();
                }
            }
        }

        @Override
        public void execute()
        {
            this.platform_control_p1.cdcDevice.writeData("LEFTMOTORCW");
            this.platform_control_p1.cdcDevice.writeData("RIGHTMOTORCCW");
            isAccelerating = true;
            isDecelerating = false;
            System.out.println("MoveForward execute()");
        }

        @Override
        public void cancel()
        {
            this.platform_control_p1.cdcDevice.writeData("LEFTMOTORCW");
            this.platform_control_p1.cdcDevice.writeData("RIGHTMOTORCCW");
            isDecelerating = true;
            isAccelerating = false;
            System.out.println("MoveForward cancel()");
        }

        @Override
        public void stop()
        {
            isStopped = true;
            System.out.println("MoveForward stop()");
        }
    }
}
