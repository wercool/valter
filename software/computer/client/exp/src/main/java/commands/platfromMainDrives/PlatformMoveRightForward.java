package commands.platfromMainDrives;

import commands.CommandRunnable;

import javafx.scene.control.Button;
import valter.PLATFORM_CONTROL_P1;
import app.MainWindowController;

public class PlatformMoveRightForward implements Runnable, CommandRunnable
{
    MainWindowController mainWindowController;
    PLATFORM_CONTROL_P1 platform_control_p1;
    private volatile boolean isAccelerating = false;
    private volatile boolean isDecelerating = false;
    private volatile boolean isTerminated = false;
    private volatile boolean isStopped = false;
    private volatile int curDuty = 1;

    public PlatformMoveRightForward(PLATFORM_CONTROL_P1 platform_control_p1)
    {
        this.mainWindowController = platform_control_p1.mainWindowController;
        this.platform_control_p1 = platform_control_p1;
        System.out.println(this.getClass().getName() + "()");
    }

    @Override
    public void run()
    {
        while (!isStopped)
        {
            if (isTerminated)
            {
                curDuty = 1;
                continue;
            }

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
                this.platform_control_p1.cdcDevice.writeData("SETRIGHTMOTORPWMDUTY#" + curDuty);
                System.out.println(this.getClass().getName() + " is Accelerating [" + "Δ " + acceleration + " (" + curDuty + " → " + setDuty + ")]");
                double dutyVal = (double) curDuty / 100;
                this.mainWindowController.rightDutyProgressBar.setProgress(dutyVal);
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
                    if (!platform_control_p1.platformMoveLeftForwardRunnable.isEcecuting())
                    {
                        this.mainWindowController.setPlatformDriveButtonsState(true, (Button) null);
                    }
                }
                this.platform_control_p1.cdcDevice.writeData("SETRIGHTMOTORPWMDUTY#" + curDuty);
                System.out.println(this.getClass().getName() + " is Decelerating [" + "Δ " + deceleration + " (" + curDuty + " → 1)]");
                double dutyVal = (double) curDuty / 100;
                this.mainWindowController.rightDutyProgressBar.setProgress(dutyVal);
            }

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
        isTerminated = false;
        this.platform_control_p1.cdcDevice.writeData("RIGHTMOTORCCW");
        isAccelerating = true;
        isDecelerating = false;
        System.out.println(this.getClass().getName() + " execute()");
    }

    @Override
    public void cancel()
    {
        this.platform_control_p1.cdcDevice.writeData("RIGHTMOTORCCW");
        isDecelerating = true;
        isAccelerating = false;
        System.out.println(this.getClass().getName() + " cancel()");
    }

    @Override
    public void terminate()
    {
        isDecelerating = false;
        isAccelerating = false;
        isTerminated = true;
    }

    @Override
    public boolean isEcecuting()
    {
        if (isAccelerating || isDecelerating)
            return true;
        else
            return false;
    }

    @Override
    public void stop()
    {
        isStopped = true;
        System.out.println(this.getClass().getName() + " stop()");
    }
}