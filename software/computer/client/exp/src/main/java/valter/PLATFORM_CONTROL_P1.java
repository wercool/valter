package valter;

import javafx.scene.control.Button;
import app.MainWindowController;

import commands.PlatformMoveBackward;
import commands.PlatformMoveForward;
import commands.PlatformMoveLeftBackward;
import commands.PlatformMoveLeftForward;
import commands.PlatformMoveRightBackward;
import commands.PlatformMoveRightForward;
import commands.PlatformRotateCCW;
import commands.PlatformRotateCW;

public class PLATFORM_CONTROL_P1
{

    public MainWindowController mainWindowController = null;
    private static PLATFORM_CONTROL_P1 instance = new PLATFORM_CONTROL_P1();

    public CDCDevice cdcDevice;

    private boolean isInitialized = false;

    //Command Threads
    public PlatformMoveForward platformMoveForwardRunnable = null;
    Thread platformMoveForwardThread = null;

    public PlatformMoveBackward platformMoveBackwardRunnable = null;
    Thread platformMoveBackwardThread = null;

    public PlatformRotateCCW platformRotateCCWRunnable = null;
    Thread platformRotateCCWThread = null;

    public PlatformRotateCW platformRotateCWRunnable = null;
    Thread platformRotateCWThread = null;

    public PlatformMoveLeftForward platformMoveLeftForwardRunnable = null;
    Thread platformMoveLeftForwardThread = null;

    public PlatformMoveLeftBackward platformMoveLeftBackwardRunnable = null;
    Thread platformMoveLeftBackwardThread = null;

    public PlatformMoveRightForward platformMoveRightForwardRunnable = null;
    Thread platformMoveRightForwardThread = null;

    public PlatformMoveRightBackward platformMoveRightBackwardRunnable = null;
    Thread platformMoveRightBackwardThread = null;

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
        platformMoveForwardRunnable = new PlatformMoveForward(this);
        platformMoveForwardThread = new Thread(platformMoveForwardRunnable);
        platformMoveForwardThread.start();

        platformMoveBackwardRunnable = new PlatformMoveBackward(this);
        platformMoveBackwardThread = new Thread(platformMoveBackwardRunnable);
        platformMoveBackwardThread.start();

        platformRotateCCWRunnable = new PlatformRotateCCW(this);
        platformRotateCCWThread = new Thread(platformRotateCCWRunnable);
        platformRotateCCWThread.start();

        platformRotateCWRunnable = new PlatformRotateCW(this);
        platformRotateCWThread = new Thread(platformRotateCWRunnable);
        platformRotateCWThread.start();

        platformMoveLeftForwardRunnable = new PlatformMoveLeftForward(this);
        platformMoveLeftForwardThread = new Thread(platformMoveLeftForwardRunnable);
        platformMoveLeftForwardThread.start();

        platformMoveLeftBackwardRunnable = new PlatformMoveLeftBackward(this);
        platformMoveLeftBackwardThread = new Thread(platformMoveLeftBackwardRunnable);
        platformMoveLeftBackwardThread.start();

        platformMoveRightForwardRunnable = new PlatformMoveRightForward(this);
        platformMoveRightForwardThread = new Thread(platformMoveRightForwardRunnable);
        platformMoveRightForwardThread.start();

        platformMoveRightBackwardRunnable = new PlatformMoveRightBackward(this);
        platformMoveRightBackwardThread = new Thread(platformMoveRightBackwardRunnable);
        platformMoveRightBackwardThread.start();

        isInitialized = true;
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
        if (isInitialized)
        {
            platformMoveForwardRunnable.stop();
            platformMoveForwardThread.interrupt();

            platformMoveBackwardRunnable.stop();
            platformMoveBackwardThread.interrupt();

            platformRotateCCWRunnable.stop();
            platformRotateCCWThread.interrupt();

            platformRotateCWRunnable.stop();
            platformRotateCWThread.interrupt();

            platformMoveLeftForwardRunnable.stop();
            platformMoveLeftForwardThread.interrupt();

            platformMoveLeftBackwardRunnable.stop();
            platformMoveLeftBackwardThread.interrupt();

            platformMoveRightForwardRunnable.stop();
            platformMoveRightForwardThread.interrupt();

            platformMoveRightBackwardRunnable.stop();
            platformMoveRightBackwardThread.interrupt();
        }
    }

    public void executeCommand(String cmd)
    {
        if (cdcDevice != null)
        {
            if (cdcDevice.getDeviceConnected())
            {
                switch (cmd)
                {
                case "PLATFORM_FORWARD_EXECUTE":
                    platformMoveForwardRunnable.execute();
                    break;
                case "PLATFORM_FORWARD_CANCEL":
                    platformMoveForwardRunnable.cancel();
                    break;
                case "PLATFORM_BACKWARD_EXECUTE":
                    platformMoveBackwardRunnable.execute();
                    break;
                case "PLATFORM_BACKWARD_CANCEL":
                    platformMoveBackwardRunnable.cancel();
                    break;
                case "PLATFORM_CCW_EXECUTE":
                    platformRotateCCWRunnable.execute();
                    break;
                case "PLATFORM_CCW_CANCEL":
                    platformRotateCCWRunnable.cancel();
                    break;
                case "PLATFORM_CW_EXECUTE":
                    platformRotateCWRunnable.execute();
                    break;
                case "PLATFORM_CW_CANCEL":
                    platformRotateCWRunnable.cancel();
                    break;
                case "PLATFORM_LEFT_FORWARD_EXECUTE":
                    platformMoveLeftForwardRunnable.execute();
                    break;
                case "PLATFORM_LEFT_FORWARD_CANCEL":
                    platformMoveLeftForwardRunnable.cancel();
                    break;
                case "PLATFORM_LEFT_BACKWARD_EXECUTE":
                    platformMoveLeftBackwardRunnable.execute();
                    break;
                case "PLATFORM_LEFT_BACKWARD_CANCEL":
                    platformMoveLeftBackwardRunnable.cancel();
                    break;
                case "PLATFORM_RIGHT_FORWARD_EXECUTE":
                    platformMoveRightForwardRunnable.execute();
                    break;
                case "PLATFORM_RIGHT_FORWARD_CANCEL":
                    platformMoveRightForwardRunnable.cancel();
                    break;
                case "PLATFORM_RIGHT_BACKWARD_EXECUTE":
                    platformMoveRightBackwardRunnable.execute();
                    break;
                case "PLATFORM_RIGHT_BACKWARD_CANCEL":
                    platformMoveRightBackwardRunnable.cancel();
                    break;

                case "STOP_ALL":
                    platformMoveForwardRunnable.terminate();
                    platformMoveBackwardRunnable.terminate();
                    platformRotateCCWRunnable.terminate();
                    platformRotateCWRunnable.terminate();
                    platformMoveLeftForwardRunnable.terminate();
                    platformMoveLeftBackwardRunnable.terminate();
                    platformMoveRightForwardRunnable.terminate();
                    platformMoveRightBackwardRunnable.terminate();

                    class TerminatingRunnable implements Runnable
                    {
                        private final MainWindowController mainWindowController;
                        private final PLATFORM_CONTROL_P1 platform_control_p1;

                        public TerminatingRunnable(MainWindowController mainWindowController, PLATFORM_CONTROL_P1 platform_control_p1)
                        {
                            this.mainWindowController = mainWindowController;
                            this.platform_control_p1 = platform_control_p1;
                        }

                        @Override
                        public void run()
                        {

                            int leftDuty = (int) Math.round(this.mainWindowController.leftDutyProgressBar.getProgress() * 100);
                            int rightDuty = (int) Math.round(this.mainWindowController.rightDutyProgressBar.getProgress() * 100);
                            double dutyVal;

                            try
                            {
                                while (leftDuty > 1 || rightDuty > 1)
                                {
                                    System.out.println("TerminatingRunnable: l:" + leftDuty + ", r:" + rightDuty);
                                    if (leftDuty - 1 > 0)
                                        leftDuty--;
                                    if (rightDuty - 1 > 0)
                                        rightDuty--;

                                    platform_control_p1.cdcDevice.writeData("SETLEFTMOTORPWMDUTY#" + leftDuty);
                                    platform_control_p1.cdcDevice.writeData("SETRIGHTMOTORPWMDUTY#" + rightDuty);

                                    dutyVal = (double) leftDuty / 100;
                                    this.mainWindowController.leftDutyProgressBar.setProgress(dutyVal);
                                    dutyVal = (double) rightDuty / 100;
                                    this.mainWindowController.rightDutyProgressBar.setProgress(dutyVal);

                                    Thread.sleep(1);
                                }
                                platform_control_p1.cdcDevice.writeData("LEFTMOTORSTOP");
                                platform_control_p1.cdcDevice.writeData("RIGHTMOTORSTOP");

                                this.mainWindowController.setPlatformDriveButtonsState(true, (Button) null);
                            } catch (Exception e)
                            {
                                e.printStackTrace();
                            }
                        }
                    }
                    Thread t = new Thread(new TerminatingRunnable(this.mainWindowController, this));
                    t.start();
                    break;
                }
            } else
            {
                MainWindowController.showTooltip(mainWindowController.mainAppObject.stage, mainWindowController.platformDrivesControlPane, "CDC Device disconnected", null);
                mainWindowController.logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device disconnected");
                mainWindowController.setPlatformDriveButtonsState(true, (Button) null);
            }
        } else
        {
            MainWindowController.showTooltip(mainWindowController.mainAppObject.stage, mainWindowController.platformDrivesControlPane, "CDC Device is not assigned", null);
            mainWindowController.logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device is not assigned");
            mainWindowController.setPlatformDriveButtonsState(true, (Button) null);
        }
    }
}
