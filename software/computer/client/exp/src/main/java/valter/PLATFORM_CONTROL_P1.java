package valter;

import java.util.ArrayList;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.scene.control.Button;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveBackward;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveForward;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveLeftBackward;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveLeftForward;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveRightBackward;
import platform_control_p1.commands.platfromMainDrives.PlatformMoveRightForward;
import platform_control_p1.commands.platfromMainDrives.PlatformRotateCCW;
import platform_control_p1.commands.platfromMainDrives.PlatformRotateCW;
import platform_control_p1.commands.turretDrive.GetTurretPosition;
import platform_control_p1.commands.turretDrive.TurretRotateCCW;
import platform_control_p1.commands.turretDrive.TurretRotateCW;
import platform_control_p1.listeners.ChargerListener;
import app.MainWindowController;

public class PLATFORM_CONTROL_P1
{

    public MainWindowController mainWindowController = null;
    private static PLATFORM_CONTROL_P1 instance = new PLATFORM_CONTROL_P1();

    public CDCDevice cdcDevice;

    private boolean isInitialized = false;

    //Commands
    public static ObservableList<String> commands = FXCollections.observableArrayList(
            "GETID",
            "MAINACCUMULATORRELAYON",
            "MAINACCUMULATORRELAYOFF",
            "CHARGERBUTTONPRESS",
            "DCDC5VENABLEON",
            "DCDC5VENABLEOFF",
            "LEFTACCUMULATORRELAYON",
            "LEFTACCUMULATORRELAYOFF",
            "RIGHTACCUMULATORRELAYON",
            "RIGHTACCUMULATORRELAYOFF",
            "STARTINPUT1READINGS",
            "STOPINPUT1READINGS",
            "SETINPUT1CHANNEL0",
            "SETINPUT1CHANNEL1",
            "SETINPUT1CHANNEL2",
            "SETINPUT1CHANNEL3",
            "SETINPUT1CHANNEL4",
            "SETINPUT1CHANNEL5",
            "SETINPUT1CHANNEL6",
            "SETINPUT1CHANNEL7",
            "SETINPUT1CHANNEL8",
            "SETINPUT1CHANNEL9",
            "SETINPUT1CHANNEL10",
            "SETINPUT1CHANNEL11",
            "SETINPUT1CHANNEL12",
            "SETINPUT1CHANNEL13",
            "SETINPUT1CHANNEL14",
            "SETINPUT1CHANNEL15",
            "SETINPUT2CHANNEL0",
            "STARTINPUT2READINGS",
            "STOPINPUT2READINGS",
            "SETINPUT2CHANNEL0",
            "SETINPUT2CHANNEL1",
            "SETINPUT2CHANNEL2",
            "SETINPUT2CHANNEL3",
            "SETINPUT2CHANNEL4",
            "SETINPUT2CHANNEL5",
            "SETINPUT2CHANNEL6",
            "SETINPUT2CHANNEL7",
            "SETINPUT2CHANNEL8",
            "SETINPUT2CHANNEL9",
            "SETINPUT2CHANNEL10",
            "SETINPUT2CHANNEL11",
            "SETINPUT2CHANNEL12",
            "SETINPUT2CHANNEL13",
            "SETINPUT2CHANNEL14",
            "SETINPUT2CHANNEL15",
            "STARTTURRETREADINGS",
            "STOPTURRETREADINGS",
            "SETPWMFREQUENCY#8000",
            "SETLEFTMOTORPWMDUTY#40",
            "SETRIGHTMOTORPWMDUTY#40",
            "SETTURRETMOTORPWMDUTY#30",
            "!SETTURRETPOSITION#525",
            "LEFTMOTORCW",
            "LEFTMOTORCCW",
            "LEFTMOTORSTOP",
            "RIGHTMOTORCW",
            "RIGHTMOTORCCW",
            "RIGHTMOTORSTOP",
            "TURRETMOTORCW",
            "TURRETMOTORCCW",
            "TURRETMOTORSTOP",
            "STARTLEFTMOTORCURRENTREADINGS",
            "STOPLEFTMOTORCURRENTREADINGS",
            "STARTRIGHTMOTORCURRENTREADINGS",
            "STOPRIGHTMOTORCURRENTREADINGS",
            "STARTTURRETMOTORCURRENTREADINGS",
            "STOPTURRETMOTORCURRENTREADINGS",
            "GETLEFTMOTORCOUNTER",
            "RESETLEFTMOTORCOUNTER",
            "GETRIGHTMOTORCOUNTER",
            "RESETRIGHTMOTORCOUNTER",
            "LEFTMOTORCOUNTERSTART",
            "LEFTMOTORCOUNTERSTOP",
            "RIGHTMOTORCOUNTERSTART",
            "RIGHTMOTORCOUNTERSTOP",
            "LEFTACCUMULATORCONNECT",
            "LEFTACCUMULATORDISCONNECT",
            "RIGHTACCUMULATORCONNECT",
            "RIGHTACCUMULATORDISCONNECT"
            );

    //Command Threads

    //Platform Main Drives Control
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

    //Turret Control
    public TurretRotateCW turretRotateCWRunnable = null;
    Thread turretRotateCWThread = null;

    public TurretRotateCCW turretRotateCCWRunnable = null;
    Thread turretRotateCCWThread = null;

    public GetTurretPosition getTurretPositionRunnable = null;
    Thread getTurretPositionThread = null;

    //Power Status
    public ChargerListener getChargerConnectedRunnable = null;
    Thread getChargerConnectedThread = null;

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
        //Platform Main Drives Control
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

        //Turret Control
        turretRotateCWRunnable = new TurretRotateCW(this);
        turretRotateCWThread = new Thread(turretRotateCWRunnable);
        turretRotateCWThread.start();

        turretRotateCCWRunnable = new TurretRotateCCW(this);
        turretRotateCCWThread = new Thread(turretRotateCCWRunnable);
        turretRotateCCWThread.start();

        getTurretPositionRunnable = new GetTurretPosition(this);
        getTurretPositionThread = new Thread(getTurretPositionRunnable);
        getTurretPositionThread.start();

        //Power Status
        getChargerConnectedRunnable = new ChargerListener(this);
        getTurretPositionThread = new Thread(getChargerConnectedRunnable);
        getTurretPositionThread.start();


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
        stopExecutionOfAllCommads();
        initialize();
    }

    public boolean isReady()
    {
        if (this.cdcDevice != null && this.cdcDevice.getDeviceConnected())
        {
            return true;
        } else
        {
            return false;
        }
    }

    public void stopExecutionOfAllCommads()
    {
        if (isInitialized)
        {
            //Platform Main Drives Control
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

            //Turret Control
            turretRotateCWRunnable.stop();
            turretRotateCWThread.interrupt();

            turretRotateCCWRunnable.stop();
            turretRotateCCWThread.interrupt();

            getTurretPositionRunnable.stop();
            getTurretPositionThread.interrupt();

            //Power Status
            getChargerConnectedRunnable.stop();
            getTurretPositionThread.interrupt();

            isInitialized = false;
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
                    //Platform Main Drives Control
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
                    case "STOP_PLATFROM":
                        platformMoveForwardRunnable.terminate();
                        platformMoveBackwardRunnable.terminate();
                        platformRotateCCWRunnable.terminate();
                        platformRotateCWRunnable.terminate();
                        platformMoveLeftForwardRunnable.terminate();
                        platformMoveLeftBackwardRunnable.terminate();
                        platformMoveRightForwardRunnable.terminate();
                        platformMoveRightBackwardRunnable.terminate();

                        Thread terminatingPlatfromDrivesThread = new Thread(new TerminatingPlatfromDrivesRunnable(this.mainWindowController, this));
                        terminatingPlatfromDrivesThread.start();
                        break;

                        //Turret Control
                    case "TURRET_CW_EXECUTE":
                        turretRotateCWRunnable.execute();
                        getTurretPositionRunnable.execute();
                        break;
                    case "TURRET_CW_CANCEL":
                        turretRotateCWRunnable.cancel();
                        break;
                    case "TURRET_CCW_EXECUTE":
                        turretRotateCCWRunnable.execute();
                        getTurretPositionRunnable.execute();
                        break;
                    case "TURRET_CCW_CANCEL":
                        turretRotateCCWRunnable.cancel();
                        break;
                    case "STOP_TURRET":
                        turretRotateCWRunnable.terminate();
                        turretRotateCCWRunnable.terminate();
                        Thread terminatingTurretControlThread = new Thread(new TerminatingTurretControlRunnable(this.mainWindowController, this));
                        terminatingTurretControlThread.start();
                        break;
                    case "GET_TURRET_POSITION_EXECUTE":
                        getTurretPositionRunnable.execute();
                        break;
                    case "GET_TURRET_POSITION_CANCEL":
                        getTurretPositionRunnable.cancel();
                        break;

                        //Switches
                    case "5V_ENABLE":
                        this.cdcDevice.writeData("DCDC5VENABLEON");
                        break;
                    case "5V_DISABLE":
                        this.cdcDevice.writeData("DCDC5VENABLEOFF");
                        break;
                    case "MAINACCUMULATORRELAYON":
                        this.cdcDevice.writeData("MAINACCUMULATORRELAYON");
                        break;
                    case "MAINACCUMULATORRELAYOFF":
                        this.cdcDevice.writeData("MAINACCUMULATORRELAYOFF");
                        break;
                    case "LEFTACCUMULATORRELAYON":
                        this.cdcDevice.writeData("LEFTACCUMULATORRELAYON");
                        break;
                    case "LEFTACCUMULATORRELAYOFF":
                        this.cdcDevice.writeData("LEFTACCUMULATORRELAYOFF");
                        break;
                    case "RIGHTACCUMULATORRELAYON":
                        this.cdcDevice.writeData("RIGHTACCUMULATORRELAYON");
                        break;
                    case "RIGHTACCUMULATORRELAYOFF":
                        this.cdcDevice.writeData("RIGHTACCUMULATORRELAYOFF");
                        break;
                    case "CHARGERBUTTONPRESS":
                        this.cdcDevice.writeData("CHARGERBUTTONPRESS");
                        break;

                        //Power Status
                    case "GET_CHARGER_CONNECTED_START":
                        getChargerConnectedRunnable.execute();
                        break;
                    case "GET_CHARGER_CONNECTED_STOP":
                        getChargerConnectedRunnable.cancel();
                        break;
                }
            } else
            {
                mainWindowController.cdcIsNotConnectedActions();
            }
        } else
        {
            mainWindowController.cdcIsNotConnectedActions();
        }
    }

    class TerminatingPlatfromDrivesRunnable implements Runnable
    {
        private final MainWindowController mainWindowController;
        private final PLATFORM_CONTROL_P1 platform_control_p1;

        public TerminatingPlatfromDrivesRunnable(MainWindowController mainWindowController, PLATFORM_CONTROL_P1 platform_control_p1)
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
                    System.out.println(this.getClass().getName() + ": l:" + leftDuty + ", r:" + rightDuty);
                    if (leftDuty - 1 > 0)
                    {
                        leftDuty--;
                    }
                    if (rightDuty - 1 > 0)
                    {
                        rightDuty--;
                    }

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

    class TerminatingTurretControlRunnable implements Runnable
    {
        private final MainWindowController mainWindowController;
        private final PLATFORM_CONTROL_P1 platform_control_p1;

        public TerminatingTurretControlRunnable(MainWindowController mainWindowController, PLATFORM_CONTROL_P1 platform_control_p1)
        {
            this.mainWindowController = mainWindowController;
            this.platform_control_p1 = platform_control_p1;
        }

        @Override
        public void run()
        {

            int turretDuty = (int) Math.round(this.mainWindowController.turretDutyProgressBar.getProgress() * 100);
            double dutyVal;
            try
            {
                while (turretDuty > 1)
                {
                    System.out.println(this.getClass().getName() + ": " + turretDuty);
                    if (turretDuty - 1 > 0)
                    {
                        turretDuty--;
                    }

                    platform_control_p1.cdcDevice.writeData("SETTURRETMOTORPWMDUTY#" + turretDuty);

                    dutyVal = (double) turretDuty / 100;
                    this.mainWindowController.turretDutyProgressBar.setProgress(dutyVal);

                    Thread.sleep(1);
                }
                platform_control_p1.cdcDevice.writeData("TURRETMOTORSTOP");

                this.mainWindowController.setTurretControlButtonsState(true, (ArrayList<Button>) null);

                this.platform_control_p1.getTurretPositionRunnable.cancel();

            } catch (Exception e)
            {
                e.printStackTrace();
            }
        }
    }

}
