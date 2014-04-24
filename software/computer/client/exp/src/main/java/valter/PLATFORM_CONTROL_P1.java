package valter;

import app.MainWindowController;

public class PLATFORM_CONTROL_P1
{

    private MainWindowController mainWindowController = null;
    private static PLATFORM_CONTROL_P1 instance = new PLATFORM_CONTROL_P1();

    CDCDevice cdcDevice;

    public static PLATFORM_CONTROL_P1 getInstance()
    {
        return instance;
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

    public void executeCommand(String cmd)
    {
        if (cdcDevice != null)
        {
            switch (cmd)
            {
            case "MOVE FORWARD":
                cdcDevice.writeData("STARTINPUT1READINGS");
                break;

            default:
                break;
            }
        } else
        {
            MainWindowController.showTooltip(mainWindowController.mainAppObject.primaryStage, mainWindowController.forwardBtn, "CDC Device is not assigned", null);
            mainWindowController.logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device is not assigned");
        }
    }
}
