package valter;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import app.MainWindowController;

public class PLATFORM_LOCATION_P1
{

    public MainWindowController mainWindowController = null;
    private static PLATFORM_LOCATION_P1 instance = new PLATFORM_LOCATION_P1();

    public CDCDevice cdcDevice;

    private boolean isInitialized = false;

    
    //Commands
    public static ObservableList<String> commands = FXCollections.observableArrayList("GETID",
            "ENABLESENSORS",
            "DISABLESENSORS",
            "!USSENSORSVOLTAGEUP#25",
            "!USSENSORSVOLTAGEDOWN#25",
            "USSENSORSDUTY#127",
            "USSENSORSBURST#250",
            "USSENSORSDELAY#2500",
            "STARTIRSENSORSREADINGS",
            "STOPIRSENSORSREADINGS",
            "STARTUSSENSORSREADINGS",
            "STOPUSSENSORSREADINGS",
            "SETSENSORSCHANNEL0",
            "SETSENSORSCHANNEL1",
            "SETSENSORSCHANNEL2",
            "SETSENSORSCHANNEL3",
            "SETSENSORSCHANNEL4",
            "SETSENSORSCHANNEL5",
            "SETSENSORSCHANNEL6",
            "SETSENSORSCHANNEL7",
            "SETSENSORSCHANNEL8",
            "SETSENSORSCHANNEL9",
            "SETSENSORSCHANNEL10",
            "SETSENSORSCHANNEL11",
            "SETSENSORSCHANNEL12",
            "SETLEFTUSSONARSERVODUTY#20",
            "DISABLELEFTUSSONARSERVO",
            "SETRIGHTUSSONARSERVODUTY#20",
            "DISABLERIGHTUSSONARSERVO",
            "STARTLEFTUSSONARREADINGS",
            "STOPLEFTUSSONARREADINGS",
            "STARTRIGHTUSSONARREADINGS",
            "STOPRIGHTUSSONARREADINGS",
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
            "READSTLM75#1",
            "READSTLM75#2",
            "STARTACCELEROMETERREADINGS",
            "STOPACCELEROMETERREADINGS",
            "STARTMAGNETOMETERREADINGS",
            "STOPMAGNETOMETERREADINGS",
            "SETLEDS#000000000000000000000000",
            "SETLEDS#111111111111111111111111",
            "SETLEDS#010101010101010101010101",
            "SETLEDS#101010101010101010101010",
            "SETSHREG#0000000000000000",
            "SETSHREG#1111000000000000",
            "SETSHREG#1010000000000000",
            "SETSHREG#0101000000000000",
            "SETSHREG#1100000000000000",
            "SETSHREG#0011000000000000",
            "SETSHREG#1111111111111111",
            "SETSHREG#0000000100000000"
            );

    //Command Threads

    public PLATFORM_LOCATION_P1()
    {
        System.out.println("PLATFORM_LOCATION_P1()");
    }

    public static PLATFORM_LOCATION_P1 getInstance()
    {
        return instance;
    }

    public void initialize()
    {

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

}
