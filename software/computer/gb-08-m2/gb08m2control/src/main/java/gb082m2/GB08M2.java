package gb082m2;

public class GB08M2
{
    static final GB08M2 instance = new GB08M2();
    boolean isInitialized = false;

    GB08M2CommandManager gb08m2CommandManager;
    public GB08M2ManualControlManager gb08m2ManualControlManager;

    //Network connections
    static String hostname = "127.0.0.1";
    static int commandPort = 9001;
    static int frontCameraPort = 8080;
    static int rearCameraPort = 8081;

    //Hardware parameters
    volatile int batteryVoltage;

    public static final String frontLeftMotorDutyCommandPrefix = "FRONTLEFTDUTY#";
    volatile int frontLeftMotorDuty = 1;
    volatile int frontLeftMotorDirection;
    volatile int frontLeftMotorCurrent;

    public static final String frontRightMotorDutyCommandPrefix = "FRONTRIGHTDUTY#";
    volatile int frontRightMotorDuty = 1;
    volatile int frontRightMotorDirection;
    volatile int frontRightMotorCurrent;

    public static final String rearLeftMotorDutyCommandPrefix = "REARLEFTDUTY#";
    volatile int rearLeftMotorDuty = 1;
    volatile int rearLeftMotorDirection;
    volatile int rearLeftMotorCurrent;

    public static final String rearRightMotorDutyCommandPrefix = "REARRIGHTDUTY#";
    volatile int rearRightMotorDuty = 1;
    volatile int rearRightMotorDirection;
    volatile int rearRightMotorCurrent;

    //Abstract parameters
    volatile int leftDuty = 1;
    volatile int rightDuty = 1;

    public boolean initialize()
    {
        isInitialized = true;

        gb08m2CommandManager = new GB08M2CommandManager();
        isInitialized &= gb08m2CommandManager.isConnected;

        gb08m2ManualControlManager = new GB08M2ManualControlManager();

        return isInitialized;
    }

    public boolean deInitialize()
    {
        gb08m2CommandManager.disconnect();
        return isInitialized = false;
    }

    //Getters and Setters
    //General
    public static GB08M2 getInstance()
    {
        return instance;
    }

    public boolean isInitialized()
    {
        return isInitialized;
    }
    
    public static String getHostname()
    {
        return hostname;
    }

    public static void setHostname(String hostname)
    {
        GB08M2.hostname = hostname;
    }

    public static int getCommandPort()
    {
        return commandPort;
    }

    public static void setCommandPort(int commandPort)
    {
        GB08M2.commandPort = commandPort;
    }

    public static int getFrontCameraPort()
    {
        return frontCameraPort;
    }

    public static void setFrontCameraPort(int frontCameraPort)
    {
        GB08M2.frontCameraPort = frontCameraPort;
    }

    public static int getRearCameraPort()
    {
        return rearCameraPort;
    }

    public static void setRearCameraPort(int rearCameraPort)
    {
        GB08M2.rearCameraPort = rearCameraPort;
    }

    //Hardware
    public synchronized int getFrontLeftMotorDuty()
    {
        return frontLeftMotorDuty;
    }

    public synchronized void setFrontLeftMotorDuty(int frontLeftMotorDuty)
    {
        if (this.frontLeftMotorDuty != frontLeftMotorDuty)
        {
            gb08m2CommandManager.sendCommand(frontLeftMotorDutyCommandPrefix + String.valueOf(frontLeftMotorDuty));
        }
        this.frontLeftMotorDuty = frontLeftMotorDuty;
    }

    public synchronized int getRearLeftMotorDuty()
    {
        return rearLeftMotorDuty;
    }

    public synchronized void setRearLeftMotorDuty(int rearLeftMotorDuty)
    {
        if (this.rearLeftMotorDuty != rearLeftMotorDuty)
        {
            gb08m2CommandManager.sendCommand(rearLeftMotorDutyCommandPrefix + String.valueOf(rearLeftMotorDuty));
        }
        this.rearLeftMotorDuty = rearLeftMotorDuty;
    }

    //Abstract
    public synchronized int getLeftDuty()
    {
        return leftDuty;
    }

    public synchronized void setLeftDuty(int leftDuty)
    {
        this.leftDuty = leftDuty;
    }

    public synchronized int getRightDuty()
    {
        return rightDuty;
    }

    public synchronized void setRightDuty(int rightDuty)
    {
        this.rightDuty = rightDuty;
    }
}
