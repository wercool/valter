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

    //Hardware control
    volatile int batteryVoltage;

    public static final String frontLeftMotorDutyCommandPrefix = "FRONTLEFTDUTY#";
    volatile int frontLeftMotorDuty = 1;
    volatile int frontLeftMotorCurrent = 0;
    public static final String frontLeftMotorCurrentCommandPrefix = "GETLEFTFRONTCURRENT";
    public static final String frontLeftMotorCurrentResultPrefix = "FLMC:";

    public static final String frontRightMotorDutyCommandPrefix = "FRONTRIGHTDUTY#";
    volatile int frontRightMotorDuty = 1;
    volatile int frontRightMotorCurrent = 0;

    public static final String rearLeftMotorDutyCommandPrefix = "REARLEFTDUTY#";
    volatile int rearLeftMotorDuty = 1;
    volatile int rearLeftMotorCurrent = 0;

    public static final String rearRightMotorDutyCommandPrefix = "REARRIGHTDUTY#";
    volatile int rearRightMotorDuty = 1;
    volatile int rearRightMotorCurrent = 0;

    public static final String leftMotorsDirectionForwardCommand = "LEFTFORWARD";
    public static final String rightMotorsDirectionForwardCommand = "RIGHTFORWARD";
    public static final String leftMotorsDirectionBackwardCommand = "LEFTBACKWARD";
    public static final String rightMotorsDirectionBackwardCommand = "RIGHTBACKWARD";

    volatile String leftMotorsDirection = "forward";
    volatile String rightMotorsDirection = "forward";

    //Abstract parameters
    volatile int leftDuty = 1;
    volatile int rightDuty = 1;

    static final int accelerationStepDelay = 15;
    static final int decelerationStepDelay = 5;

    public GB08M2()
    {
        gb08m2ManualControlManager = new GB08M2ManualControlManager();
    }

    public boolean initialize()
    {
        isInitialized = true;

        gb08m2CommandManager = new GB08M2CommandManager();
        isInitialized &= gb08m2CommandManager.isConnected;

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

    public String getLeftMotorsDirection()
    {
        return leftMotorsDirection;
    }

    public void setLeftMotorsDirection(String leftMotorsDirection)
    {
        if (leftMotorsDirection.startsWith("forward"))
        {
            gb08m2CommandManager.sendCommand(leftMotorsDirectionForwardCommand);
        }
        if (leftMotorsDirection.startsWith("backward"))
        {
            gb08m2CommandManager.sendCommand(leftMotorsDirectionBackwardCommand);
        }
        this.leftMotorsDirection = leftMotorsDirection;
    }

    public String getRightMotorsDirection()
    {
        return rightMotorsDirection;
    }

    public void setRightMotorsDirection(String rightMotorsDirection)
    {
        if (rightMotorsDirection.startsWith("forward"))
        {
            gb08m2CommandManager.sendCommand(rightMotorsDirectionForwardCommand);
        }
        if (leftMotorsDirection.startsWith("backward"))
        {
            gb08m2CommandManager.sendCommand(rightMotorsDirectionBackwardCommand);
        }
        this.rightMotorsDirection = rightMotorsDirection;
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

    synchronized public int getFrontRightMotorDuty()
    {
        return frontRightMotorDuty;
    }

    public void setFrontRightMotorDuty(int frontRightMotorDuty)
    {
        if (this.frontRightMotorDuty != frontRightMotorDuty)
        {
            gb08m2CommandManager.sendCommand(frontRightMotorDutyCommandPrefix + String.valueOf(frontRightMotorDuty));
        }
        this.frontRightMotorDuty = frontRightMotorDuty;
    }

    synchronized public int getRearRightMotorDuty()
    {
        return rearRightMotorDuty;
    }

    public void setRearRightMotorDuty(int rearRightMotorDuty)
    {
        if (this.rearRightMotorDuty != rearRightMotorDuty)
        {
            gb08m2CommandManager.sendCommand(rearRightMotorDutyCommandPrefix + String.valueOf(rearRightMotorDuty));
        }
        this.rearRightMotorDuty = rearRightMotorDuty;
    }

    public synchronized void retrieveFrontLeftMotorCurrent()
    {
        gb08m2CommandManager.sendCommand(frontLeftMotorCurrentCommandPrefix);
    }

    public synchronized int getFrontLeftMotorCurrent()
    {
        return frontLeftMotorCurrent;
    }

    public synchronized void setFrontLeftMotorCurrent(int frontLeftMotorCurrent)
    {
        this.frontLeftMotorCurrent = frontLeftMotorCurrent;
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
