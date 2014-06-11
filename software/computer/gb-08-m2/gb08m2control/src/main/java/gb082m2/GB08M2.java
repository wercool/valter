package gb082m2;

public class GB08M2
{
    private static final GB08M2 instance = new GB08M2();
    private GB08M2CommandManager gb08m2CommandManager;
    boolean isInitialized = false;

    //Network locations
    static String hostname = "127.0.0.1";
    static int commandPort = 9001;
    static int frontCameraPort = 8080;
    static int rearCameraPort = 8081;

    boolean GB08M2CommandManagerConnected = false;

    //Hardware parameters
    volatile int batteryVoltage;

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
    public static GB08M2 getInstance()
    {
        return instance;
    }

    public boolean isInitialized()
    {
        return isInitialized;
    }

    public boolean isGB08M2CommandManagerConnected()
    {
        return GB08M2CommandManagerConnected;
    }

    public void setGB08M2CommandManagerConnected(boolean gB08M2CommandManagerConnected)
    {
        GB08M2CommandManagerConnected = gB08M2CommandManagerConnected;
    }
}
