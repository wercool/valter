package valter;

import javafx.beans.property.SimpleStringProperty;

public final class ValterCDCDevice
{
    private final SimpleStringProperty systemDeviceName;
    private final SimpleStringProperty valterDeviceState;
    private final SimpleStringProperty valterDeviceName;

    public ValterCDCDevice(String _systemDeviceName,
                             String _valterDeviceState,
                             String _valterDeviceName)
    {
        this.systemDeviceName   = new SimpleStringProperty(_systemDeviceName);
        this.valterDeviceState  = new SimpleStringProperty(_valterDeviceState);
        this.valterDeviceName   = new SimpleStringProperty(_valterDeviceName);
    }

    public String getSystemDeviceName()
    {
        return systemDeviceName.get();
    }

    public void setSystemDeviceName(String systemDeviceName)
    {
        this.systemDeviceName.set(systemDeviceName);
    }

    public String getValterDeviceState()
    {
        return valterDeviceState.get();
    }

    public void setValterDeviceState(String valterDeviceState)
    {
        this.valterDeviceState.set(valterDeviceState);
    }

    public String getValterDeviceName()
    {
        return valterDeviceName.get();
    }

    public void setValterDeviceName(String valterDeviceName)
    {
        this.valterDeviceName.set(valterDeviceName);
    }
}
