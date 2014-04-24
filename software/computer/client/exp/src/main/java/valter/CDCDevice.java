package valter;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

import java.io.InputStream;
import java.io.OutputStream;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class CDCDevice implements SerialPortEventListener
{
    private static final Logger log = LoggerFactory.getLogger(CDCDevice.class);

    public int deviceIndex = 0;

    // the timeout value for connecting with the port
    final static int TIMEOUT = 2000;

    // this is the object that contains the opened port
    private CommPortIdentifier selectedPortIdentifier = null;
    private SerialPort serialPort = null;
    private String devicePortName = "";
    private Boolean deviceConnected = false;

    // input and output streams for sending and receiving data
    private final InputStream input = null;
    private final OutputStream output = null;

    public String getPortName()
    {
        return devicePortName;
    }

    public void setPortName(String portName)
    {
        this.devicePortName = portName;
    }

    public Boolean getDeviceConnected()
    {
        return deviceConnected;
    }

    public void setDeviceConnected(Boolean connected)
    {
        this.deviceConnected = connected;
    }

    public CDCDevice(CommPortIdentifier selectedPortIdentifier)
    {
        super();
        this.selectedPortIdentifier = selectedPortIdentifier;
        setPortName(selectedPortIdentifier.getName());
        setDeviceConnected(false);
    }

    // connect to the selected port
    public void connect()
    {
        CommPort commPort = null;

        try
        {
            // the method below returns an object of type CommPort
            commPort = selectedPortIdentifier.open(getClass().getName(), TIMEOUT);
            // the CommPort object can be casted to a SerialPort object
            serialPort = (SerialPort) commPort;
            serialPort.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
            setDeviceConnected(true);
        } catch (PortInUseException e)
        {
            log.error(this.selectedPortIdentifier + " is in use. (" + e.toString() + ")");
            setDeviceConnected(false);
        } catch (Exception e)
        {
            log.error("Failed to open " + this.selectedPortIdentifier + "(" + e.toString() + ")");
            setDeviceConnected(false);
        }
    }

    @Override
    public void serialEvent(SerialPortEvent arg0)
    {
    }

}