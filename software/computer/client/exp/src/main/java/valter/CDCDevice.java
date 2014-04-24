package valter;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;

import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import app.MainWindowController;

public class CDCDevice
{
    private static final Logger log = LoggerFactory.getLogger(CDCDevice.class);

    // the timeout value for connecting with the port
    final static int TIMEOUT = 2000;

    final static int NEW_LINE_ASCII = 10;

    MainWindowController mainWindowController = null;

    // this is the object that contains the opened port
    private CommPortIdentifier selectedPortIdentifier = null;
    private SerialPort serialPort = null;
    private String portName = null;

    private final BooleanProperty deviceConnected = new SimpleBooleanProperty();
    private final StringProperty deviceName = new SimpleStringProperty();

    BufferedReader inputDataBufferedReader = null;
    public String dataString = null;
    public Long dataStringId = (long) 0;

    // input and output streams for sending and receiving data
    private InputStream input = null;
    private OutputStream output = null;

    Thread serialPortReadThread = null;
    public SerialPortReader serialPortReader;

    public String getPortName()
    {
        return portName;
    }

    public void setPortName(String portName)
    {
        this.portName = portName;
    }

    public String getDeviceName()
    {
        return deviceName.get();
    }

    public SimpleStringProperty deviceNameProperty()
    {
        return (SimpleStringProperty) deviceName;
    }

    public void setDeviceName(String deviceName)
    {
        this.deviceName.set(deviceName);
    }

    public Boolean getDeviceConnected()
    {
        return deviceConnected.get();
    }

    public SimpleBooleanProperty deviceConnectedProperty()
    {
        return (SimpleBooleanProperty) deviceConnected;
    }

    private void setDeviceConnected(Boolean connected)
    {
        deviceConnected.set(connected);
    }

    private void logToConsole(String msg)
    {
        mainWindowController.logToConsole(msg);
    }

    public CDCDevice(CommPortIdentifier selectedPortIdentifier)
    {
        super();
        this.selectedPortIdentifier = selectedPortIdentifier;
        setPortName(selectedPortIdentifier.getName());
        setDeviceConnected(false);
    }

    public CDCDevice(CommPortIdentifier selectedPortIdentifier, MainWindowController mainWindowController)
    {
        super();
        this.selectedPortIdentifier = selectedPortIdentifier;
        setPortName(selectedPortIdentifier.getName());
        setDeviceConnected(false);
        this.mainWindowController = mainWindowController;
    }

    // connect to the selected port
    public Boolean connect()
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

            if (initIOStream())
            {
                inputDataBufferedReader = new BufferedReader(new InputStreamReader(input));
                serialPortReader = new SerialPortReader(inputDataBufferedReader, this);
                serialPortReadThread = new Thread(serialPortReader);
                serialPortReadThread.start();
            }
            return true;
        } catch (PortInUseException e)
        {
            log.error(this.selectedPortIdentifier + " is in use. (" + e.toString() + ")");
            disconnect();
            setDeviceConnected(false);
            return false;
        } catch (Exception e)
        {
            log.error("Failed to open " + this.selectedPortIdentifier + "(" + e.toString() + ")");
            disconnect();
            setDeviceConnected(false);
            return false;
        }
    }

    //disconnect the serial port
    public void disconnect()
    {
        //close the serial port
        try
        {
            serialPortReadThread.interrupt();
            serialPortReader.cancel();
            serialPort.close();
            input.close();
            output.close();
            setDeviceConnected(false);
        } catch (Exception e)
        {
            log.error("Failed to close " + serialPort.getName() + "(" + e.toString() + ")");
        }
    }

    //open the input and output streams
    private boolean initIOStream()
    {
        //return value for whether opening the streams is successful or not
        boolean successful = false;

        try
        {
            input = serialPort.getInputStream();
            output = serialPort.getOutputStream();

            successful = true;
            return successful;
        } catch (IOException e)
        {
            log.error("I/O Streams failed to open. (" + e.toString() + ")");
            return successful;
        }
    }

    //method that can be called to send data
    public void writeData(String data)
    {
        try
        {
            output.write(data.getBytes());
            output.flush();
            try
            {
                logToConsole(getDeviceName() + " < " + data);
            } catch (NullPointerException e)
            {
                log.error("Log not ready to write data (" + e.toString() + ")");
            }
        } catch (Exception e)
        {
            log.error("Failed to write data. (" + e.toString() + ")");
        }
    }

    public static class SerialPortReader implements Runnable
    {
        private volatile boolean isCancelled = false;
        BufferedReader in;
        public CDCDevice device;

        public SerialPortReader(BufferedReader in, CDCDevice device)
        {
            this.in = in;
            this.device = device;
        }

        @Override
        public void run()
        {
            while (!isCancelled)
            {
                try
                {
                    if (in.ready())
                    {
                        device.dataString = in.readLine();
                        device.dataStringId++;
                        try
                        {
                            device.logToConsole(device.getDeviceName() + " [" + device.dataStringId + "] > " + device.dataString);
                        } catch (NullPointerException e)
                        {
                            log.error("Log not ready to write data (" + e.toString() + ")");
                        } catch (IndexOutOfBoundsException e)
                        {
                            log.error("Log not ready to write data (" + e.toString() + ")");
                        }
                    }
                } catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        }

        public void cancel()
        {
            isCancelled = true;
        }

        @SuppressWarnings("unused")
        public void setDeviceName(String devName)
        {
            this.device.setDeviceName(devName);
        }
    }
}