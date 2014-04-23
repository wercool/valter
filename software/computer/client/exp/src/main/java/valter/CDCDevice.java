package valter;

import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;
import gnu.io.SerialPort;
import gnu.io.UnsupportedCommOperationException;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.util.Enumeration;

import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;

public class CDCDevice
{
    private final String OS;
    static String comPortPrefix = "ttyACM";
    private SerialPort devicePort = null;

    public SerialPort getDevicePort()
    {
        return devicePort;
    }

    public void setDevicePort(SerialPort devicePort)
    {
        this.devicePort = devicePort;
    }

    Boolean isConnected = false;
    private final SimpleStringProperty portName = new SimpleStringProperty();

    public String getPortName()
    {
        return portName.get();
    }

    public void setPortName(String portName)
    {
        this.portName.set(portName);
    }

    private final SimpleStringProperty deviceName = new SimpleStringProperty();

    public String getDeviceName()
    {
        return deviceName.get();
    }

    public void setDeviceName(String deviceName)
    {
        this.deviceName.set(deviceName);
    }

    private final SimpleBooleanProperty deviceConnected = new SimpleBooleanProperty();

    public boolean getDeviceConnected()
    {
        return deviceConnected.get();
    }

    public void setDeviceConnected(boolean deviceConnected)
    {
        this.deviceConnected.set(deviceConnected);
    }

    BufferedReader input = null;
    Thread serialPortReadThread = null;
    SerialPortReader serialPortReader;
    public volatile String dataLine = null;
    public volatile Long dataLineId = (long) 0;

    public CDCDevice()
    {
        OS = System.getProperty("os.name");
        if (OS.contains("Linux"))
        {
            comPortPrefix = "ttyACM";
        } else
        {
            comPortPrefix = "COM";
        }
    }

    public static ObservableList<CDCDevice> getDevices()
    {
        ObservableList<CDCDevice> CDCDevices = FXCollections.observableArrayList();

        Enumeration<?> portEnum = CommPortIdentifier.getPortIdentifiers();
        System.out.println("Available serial ports:");
        while (portEnum.hasMoreElements())
        {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            System.out.println(currPortId.getName());
        }
        System.out.println();
        portEnum = CommPortIdentifier.getPortIdentifiers();
        while (portEnum.hasMoreElements())
        {
            CDCDevice device = new CDCDevice();

            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            if (currPortId.getName().contains(comPortPrefix))
            {
                device.setPortName(currPortId.getName());
                try
                {
                    CommPort commPort = currPortId.open(device.getClass().getName(), 2000);
                    if (commPort instanceof SerialPort)
                    {
                        SerialPort serialPort = (SerialPort) commPort;
                        try
                        {
                            serialPort.setSerialPortParams(115200, SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
                            try
                            {
                                BufferedReader in = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
                                OutputStream out;
                                out = serialPort.getOutputStream();
                                out.write("GETID".getBytes());
                                out.close();
                                out = null;
                                try
                                {
                                    String result = null;
                                    while ((result = in.readLine()).contains("MSG:GETID"))
                                    {
                                        result = in.readLine();
                                        device.setDeviceName(result);
                                        device.setDevicePort(serialPort);
                                        device.setDeviceConnected(false);
                                        CDCDevices.add(device);
                                        break;
                                    }
                                    in.close();
                                    in = null;
                                } catch (IOException e)
                                {
                                    System.out.println(e.toString());
                                }
                            } catch (IOException e)
                            {
                                e.printStackTrace();
                            }
                        } catch (UnsupportedCommOperationException e)
                        {
                            System.out.println(currPortId.getName() + " does not support target port settings!");
                        }
                        serialPort.close();
                    }
                } catch (PortInUseException e)
                {
                    System.out.println(currPortId.getName() + " is already in use!");
                }
            }
        }
        return CDCDevices;
    }

    public void sendCommand(String cmd)
    {
        if (this.devicePort != null)
        {
            OutputStream out;
            try
            {
                out = devicePort.getOutputStream();
                System.out.println("[" + cmd + "]");
                out.write(cmd.getBytes());
                out.close();
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    public void connect()
    {
        try
        {
            input = new BufferedReader(new InputStreamReader(this.devicePort.getInputStream()));
        } catch (IOException e)
        {
            e.printStackTrace();
        }
        serialPortReader = new SerialPortReader(input, this);
        serialPortReadThread = new Thread(serialPortReader);
        serialPortReadThread.start();
        setDeviceConnected(true);
    }

    public void disconnect()
    {
        serialPortReadThread.interrupt();
        serialPortReader.cancel();
        this.devicePort.close();
        setDeviceConnected(false);
        System.out.println(this.deviceName + " disconnected.");
    }

    public static class SerialPortReader implements Runnable
    {
        private volatile boolean isCancelled = false;
        BufferedReader in;
        CDCDevice device;

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
                        device.dataLine = in.readLine();
                        System.out.println(device.dataLine);
                        device.dataLineId++;
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
    }
}
