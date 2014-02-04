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

public class CDCDevice
{
    String OS;
    String comPortPrefix = "ttyACM";
    SerialPort devicePort = null;
    Boolean isConnected = false;
    String portName;
    BufferedReader input = null;
    Thread serialPortReadThread = null;
    SerialPortReader serialPortReader;
    public volatile String dataLine = null;
    public volatile Long dataLineId = (long) 0;

    public CDCDevice()
    {
        OS = System.getProperty("os.name");
        System.out.println(OS);
        if (OS.contains("Linux"))
        {
            comPortPrefix = "ttyACM";
        }
        else
        {
            comPortPrefix = "COM";
        }
    }

    public Boolean findDevicePort()
    {
        Enumeration<?> portEnum = CommPortIdentifier.getPortIdentifiers();
        System.out.println("Available serial ports:");
        while (portEnum.hasMoreElements())
        {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            System.out.println(currPortId.getName());
        }
        portEnum = CommPortIdentifier.getPortIdentifiers();
        while (portEnum.hasMoreElements())
        {
            CommPortIdentifier currPortId = (CommPortIdentifier) portEnum.nextElement();
            if (currPortId.getName().contains(comPortPrefix))
            {
                portName = currPortId.getName();
                try
                {
                    CommPort commPort = currPortId.open(this.getClass().getName(), 2000);
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
                                System.out.println("Sending GETID...");
                                out.write("GETID".getBytes());
                                out.close();
                                out = null;
                                try
                                {
                                    String result = null;
                                    if ((result = in.readLine()) != null)
                                    {
                                        System.out.println(result);
                                        if (result.contains("MPD DEVICE"))
                                        {
                                            this.devicePort = serialPort;
                                            System.out.println("MPD device is on the " + portName + " port");
                                            in.close();
                                            input = new BufferedReader(new InputStreamReader(serialPort.getInputStream()));
                                            serialPortReader = new SerialPortReader(input, this);
                                            serialPortReadThread = new Thread(serialPortReader);
                                            serialPortReadThread.start();
                                            this.isConnected = true;
                                            return true;
                                        }
                                    }
                                    in.close();
                                    in = null;
                                }
                                catch (IOException e)
                                {
                                    System.out.println(portName + " - no MPD device connected");
                                }
                            }
                            catch (IOException e)
                            {
                                e.printStackTrace();
                            }
                        }
                        catch (UnsupportedCommOperationException e)
                        {
                            System.out.println(currPortId.getName() + " does not support target port settings!");
                        }
                        serialPort.close();
                    }
                }
                catch (PortInUseException e)
                {
                    System.out.println(currPortId.getName() + " is already in use!");
                }
            }
        }
        return false;
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
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }
        }
    }

    public void disconnect()
    {
        serialPortReadThread.interrupt();
        serialPortReader.cancel();
        sendCommand("STOPREADINGS");
        sendCommand("STOP");
        this.devicePort.close();
        this.isConnected = false;
        System.out.println("Device disconnected.");
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
        public void run ()
        {
            while(!isCancelled)
            {
                try
                {
                    if (in.ready())
                    {
                        device.dataLine = in.readLine();
                        System.out.println(device.dataLine);
                        device.dataLineId++;
                    }
                }
                catch (IOException e)
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
