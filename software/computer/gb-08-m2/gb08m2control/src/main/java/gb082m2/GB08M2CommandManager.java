package gb082m2;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

import utils.PopupDialog;

public class GB08M2CommandManager
{
    GB08M2CommandSocketClient commandSocketClient;
    GB08M2CommandParserClient commandParserClient;
    boolean isConnected = false;
    Socket commandSocket;
    PrintWriter cmdWriter;
    BufferedReader cmdReader;
    volatile ArrayList<String> cmdWriterSpool;
    volatile ArrayList<String> cmdReaderSpool;

    public GB08M2CommandManager()
    {
        isConnected = connect();

        if (isConnected)
        {
            try
            {
                cmdWriter = new PrintWriter(commandSocket.getOutputStream(), true);
            } catch (IOException e)
            {
                //e.printStackTrace();
            }
            try
            {
                cmdReader = new BufferedReader(new InputStreamReader(commandSocket.getInputStream()));
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            cmdWriterSpool = new ArrayList<String>();
            cmdReaderSpool = new ArrayList<String>();

            commandSocketClient = new GB08M2CommandSocketClient();
            commandSocketClient.start();

            commandParserClient = new GB08M2CommandParserClient();
            commandParserClient.start();
        }
    }

    private boolean connect()
    {
        try
        {
            commandSocket = new Socket();
            commandSocket.connect(new InetSocketAddress(GB08M2.hostname, GB08M2.commandPort), 5000);
        } catch (UnknownHostException e)
        {
            e.printStackTrace();
            new PopupDialog(e.getMessage());
        } catch (IOException e)
        {
            e.printStackTrace();
            new PopupDialog(e.getMessage());
        } catch (IllegalArgumentException e)
        {
            new PopupDialog(e.getMessage());
        }
        return (commandSocket != null) ? commandSocket.isConnected() : false;
    }

    public void disconnect()
    {
        isConnected = false;
        try
        {
            cmdWriter.close();
            cmdReader.close();
            commandSocket.close();
            cmdWriter = null;
            cmdReader = null;
        } catch (IOException e)
        {
            //e.printStackTrace();
        } catch (NullPointerException e)
        {
            //e.printStackTrace();
        }
    }

    synchronized public void sendCommand(String cmd)
    {
        cmdWriterSpool.add(cmd);
    }

    class GB08M2CommandSocketClient implements Runnable
    {
        Thread thread;

        public GB08M2CommandSocketClient()
        {
            thread = new Thread(this);
        }

        public void start()
        {
            thread.start();
        }

        @Override
        synchronized public void run()
        {
            while (isConnected)
            {
                if (cmdWriterSpool.size() > 0)
                {
                    cmdWriter.println(cmdWriterSpool.get(0));
                    System.out.println("CMD > " + cmdWriterSpool.get(0));
                    cmdWriterSpool.remove(0);
                }
                try
                {
                    if (cmdReader.ready())
                    {
                        cmdReaderSpool.add(cmdReader.readLine());
                    }
                } catch (IOException e1)
                {
                    e1.printStackTrace();
                }
                try
                {
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    class GB08M2CommandParserClient implements Runnable
    {
        Thread thread;

        public GB08M2CommandParserClient()
        {
            thread = new Thread(this);
        }

        public void start()
        {
            thread.start();
        }

        @Override
        synchronized public void run()
        {
            while (isConnected)
            {
                if (cmdReaderSpool.size() > 0)
                {
                    System.out.println("RES < " + cmdReaderSpool.get(0));
                    String[] cmdResultParts = cmdReaderSpool.get(0).split(":");
                    try
                    {
                        if (cmdResultParts.length > 1)
                        {
                            switch (cmdResultParts[0])
                            {
                            //Motors
                                case GB08M2.frontLeftMotorCurrentResultPrefix:
                                    GB08M2.getInstance().setFrontLeftMotorCurrent(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                case GB08M2.frontRightMotorCurrentResultPrefix:
                                    GB08M2.getInstance().setFrontRightMotorCurrent(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                case GB08M2.rearLeftMotorCurrentResultPrefix:
                                    GB08M2.getInstance().setRearLeftMotorCurrent(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                case GB08M2.rearRightMotorCurrentResultPrefix:
                                    GB08M2.getInstance().setRearRightMotorCurrent(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                            //Encoders
                                case GB08M2.leftEncoderTicksResultPrefix:
                                    GB08M2.getInstance().setLeftEncoderTicks(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                case GB08M2.rightEncoderTicksResultPrefix:
                                    GB08M2.getInstance().setRightEncoderTicks(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                //Distance scanner
                                case GB08M2.distanceScannerResultPrefix:
                                    GB08M2.getInstance().setDistanceScannerDistance(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                                //Battery
                                case GB08M2.batteryVoltageResultPrefix:
                                    GB08M2.getInstance().setBatteryVoltage(Integer.parseInt(cmdResultParts[1]));
                                    cmdReaderSpool.remove(0);
                                break;
                            }
                        }
                    } catch (NumberFormatException e)
                    {
                        e.printStackTrace();
                    } catch (IndexOutOfBoundsException e)
                    {
                        e.printStackTrace();
                    }
                }

                try
                {
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
