package application;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

public class RCTankCommandManager
{
    CommandSocketClient commandSocketClient;
    Socket commandSocket;
    PrintWriter cmdWriter;
    volatile ArrayList<String> cmdWriterSpool;
    String hostName;
    int commandPort;

    public RCTankCommandManager(String hostName, int commandPort)
    {
        this.hostName = hostName;
        this.commandPort = commandPort;

        commandSocket = new Socket();
        commandSocketClient = new CommandSocketClient();
        cmdWriterSpool = new ArrayList<String>();
    }

    public void connect()
    {
        try
        {
            commandSocket.connect(new InetSocketAddress(hostName, commandPort), 5000);

            if (commandSocket.isConnected())
            {
                commandSocketClient.start();
            }

            try
            {
                cmdWriter = new PrintWriter(commandSocket.getOutputStream(), true);
            } catch (IOException e)
            {
                e.printStackTrace();
            }

        } catch (UnknownHostException e)
        {
            e.printStackTrace();
        } catch (IOException e)
        {
            e.printStackTrace();
        } catch (IllegalArgumentException e)
        {
            e.printStackTrace();
        }
    }

    public void disconnect()
    {
        try
        {
            if (cmdWriter != null)
            {
                cmdWriter.close();
            }
            commandSocket.close();
            commandSocketClient.stop();
        } catch (IOException e)
        {
            e.printStackTrace();
        } catch (NullPointerException e)
        {
            e.printStackTrace();
        }
    }

    synchronized public void sendCommand(String cmd)
    {
        cmdWriterSpool.add(cmd);
    }

    class CommandSocketClient implements Runnable
    {
        Thread thread;
        boolean isStopped = false;

        public CommandSocketClient()
        {
            thread = new Thread(this);
        }

        public void start()
        {
            thread.start();
        }

        public void stop()
        {
            isStopped = true;
        }

        @Override
        synchronized public void run()
        {
            while (!isStopped)
            {
                if (cmdWriterSpool.size() > 0)
                {
                    cmdWriter.println(cmdWriterSpool.get(0));
                    System.out.println("CMD > " + cmdWriterSpool.get(0));
                    cmdWriterSpool.remove(0);
                }

                try
                {
                    Thread.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
