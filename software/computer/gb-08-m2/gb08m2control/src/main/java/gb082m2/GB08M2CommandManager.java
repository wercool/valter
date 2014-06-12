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
    boolean isConnected = false;
    Socket commandSocket;
    PrintWriter cmdWriter;
    BufferedReader cmdReader;
    volatile ArrayList<String> cmdWriterSpool;

    volatile String cmd = null;

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

            commandSocketClient = new GB08M2CommandSocketClient();
            commandSocketClient.start();
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
        }
        isConnected = false;
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
                    Thread.sleep(1);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
