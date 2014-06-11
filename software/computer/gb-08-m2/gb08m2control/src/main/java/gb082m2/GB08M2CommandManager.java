package gb082m2;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;

import utils.PopupDialog;

public class GB08M2CommandManager
{
    GB08M2CommandSocketClient commandSocketClient;
    volatile boolean isConnected = false;
    Socket commandSocket;
    PrintWriter cmdOut;
    BufferedReader cmdIn;

    public GB08M2CommandManager()
    {
        isConnected = connect();

        if (isConnected)
        {
            try
            {
                cmdOut = new PrintWriter(commandSocket.getOutputStream(), true);
            } catch (IOException e)
            {
                //e.printStackTrace();
            }
            try
            {
                cmdIn = new BufferedReader(new InputStreamReader(commandSocket.getInputStream()));
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }

            commandSocketClient = new GB08M2CommandSocketClient();
            commandSocketClient.start();
        }
    }

    private boolean connect()
    {
        try
        {
            commandSocket = new Socket(GB08M2.hostname, GB08M2.commandPort);
        } catch (UnknownHostException e)
        {
            //e.printStackTrace();
            new PopupDialog(e.getMessage());
        } catch (IOException e)
        {
            //e.printStackTrace();
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
            cmdOut.close();
            cmdIn.close();
            cmdOut = null;
            cmdIn = null;
            commandSocket.close();
        } catch (IOException e)
        {
            //e.printStackTrace();
        }
        isConnected = false;
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
        public void run()
        {
            while (isConnected)
            {
                cmdOut.println("PING");
                cmdOut.flush();
                try
                {
                    Thread.sleep(200);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }
    }
}
