package valter;

import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;

import javafx.application.Platform;
import app.MainWindowController;

public class CommandsClientListenerThread extends Thread
{
    protected MainWindowController mainWindowController;
    Socket clientSocket = null;
    volatile boolean isStopped = false;

    public CommandsClientListenerThread(Socket socketClient, MainWindowController mainWindowController)
    {
        this.mainWindowController = mainWindowController;
        this.clientSocket = socketClient;
    }
    
    @Override
    public void run()
    {
        try
        {
            InputStream input = clientSocket.getInputStream();
            while (!this.isStopped)
            {
                String serverData = "";
                if (input.available() > 0)
                {
                    serverData = getString(input);

                    System.out.println(clientSocket.getRemoteSocketAddress() + " → " + serverData);

                    //Process commands
                    if (serverData.contains("PING"))
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                mainWindowController.pingPongBtn.setText("PING");
                            }
                        });
                        continue;
                    }
                    if (serverData.contains("PONG"))
                    {
                        Platform.runLater(new Runnable()
                        {
                            @Override
                            public void run()
                            {
                                mainWindowController.pingPongBtn.setText("PONG");
                            }
                        });
                        continue;
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
            input.close();
        } catch (IOException e1)
        {
            e1.printStackTrace();
        }
    }

    public void stopListener()
    {
        this.isStopped = true;
    }

    private String getString(InputStream in) throws IOException
    {
        byte[] strIn = new byte[100];
        int ix = 0;
        while (true)
        {
            int rd = in.read();
            if ((rd == (int) '\n') || (rd < 0))
                break;
            strIn[ix++] = (byte) rd;
        }
        if (ix == 0)
            return null;
        return new String(strIn, 0, ix);
    }
}
