package gb08m2;

import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;

import javafx.application.Platform;
import app.MainWindowController;

public class GB08M2CommandsClientListenerThread extends Thread
{
    protected MainWindowController mainWindowController;
    Socket clientSocket = null;
    volatile boolean isStopped = false;

    public GB08M2CommandsClientListenerThread(Socket socketClient, MainWindowController mainWindowController)
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

                    if (serverData != null)
                    {
                        if (serverData.contains("BATTERY VOLTAGE:"))
                        {
                            final String[] serverDataParts = serverData.split(":");
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    mainWindowController.batteryVoltageLabel.setText("Battery Voltage: " + serverDataParts[1].trim());
                                }
                            });
                            continue;
                        }
                        if (serverData.contains("LEFT ENCODER:"))
                        {
                            final String[] serverDataParts = serverData.split(":");
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    mainWindowController.leftEncoderLabel.setText("Left: " + serverDataParts[1].trim());
                                }
                            });
                            continue;
                        }
                        if (serverData.contains("RIGHT ENCODER:"))
                        {
                            final String[] serverDataParts = serverData.split(":");
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    mainWindowController.rightEncoderLabel.setText("Right: " + serverDataParts[1].trim());
                                }
                            });
                            continue;
                        }
                        if (serverData.contains("DISTANCE:"))
                        {
                            final String[] serverDataParts = serverData.split(":");
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    mainWindowController.gb08m2IRRFdistance = Integer.parseInt(serverDataParts[1].trim());
                                    mainWindowController.distanceLabel.setText("Distance: " + serverDataParts[1].trim());
                                }
                            });
                            continue;
                        }
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
            if ((rd == '\n') || (rd < 0))
                break;
            strIn[ix++] = (byte) rd;
        }
        if (ix == 0)
            return null;
        return new String(strIn, 0, ix);
    }
}
