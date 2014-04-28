package platform_control_p1.commands.switches;

import javafx.application.Platform;
import valter.PLATFORM_CONTROL_P1;
import app.MainWindowController;

import commands.CommandRunnable;

public class GetChargerConnected implements Runnable, CommandRunnable
{
    MainWindowController mainWindowController;
    PLATFORM_CONTROL_P1 platform_control_p1;
    private volatile boolean isCancelled = true;
    private volatile boolean isStopped = false;
    String reading;

    public GetChargerConnected(PLATFORM_CONTROL_P1 platform_control_p1)
    {
        this.mainWindowController = platform_control_p1.mainWindowController;
        this.platform_control_p1 = platform_control_p1;
        System.out.println(this.getClass().getName() + "()");
    }

    @Override
    public void run()
    {

        while (!isStopped)
        {
            if (isCancelled)
            {
                try
                {
                    Thread.sleep(100);
                } catch (InterruptedException e)
                {
                    //e.printStackTrace();
                }
                continue;
            }
            String dataString = this.platform_control_p1.cdcDevice.dataString;
            if (dataString.contains("INPUT2 CHANNEL [8]:"))
            {
                System.out.println(this.getClass().getName() + " CHARGER CONNECTED: " + reading);
                reading = dataString.substring(20, dataString.length());
                int readingInt = Integer.parseInt(reading);
                if (readingInt > 1000)
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargerConnectedRadioButton.setSelected(true);
                        }
                    });
                } else
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargerConnectedRadioButton.setSelected(false);
                        }
                    });
                }
            }
            try
            {
                Thread.sleep(100);
            } catch (InterruptedException e)
            {
                //e.printStackTrace();
            }
        }
    }

    @Override
    public void execute()
    {
        isCancelled = false;
        this.platform_control_p1.cdcDevice.writeData("SETINPUT2CHANNEL8");
        this.platform_control_p1.cdcDevice.writeData("STARTINPUT2READINGS");
        System.out.println(this.getClass().getName() + " execute()");
    }

    @Override
    public void cancel()
    {
        isCancelled = true;
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT2READINGS");
        System.out.println(this.getClass().getName() + " cancel()");
    }

    @Override
    public void stop()
    {
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT2READINGS");
        isStopped = true;
        System.out.println(this.getClass().getName() + " stop()");
    }

    @Override
    public void terminate()
    {
        cancel();
    }

    @Override
    public boolean isEcecuting()
    {
        return isCancelled;
    }
}