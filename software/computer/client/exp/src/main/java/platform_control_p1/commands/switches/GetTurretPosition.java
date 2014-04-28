package platform_control_p1.commands.switches;

import javafx.application.Platform;
import valter.PLATFORM_CONTROL_P1;
import app.MainWindowController;

import commands.CommandRunnable;

public class GetTurretPosition implements Runnable, CommandRunnable
{
    MainWindowController mainWindowController;
    PLATFORM_CONTROL_P1 platform_control_p1;
    private volatile boolean isCancelled = true;
    private volatile boolean isStopped = false;
    String reading;

    public GetTurretPosition(PLATFORM_CONTROL_P1 platform_control_p1)
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
            if (dataString.contains("TURRET:"))
            {
                reading = dataString.substring(8, dataString.length());
                System.out.println(this.getClass().getName() + " TURRET POSITION: " + reading);
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        mainWindowController.currentTurretPositionLabel.setText("Current Position: " + reading);
                    }
                });
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
        this.platform_control_p1.cdcDevice.writeData("STARTTURRETREADINGS");
        System.out.println(this.getClass().getName() + " execute()");
    }

    @Override
    public void cancel()
    {
        isCancelled = true;
        this.platform_control_p1.cdcDevice.writeData("STOPTURRETREADINGS");
        System.out.println(this.getClass().getName() + " cancel()");
    }

    @Override
    public void stop()
    {
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