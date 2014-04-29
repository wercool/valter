package platform_control_p1.listeners;

import interfaces.CommandRunnable;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import valter.PLATFORM_CONTROL_P1;
import app.MainWindowController;

public class ChargerListener implements Runnable, CommandRunnable
{
    MainWindowController mainWindowController;
    PLATFORM_CONTROL_P1 platform_control_p1;
    private volatile boolean isCancelled = true;
    private volatile boolean isStopped = false;
    String reading;
    List<String> commandsQueue = new ArrayList<String>();
    int commandsQueueIdx = 0;

    public ChargerListener(PLATFORM_CONTROL_P1 platform_control_p1)
    {
        this.mainWindowController = platform_control_p1.mainWindowController;
        this.platform_control_p1 = platform_control_p1;
        System.out.println(this.getClass().getName() + "()");

        commandsQueue.add("STARTINPUT2READINGS");
        commandsQueue.add("SETINPUT2CHANNEL6");
        commandsQueue.add("SETINPUT2CHANNEL7");
        commandsQueue.add("SETINPUT2CHANNEL8");
        commandsQueue.add("SETINPUT2CHANNEL9");
        commandsQueue.add("SETINPUT2CHANNEL10");
        commandsQueue.add("STOPINPUT2READINGS");
        commandsQueue.add("STARTINPUT1READINGS");
        commandsQueue.add("SETINPUT1CHANNEL8");
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
            if (commandsQueueIdx > commandsQueue.size() - 1)
                commandsQueueIdx = 0;

            this.platform_control_p1.cdcDevice.writeData(commandsQueue.get(commandsQueueIdx++));

            String dataString = this.platform_control_p1.cdcDevice.dataString;

            if (dataString.contains("INPUT1 CHANNEL [8]:"))
            {
                reading = dataString.substring(20, dataString.length());
                System.out.println(this.getClass().getName() + " Charger Voltage: " + reading);
                //int readingInt = Integer.parseInt(reading);
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        mainWindowController.chargerVoltageLabel.setText("Charger Voltage: " + reading);
                    }
                });
                continue;
            }
            if (dataString.contains("INPUT2 CHANNEL [6]:"))
            {
                reading = dataString.substring(20, dataString.length());
                System.out.println(this.getClass().getName() + " Charge In Progress [" + reading + "]");
                int readingInt = Integer.parseInt(reading);
                if (readingInt > 600)
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargeInProgressRadioButton.setSelected(true);
                        }
                    });
                } else
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargeInProgressRadioButton.setSelected(false);
                        }
                    });
                }
                continue;
            }
            if (dataString.contains("INPUT2 CHANNEL [7]:"))
            {
                reading = dataString.substring(20, dataString.length());
                System.out.println(this.getClass().getName() + " Charge Completed [" + reading + "]");
                int readingInt = Integer.parseInt(reading);
                if (readingInt > 700)
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargeCompleteRadioButton.setSelected(true);
                        }
                    });
                } else
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.chargeCompleteRadioButton.setSelected(false);
                        }
                    });
                }
                continue;
            }
            if (dataString.contains("INPUT2 CHANNEL [8]:"))
            {
                reading = dataString.substring(20, dataString.length());
                System.out.println(this.getClass().getName() + " Charge Connected [" + reading + "]");
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
                continue;
            }
            if (dataString.contains("INPUT2 CHANNEL [9]:"))
            {
                reading = dataString.substring(20, dataString.length());
                System.out.println(this.getClass().getName() + " Charger 35Ah Mode [" + reading + "]");
                int readingInt = Integer.parseInt(reading);
                if (readingInt > 1000)
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.charger35AhModeRadioButton.setSelected(true);
                        }
                    });
                } else
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.charger35AhModeRadioButton.setSelected(false);
                        }
                    });
                }
                continue;
            }
            if (dataString.contains("INPUT2 CHANNEL [10]:"))
            {
                reading = dataString.substring(21, dataString.length());
                System.out.println(this.getClass().getName() + " Charger 120Ah Mode [" + reading + "]");
                int readingInt = Integer.parseInt(reading);
                if (readingInt > 1000)
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.charger120AhModeRadioButton.setSelected(true);
                        }
                    });
                } else
                {
                    Platform.runLater(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            mainWindowController.charger120AhModeRadioButton.setSelected(false);
                        }
                    });
                }
                continue;
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
        System.out.println(this.getClass().getName() + " execute()");
    }

    @Override
    public void cancel()
    {
        isCancelled = true;
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT2READINGS");
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT1READINGS");
        System.out.println(this.getClass().getName() + " cancel()");
    }

    @Override
    public void stop()
    {
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT2READINGS");
        this.platform_control_p1.cdcDevice.writeData("STOPINPUT1READINGS");
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