package utils;

import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.Robot;
import java.awt.Toolkit;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Random;
import java.util.Scanner;

import javafx.application.Platform;
import application.MainAppController;

public class Z800Tracker implements Runnable
{
    static final Z800Tracker instance = new Z800Tracker();
    Thread thread;
    String line;
    Scanner scan = new Scanner(System.in);
    static Robot robot;

    float prevX;
    float prevY;

    static int prevMouseX;
    static int prevMouseY;

    Process process;

    volatile boolean isStopped = false;

    public Z800Tracker()
    {
        try
        {
            robot = new Robot();
        } catch (AWTException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
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

    public boolean isRunning()
    {
        return !isStopped;
    }

    @Override
    public void run()
    {
        int tiltUp = 0;
        int tiltDown = 0;
        int panLeft = 0;
        int panRight = 0;

        Robot robot = null;
        try
        {
            robot = new Robot();
        } catch (AWTException e1)
        {
            // TODO Auto-generated catch block
            e1.printStackTrace();
        }
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        prevMouseX = (int) (screenSize.getWidth() / 2);
        prevMouseY = (int) (screenSize.getHeight() / 2);

        Random random = new Random();
        InputStream stdout;
        BufferedReader reader;
        System.out.println("INFO: Z800Tracker()");
        try
        {
            String[] cmd = { "/bin/bash", "-c", "echo \"dbrnjhbz1989\"| sudo -S /home/maska/workspace/bin/z800tracker.sh" };
            process = Runtime.getRuntime().exec(cmd);
            stdout = process.getInputStream();
            reader = new BufferedReader(new InputStreamReader(stdout));
            BufferedReader stdError = new BufferedReader(new InputStreamReader(process.getErrorStream()));

            while (!isStopped && (line = reader.readLine()) != null)
            {
                //System.out.println(line);
                String out[] = line.split(", ");
                if (robot != null)
                {
                    if (Float.parseFloat(out[3]) != prevX)
                    {
                        if (Float.parseFloat(out[3]) > prevX)
                        {
                            panRight = 0;
                            panLeft++;
                            if (panLeft > 4)
                            {
                                prevMouseX += 10;
                                Platform.runLater(new Runnable()
                                {
                                    @Override
                                    public void run()
                                    {
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() - 1);
                                    }
                                });
                                panLeft = 0;
                            }
                        } else
                        {
                            panLeft = 0;
                            panRight++;
                            if (panRight > 4)
                            {
                                prevMouseX -= 10;
                                Platform.runLater(new Runnable()
                                {
                                    @Override
                                    public void run()
                                    {
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                        MainAppController.instance.cameraPanSlider.setValue(MainAppController.instance.cameraPanSlider.getValue() + 1);
                                    }
                                });
                                panRight = 0;
                            }
                        }
                    }
                    if (Float.parseFloat(out[2]) != prevY)
                    {
                        if (Float.parseFloat(out[2]) > prevY)
                        {
                            tiltDown = 0;
                            tiltUp++;
                            if (tiltUp > 4)
                            {
                                prevMouseY += 10;
                                Platform.runLater(new Runnable()
                                {
                                    @Override
                                    public void run()
                                    {
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() + 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() + 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() + 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() + 1);
                                    }
                                });
                                tiltUp = 0;
                            }
                        } else
                        {
                            tiltUp = 0;
                            tiltDown++;
                            if (tiltDown > 4)
                            {
                                prevMouseY -= 10;
                                Platform.runLater(new Runnable()
                                {
                                    @Override
                                    public void run()
                                    {
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() - 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() - 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() - 1);
                                        MainAppController.instance.cameraTiltSlider.setValue(MainAppController.instance.cameraTiltSlider.getValue() - 1);
                                    }
                                });
                                tiltDown = 0;
                            }
                        }
                    }
                    if (MainAppController.instance.z800CursorMoveCheckBox.isSelected())
                    {
                        robot.mouseMove(prevMouseX, prevMouseY);
                    }
                    prevX = Float.parseFloat(out[3]);
                    prevY = Float.parseFloat(out[2]);
                }
            }
            // read any errors from the attempted command
            String s = null;
            System.out.println("Here is the standard error of the command (if any):\n");
            while ((s = stdError.readLine()) != null)
            {
                System.out.println(s);
            }
        } catch (IOException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static void resetZ800Tracker()
    {
        Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
        robot.mouseMove((int) (screenSize.getWidth() / 2), (int) (screenSize.getHeight() / 2));
        prevMouseX = (int) (screenSize.getWidth() / 2);
        prevMouseY = (int) (screenSize.getHeight() / 2);
    }
}
