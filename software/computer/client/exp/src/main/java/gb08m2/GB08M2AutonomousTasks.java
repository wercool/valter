package gb08m2;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Line;
import app.MainWindowController;

public class GB08M2AutonomousTasks
{

    MainWindowController mainApp;
    VideoFrontProcessing videoFrontProcessing;
    VideoRearProcessing videoRearProcessing;
    IRRangeFinderScanning irRangeFinderScanning;
    public List<double[]> irRangeFinderReadings = new ArrayList<double[]>();

    public GB08M2AutonomousTasks(MainWindowController mainApp)
    {
        this.mainApp = mainApp;
    }

    public void startFrontVideoProcessing()
    {
        videoFrontProcessing = new VideoFrontProcessing(this.mainApp);
        new Thread(videoFrontProcessing).start();
    }

    public void stopFrontVideoProcessing()
    {
        videoFrontProcessing.stop();
    }

    public void startRearVideoProcessing()
    {
        videoRearProcessing = new VideoRearProcessing(this.mainApp);
        new Thread(videoRearProcessing).start();
    }

    public void stopRearVideoProcessing()
    {
        videoRearProcessing.stop();
    }

    public void startIRRangeFinderScanning()
    {
        irRangeFinderScanning = new IRRangeFinderScanning(this.mainApp);
        new Thread(irRangeFinderScanning).start();
    }

    public void stopIRRangeFinderScanning()
    {
        irRangeFinderScanning.stop();
    }

    class VideoFrontProcessing implements Runnable
    {
        MainWindowController mainApp;
        boolean isStopped;

        public VideoFrontProcessing(MainWindowController mainApp)
        {
            this.mainApp = mainApp;
            isStopped = false;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        mainApp.frontCameraFrameImageView.setImage(mainApp.mainVideoImageView.getImage());
                        mainApp.frontCameraFrameImageView.setCache(false);
                    }
                });
                try
                {
                    Thread.sleep(250);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }

        public void stop()
        {
            isStopped = true;
        }
    }

    class VideoRearProcessing implements Runnable
    {
        MainWindowController mainApp;
        boolean isStopped;

        public VideoRearProcessing(MainWindowController mainApp)
        {
            this.mainApp = mainApp;
            isStopped = false;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        mainApp.rearCameraFrameImageView.setImage(mainApp.rearVideoImageView.getImage());
                        mainApp.rearCameraFrameImageView.setCache(false);
                    }
                });
                try
                {
                    Thread.sleep(250);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }

        public void stop()
        {
            isStopped = true;
        }
    }

    class IRRangeFinderScanning implements Runnable
    {
        MainWindowController mainApp;
        boolean isStopped;
        int angle = 180;
        int direction = 1;
        double servoPosition = 1450;
        int period = 0;

        Line centerLine;
        Line scanLine;

        List<Circle> points = new ArrayList<Circle>();

        double startX;
        double startY;

        public IRRangeFinderScanning(MainWindowController mainApp)
        {
            this.mainApp = mainApp;
            isStopped = false;

            startX = mainApp.IRRangeFinderPane.getWidth() * (0.5);
            startY = mainApp.IRRangeFinderPane.getHeight();
            centerLine = new Line(startX, 0, startX, startY);
            centerLine.getStrokeDashArray().addAll(1d, 11d);
            mainApp.IRRangeFinderPane.getChildren().clear();
            mainApp.IRRangeFinderPane.getChildren().add(centerLine);
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        if (mainApp.IRRangeFinderPane.getWidth() * (0.5) != startX)
                        {
                            startX = mainApp.IRRangeFinderPane.getWidth() * (0.5);
                            startY = mainApp.IRRangeFinderPane.getHeight();
                            centerLine = new Line(startX, 0, startX, startY);
                            centerLine.getStrokeDashArray().addAll(1d, 11d);
                            mainApp.IRRangeFinderPane.getChildren().clear();
                            mainApp.IRRangeFinderPane.getChildren().add(centerLine);
                        }

                        double length = 900 - mainApp.gb08m2IRRFdistance;
                        double angleRad = angle * Math.PI / 180;
                        double endX = startX + (length / 2) * Math.sin(angleRad);
                        double endY = startY + (length / 2) * Math.cos(angleRad);

                        if (scanLine != null)
                        {
                            mainApp.IRRangeFinderPane.getChildren().remove(scanLine);
                        }
                        scanLine = new Line(startX, startY, endX, endY);
                        mainApp.IRRangeFinderPane.getChildren().add(scanLine);

                        if (mainApp.gb08m2IRRFdistance > 140)
                        {
                            Circle point = new Circle(endX, endY, 1);
                            mainApp.IRRangeFinderPane.getChildren().add(point);
                            points.add(point);
                        }

                        if (period == 2)
                        {
                            period = 0;
                            if (mainApp.clearIRRangeFinder.isSelected())
                            {
                                for (int i = 0; i < points.size(); i++)
                                {
                                    mainApp.IRRangeFinderPane.getChildren().remove(points.get(i));
                                }
                                points.clear();
                            }
                        }
                    }
                });

                if (direction == 1)
                {
                    if (angle < 240)
                    {
                        angle += 1;
                        servoPosition += 10;
                        mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#" + (int) servoPosition);
                        try
                        {
                            Thread.sleep(20);
                        } catch (InterruptedException e)
                        {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                        mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCE");
                    } else
                    {
                        direction = 2;
                        period++;
                    }
                } else
                {
                    if (angle > 120)
                    {
                        angle -= 1;
                        servoPosition -= 10;
                        mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#" + (int) servoPosition);
                        try
                        {
                            Thread.sleep(20);
                        } catch (InterruptedException e)
                        {
                            // TODO Auto-generated catch block
                            e.printStackTrace();
                        }
                        mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCE");
                    } else
                    {
                        direction = 1;
                        period++;
                    }
                }
            }
        }

        public void stop()
        {
            mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#1450");
            try
            {
                Thread.sleep(50);
            } catch (InterruptedException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONRESET");
            isStopped = true;
        }
    }
}
