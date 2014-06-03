package gb08m2;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.scene.SnapshotParameters;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
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

        public IRRangeFinderScanning(MainWindowController mainApp)
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
                        int imgW = (int) mainApp.IRRangeFinderPanel.getWidth() - 5;
                        int imgH = (int) (mainApp.IRRangeFinderPanel.getHeight() - 80);
                        Canvas canvas = new Canvas(imgW, imgH);
                        GraphicsContext gc = canvas.getGraphicsContext2D();
                        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

                        double startX = canvas.getWidth() * (0.5);
                        double startY = canvas.getHeight();

                        gc.setFill(Color.RED);
                        gc.fillOval(startX - 10, startY - 10, 20, 20);

                        double length = 900 - mainApp.gb08m2IRRFdistance;

                        double angleRad = angle * Math.PI / 180;
                        double endX = startX + (length / 2) * Math.sin(angleRad);
                        double endY = startY + (length / 2) * Math.cos(angleRad);

                        irRangeFinderReadings.add(new double[] { endX, endY });

                        gc.setStroke(Color.BLACK);
                        gc.moveTo(startX, startY);
                        gc.lineTo(endX, endY);
                        gc.stroke();

                        for (int i = 0; i < irRangeFinderReadings.size(); i++)
                        {
                            double[] point = irRangeFinderReadings.get(i);
                            gc.getPixelWriter().setColor((int) point[0], (int) point[1], Color.GREEN);
                            gc.getPixelWriter().setColor((int) point[0] - 1, (int) point[1] - 1, Color.GREEN);
                            gc.getPixelWriter().setColor((int) point[0] + 1, (int) point[1] + 1, Color.GREEN);
                            gc.getPixelWriter().setColor((int) point[0] + 1, (int) point[1] - 1, Color.GREEN);
                            gc.getPixelWriter().setColor((int) point[0] - 1, (int) point[1] + 1, Color.GREEN);
                        }

                        WritableImage wi = new WritableImage(imgW, imgH);
                        canvas.snapshot(new SnapshotParameters(), wi);
                        mainApp.irRangeFinderImageView.setImage(wi);
                        //mainApp.irRangeFinderImageView.setCache(false);

                        if (direction == 1)
                        {
                            if (angle < 245)
                            {
                                angle += 1;
                                servoPosition += 10;
                                mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#" + (int) servoPosition);
                                //mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCE");
                            } else
                            {
                                direction = 2;
                            }
                        } else
                        {
                            if (angle > 115)
                            {
                                angle -= 1;
                                servoPosition -= 10;
                                mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#" + (int) servoPosition);
                                //mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCE");
                            } else
                            {
                                direction = 1;
                            }
                        }

                        //System.out.println("angle: " + angle);
                        //System.out.println("servoPosition: " + servoPosition);
                        //System.out.println("distance: " + mainApp.gb08m2IRRFdistance);
                        //System.out.println(irRangeFinderReadings.size());
                    }
                });
                try
                {
                    Thread.sleep(50);
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

    public void getScan()
    {
        mainApp.GB08M2CommandsClientListener.distanceScan = "";
        mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCESCAN");
        try
        {
            Thread.sleep(4000);
        } catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        if (mainApp.GB08M2CommandsClientListener.distanceScan.length() > 1000)
        {
            final String[] distanceScanParts = mainApp.GB08M2CommandsClientListener.distanceScan.split("SCAN:");
            final String[] distanceScanReadings = distanceScanParts[1].trim().split(";");

            int imgW = (int) mainApp.IRRangeFinderPanel.getWidth() - 5;
            int imgH = (int) (mainApp.IRRangeFinderPanel.getHeight() - 80);
            Canvas canvas = new Canvas(imgW, imgH);
            GraphicsContext gc = canvas.getGraphicsContext2D();
            gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

            double startX = canvas.getWidth() * (0.5);
            double startY = canvas.getHeight();

            gc.setFill(Color.RED);
            gc.fillOval(startX - 10, startY - 10, 20, 20);

            for (int i = 0; i < distanceScanReadings.length; i++)
            {
                final String[] distanceScanReadingParts = distanceScanReadings[i].split(",");
                //System.out.println(distanceScanReadingParts[0] + " -> " + distanceScanReadingParts[1]);
                int angle = Integer.parseInt(distanceScanReadingParts[0]);
                double length = 900 - Integer.parseInt(distanceScanReadingParts[1]);
                double angleRad = angle * Math.PI / 180;
                double endX = startX + (length / 2) * Math.sin(angleRad);
                double endY = startY + (length / 2) * Math.cos(angleRad);

                gc.getPixelWriter().setColor((int) endX, (int) endY, Color.GREEN);
                gc.getPixelWriter().setColor((int) endX + 1, (int) endY + 1, Color.GREEN);
                gc.getPixelWriter().setColor((int) endX - 1, (int) endY - 1, Color.GREEN);
                gc.getPixelWriter().setColor((int) endX + 1, (int) endY - 1, Color.GREEN);
                gc.getPixelWriter().setColor((int) endX - 1, (int) endY + 1, Color.GREEN);
            }

            WritableImage wi = new WritableImage(imgW, imgH);
            canvas.snapshot(new SnapshotParameters(), wi);
            mainApp.irRangeFinderImageView.setImage(wi);
        }
    }
}
