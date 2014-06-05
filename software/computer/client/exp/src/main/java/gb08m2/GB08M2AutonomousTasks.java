package gb08m2;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.scene.SnapshotParameters;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.image.WritableImage;
import javafx.scene.paint.Color;
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
        int imgW;
        int imgH;
        int period = 0;

        Line centerLine;
        Line scanLine;
        
        List<Circle> points = new ArrayList<Circle>();

        double startX;
        double startY;

        Canvas canvas;
        GraphicsContext gc;

        public IRRangeFinderScanning(MainWindowController mainApp)
        {
            this.mainApp = mainApp;
            isStopped = false;

            //            imgW = (int) mainApp.IRRangeFinderPanel.getWidth() - 5;
            //            imgH = (int) (mainApp.IRRangeFinderPanel.getHeight() - 80);
            //            mainApp.irRangeFinderImageView.setCache(false);

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
                        /*
                        System.out.println(">>>>>>>>>>>>>>>>>>>" + mainApp.IRRangeFinderPane.getWidth());

                        canvas = new Canvas(imgW, imgH);
                        gc = canvas.getGraphicsContext2D();

                        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

                        double startX = canvas.getWidth() * (0.5);
                        double startY = canvas.getHeight();

                        gc.setFill(Color.RED);
                        gc.fillOval(startX - 10, startY - 10, 20, 20);

                        double length = 900 - mainApp.gb08m2IRRFdistance;

                        double angleRad = angle * Math.PI / 180;
                        double endX = startX + (length / 2) * Math.sin(angleRad);
                        double endY = startY + (length / 2) * Math.cos(angleRad);

                        if (mainApp.gb08m2IRRFdistance > 10)
                        {
                            gc.getPixelWriter().setColor((int) endX, (int) endY, Color.BLACK);
                            gc.getPixelWriter().setColor((int) endX + 1, (int) endY + 1, Color.BLACK);
                            gc.getPixelWriter().setColor((int) endX - 1, (int) endY - 1, Color.BLACK);
                            gc.getPixelWriter().setColor((int) endX + 1, (int) endY - 1, Color.BLACK);
                            gc.getPixelWriter().setColor((int) endX - 1, (int) endY + 1, Color.BLACK);
                        }

                        gc.setStroke(Color.BLACK);
                        gc.moveTo(startX, startY);
                        gc.lineTo(endX, endY);
                        gc.stroke();

                        WritableImage wi = new WritableImage(imgW, imgH);
                        canvas.snapshot(new SnapshotParameters(), wi);
                        mainApp.irRangeFinderImageView.setImage(wi);

                        //System.out.println("angle: " + angle);
                        //System.out.println("servoPosition: " + servoPosition);
                        //System.out.println("distance: " + mainApp.gb08m2IRRFdistance);
                        //System.out.println(irRangeFinderReadings.size());
                         */

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

                        if (mainApp.gb08m2IRRFdistance > 80)
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
                    if (angle < 245)
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
                    if (angle > 115)
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

    public void getScan()
    {
        mainApp.GB08M2CommandsClientListener.distanceScan = "";
        mainApp.gb08m2SendCmdOverTCPIP("RADARROTATIONSET#1450");
        mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCE");
        try
        {
            Thread.sleep(500);
        } catch (InterruptedException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        mainApp.gb08m2SendCmdOverTCPIP("GETDISTANCESCAN");
        try
        {
            Thread.sleep(3000);
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
                int distance_adc = Integer.parseInt(distanceScanReadingParts[1]);
                //distance_adc
                //250mm ~ 690
                //500mm ~ 380
                //1000mm ~ 190
                if (distance_adc > 150)
                {
                    double length = 900 - distance_adc;
                    double angleRad = angle * Math.PI / 180;
                    double endX = startX + (length / 2) * Math.sin(angleRad);
                    double endY = startY + (length / 2) * Math.cos(angleRad);

                    gc.getPixelWriter().setColor((int) endX, (int) endY, Color.BLACK);
                    gc.getPixelWriter().setColor((int) endX + 1, (int) endY + 1, Color.BLACK);
                    gc.getPixelWriter().setColor((int) endX - 1, (int) endY - 1, Color.BLACK);
                    gc.getPixelWriter().setColor((int) endX + 1, (int) endY - 1, Color.BLACK);
                    gc.getPixelWriter().setColor((int) endX - 1, (int) endY + 1, Color.BLACK);
                }
            }

            WritableImage wi = new WritableImage(imgW, imgH);
            canvas.snapshot(new SnapshotParameters(), wi);
            mainApp.irRangeFinderImageView.setImage(wi);
        }
    }
}
