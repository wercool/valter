package gb08m2;

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

                        Canvas canvas = new Canvas(346, 260);
                        WritableImage wi = new WritableImage(346, 260);
                        GraphicsContext gc = canvas.getGraphicsContext2D();
                        gc.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());

                        double startX = canvas.getWidth() * (0.5);
                        double startY = canvas.getHeight();

                        gc.setStroke(Color.BLACK);
                        gc.setFill(Color.RED);

                        gc.fillOval(startX - 10, startY - 10, 20, 20);

                        double angleRad = angle * Math.PI / 180;
                        double endX = startX + 200 * Math.sin(angleRad);
                        double endY = startY + 200 * Math.cos(angleRad);

                        gc.moveTo(startX, startY);
                        gc.lineTo(endX, endY);

                        gc.stroke();

                        canvas.snapshot(new SnapshotParameters(), wi);
                        mainApp.irRangeFinderImageView.setImage(wi);
                        mainApp.irRangeFinderImageView.setCache(false);

                        if (direction == 1)
                        {
                            if (angle < 265)
                            {
                                angle += 5;
                            } else
                            {
                                direction = 2;
                            }
                        } else
                        {
                            if (angle > 95)
                            {
                                angle -= 5;
                            } else
                            {
                                direction = 1;
                            }
                        }

                        System.out.println(angle);
                    }
                });
                try
                {
                    Thread.sleep(100);
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
}
