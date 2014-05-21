package app;


import java.awt.image.BufferedImage;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;

public class MJPGStreamGrabber
{

    private String streamURL;
    private ImageView streamOutputView;
    public boolean isCapturing = false;
    private Mat MJPGMatFrame;
    private BufferedImage MJPGframe;
    private VideoCapture capture;
    private Thread captureFrameThread;

    public MJPGStreamGrabber(String streamURL, ImageView streamOutputView)
    {
        this.streamURL = streamURL;
        this.streamOutputView = streamOutputView;
        MJPGMatFrame = new Mat();
    }

    public void startCapture()
    {
        capture = new VideoCapture();
        capture.open(streamURL);
        isCapturing = true;
        captureFrameThread = new Thread(new RunnableCaptureFrame(), "frontCameraCaptureFrameThread");
        captureFrameThread.start();
    }

    public void stopCapture()
    {
        isCapturing = false;
        if (capture != null)
        {
            if (capture.isOpened())
            {
                capture.release();
            }
        }
    }

    private void outputFrame()
    {
        CascadeClassifier objectDetector = new CascadeClassifier("/home/maska/Desktop/classifier/cascade.xml");
        Mat mRgba = new Mat();
        Mat mGrey = new Mat();
        MatOfRect detectedObjects = new MatOfRect();
        MJPGMatFrame.copyTo(mRgba);
        MJPGMatFrame.copyTo(mGrey);
        Imgproc.cvtColor(mRgba, mGrey, Imgproc.COLOR_BGR2GRAY);
        Imgproc.equalizeHist(mGrey, mGrey);
        objectDetector.detectMultiScale(mGrey, detectedObjects);
        //Size minSize = new Size(40, 20);
        //Size maxSize = new Size(50, 100);
        //objectDetector.detectMultiScale(mGrey, detectedObjects, 1.1, 2, Objdetect.CASCADE_DO_CANNY_PRUNING | Objdetect.CASCADE_DO_ROUGH_SEARCH, minSize, maxSize);
        System.out.println(String.format("Detected %s objects", detectedObjects.toArray().length));

        Rect[] detectedObjectsArray = detectedObjects.toArray();
        for (int i = 0; i < detectedObjectsArray.length; i++)
        {
            Core.rectangle(mRgba, detectedObjectsArray[i].tl(), detectedObjectsArray[i].br(), new Scalar(0, 255, 0), 1);
        }

        MJPGframe = matToBufferedImage(mRgba);

        Image image = SwingFXUtils.toFXImage(MJPGframe, null);
        streamOutputView.setImage(image);
    }

    /**  
     * Converts/writes a Mat into a BufferedImage.
     *  
     * @param matrix Mat of type CV_8UC3 or CV_8UC1
     * @return BufferedImage of type TYPE_3BYTE_BGR or TYPE_BYTE_GRAY
     */
    public static BufferedImage matToBufferedImage(Mat matrix)
    {
        int cols = matrix.cols();
        int rows = matrix.rows();
        int elemSize = (int) matrix.elemSize();
        byte[] data = new byte[cols * rows * elemSize];
        int type;
        matrix.get(0, 0, data);
        switch (matrix.channels())
        {
            case 1:
                type = BufferedImage.TYPE_BYTE_GRAY;
                break;
            case 3:
                type = BufferedImage.TYPE_3BYTE_BGR;
                // bgr to rgb  
                byte b;
                for (int i = 0; i < data.length; i = i + 3)
                {
                    b = data[i];
                    data[i] = data[i + 2];
                    data[i + 2] = b;
                }
                break;
            default:
                return null;
        }
        BufferedImage image2 = new BufferedImage(cols, rows, type);
        image2.getRaster().setDataElements(0, 0, cols, rows, data);
        return image2;
    }
    
    private class RunnableCaptureFrame implements Runnable 
    {
        @Override
        public void run()
        {
            while (isCapturing)
            {
                capture.read(MJPGMatFrame);
                if (!MJPGMatFrame.empty())
                {
                    //MJPGframe = matToBufferedImage(MJPGMatFrame);
                    outputFrame();
                    try
                    {
                        Thread.sleep(50);
                    } catch (InterruptedException e)
                    {
                        e.printStackTrace();
                    }
                }
            }
        }
    };
}
