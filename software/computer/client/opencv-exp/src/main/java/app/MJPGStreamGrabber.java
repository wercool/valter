package app;


import java.awt.image.BufferedImage;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfRect;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.highgui.Highgui;
import org.opencv.highgui.VideoCapture;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.objdetect.Objdetect;

public class MJPGStreamGrabber
{

    private String streamURL;
    private ImageView streamOutputView;
    public boolean isCapturing = false;
    private Mat MJPGMatFrame;
    private BufferedImage MJPGframe;
    private VideoCapture capture;
    private Thread captureFrameThread;

    //HAAR settings
    private int neighboursNum = 1;
    Mat template;

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

        //template = Highgui.imread("/home/maska/Desktop/door-flap.jpg");
        template = Highgui.imread("/home/maska/Desktop/object.jpg");
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
        //CascadeClassifier objectDetector = new CascadeClassifier("/home/maska/Desktop/classifier/cascade.xml");
        CascadeClassifier objectDetector = new CascadeClassifier("/home/maska/Desktop/door/cascade.xml");
        Mat mRgba = new Mat();
        Mat mGrey = new Mat();
        MatOfRect detectedObjects = new MatOfRect();
        MJPGMatFrame.copyTo(mRgba);
        MJPGMatFrame.copyTo(mGrey);
        Imgproc.cvtColor(mRgba, mGrey, Imgproc.COLOR_BGR2GRAY);
        Imgproc.equalizeHist(mGrey, mGrey);
        //        objectDetector.detectMultiScale(mGrey, detectedObjects);
        //        Size minSize = new Size(50, 50);
        //        Size maxSize = new Size(200, 200);
        //        objectDetector.detectMultiScale(mGrey, detectedObjects, 1.1, 3, Objdetect.CASCADE_DO_CANNY_PRUNING | Objdetect.CASCADE_DO_ROUGH_SEARCH, minSize, maxSize);

        MatOfInt rejectLevels = new MatOfInt();
        MatOfDouble levelWeights = new MatOfDouble();
        double scaleFactor = 1.1;
        //int minNeighbors = 37;
        int deprecatedFlag = 0;
        Size minSize = new Size(100, 100);
        Size maxSize = new Size(400, 400);
        boolean outputRejectLevels = false;

        //objectDetector.detectMultiScale(mGrey, detectedObjects, rejectLevels, levelWeights);
        objectDetector.detectMultiScale(mGrey, detectedObjects, rejectLevels, levelWeights, scaleFactor, neighboursNum, Objdetect.CASCADE_DO_CANNY_PRUNING | Objdetect.CASCADE_DO_ROUGH_SEARCH, new Size(), new Size(), outputRejectLevels);

        double[] weights = new double[0];

        if (levelWeights.size().width > 0)
        {
            weights = levelWeights.toArray();
        }

        System.out.println(String.format("Detected %s objects", detectedObjects.toArray().length));

        Rect[] detectedObjectsArray = detectedObjects.toArray();
        for (int i = 0; i < detectedObjectsArray.length; i++)
        {
            if (levelWeights.size().width > 0)
            {
                System.out.println("weight [" + i + "]: " + weights[i]);
            }
            Core.rectangle(mRgba, detectedObjectsArray[i].tl(), detectedObjectsArray[i].br(), new Scalar(0, 255, 0), 1);
            Core.putText(mRgba, Integer.toString(i), detectedObjectsArray[i].tl(), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0));
        }

        //Core.rectangle(mRgba, detectedObjectsArray[detectedObjectsArray.length - 1].tl(), detectedObjectsArray[detectedObjectsArray.length - 1].br(), new Scalar(0, 255, 0), 1);
        //Core.putText(mRgba, Integer.toString(detectedObjectsArray.length - 1), detectedObjectsArray[detectedObjectsArray.length - 1].tl(), Core.FONT_HERSHEY_SIMPLEX, 1, new Scalar(0, 255, 0));

        //        if (detectedObjectsArray.length > 1)
        //        {
        //            neighboursNum++;
        //            System.out.println(String.format("neighboursNum = %d", neighboursNum));
        //        }

        //Template Matching
        int result_cols = mRgba.cols() - template.cols() + 1;
        int result_rows = mRgba.rows() - template.rows() + 1;
        Mat result = new Mat(result_rows, result_cols, CvType.CV_32FC1);

        // / Do the Matching and Normalize
        Imgproc.matchTemplate(mRgba, template, result, Imgproc.TM_CCOEFF);
        Core.normalize(result, result, 0, 1, Core.NORM_MINMAX, -1, new Mat());

        // / Localizing the best match with minMaxLoc
        MinMaxLocResult mmr = Core.minMaxLoc(result);
        Point matchLoc = mmr.maxLoc;

        Core.rectangle(mRgba, matchLoc, new Point(matchLoc.x + template.cols(), matchLoc.y + template.rows()), new Scalar(0, 0, 255));

        //Frame Output
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
