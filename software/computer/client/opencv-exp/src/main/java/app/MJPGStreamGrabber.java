package app;


import java.awt.image.BufferedImage;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import org.opencv.core.Mat;
import org.opencv.highgui.VideoCapture;

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
        capture.release();
    }

    private void outputFrame()
    {
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
                    MJPGframe = matToBufferedImage(MJPGMatFrame);
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
