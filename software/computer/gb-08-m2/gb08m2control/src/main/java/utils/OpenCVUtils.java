package utils;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.core.MatOfInt;
import org.opencv.imgproc.Imgproc;

public class OpenCVUtils
{
    public static double compareTwoOpenCVMatHist(Mat mat1, Mat mat2)
    {
        Mat H1 = getColorHistogram(mat1);
        Mat H2 = getColorHistogram(mat2);

        return Imgproc.compareHist(H1, H2, Imgproc.CV_COMP_BHATTACHARYYA);
    }

    public static Mat getColorHistogram(Mat mat)
    {
        try
        {
            Mat src = new Mat();
            Imgproc.cvtColor(mat, src, Imgproc.COLOR_BGR2HSV);
            List<Mat> imagesList = new ArrayList<>();
            imagesList.add(src);
            //Mat tempChannel=new Mat(new Size(3,1),org.opencv.core.CvType.
            int histSizeArray[] =
                { 256, 256, 256 };
            int channelArray[] =
                { 0, 1, 2 };
            MatOfInt channels = new MatOfInt(channelArray);
            //mChannels = new MatOfInt[] { new MatOfInt(0), new MatOfInt(1), new MatOfInt(2) };
            Mat hist = new Mat();
            MatOfInt histSize = new MatOfInt(256);
            //histSize.fromArray(histSizeArray);
            //histSize = new MatOfInt(256);
            float hrangesArray[] =
                { 0.0f, 255.0f };
            MatOfFloat ranges = new MatOfFloat(0.0f, 255.0f);
            org.opencv.imgproc.Imgproc.calcHist(imagesList, channels, new Mat(), hist, histSize, ranges);
            return hist;
        } catch (Exception e)
        {
            e.printStackTrace();
            return null;
        }
    }
}
