package utils;

import java.util.Vector;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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
            Mat src;
            if (mat.channels() > 1)
            {
                src = new Mat(mat.height(), mat.width(), CvType.CV_8UC1);
                Imgproc.cvtColor(mat, src, Imgproc.COLOR_RGB2GRAY);
            } else
            {
                src = mat;
            }
            Vector<Mat> bgr_planes = new Vector<Mat>();
            Core.split(src, bgr_planes);
            MatOfInt histSize = new MatOfInt(256);
            final MatOfFloat histRange = new MatOfFloat(0f, 256f);
            boolean accumulate = false;
            Mat b_hist = new Mat();
            Imgproc.calcHist(bgr_planes, new MatOfInt(0), new Mat(), b_hist, histSize, histRange, accumulate);
            return b_hist;
        } catch (Exception e)
        {
            e.printStackTrace();
            return null;
        }
    }
}
