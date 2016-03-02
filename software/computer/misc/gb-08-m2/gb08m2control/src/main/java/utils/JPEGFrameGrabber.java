package utils;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import org.opencv.core.CvType;
import org.opencv.core.Mat;

public class JPEGFrameGrabber
{
    public static Mat bufferedImageToMat(BufferedImage bimage)
    {
        byte[] data = ((DataBufferByte) bimage.getRaster().getDataBuffer()).getData();
        Mat mat = new Mat(bimage.getHeight(), bimage.getWidth(), CvType.CV_8UC3);
        mat.put(0, 0, data);
        return mat;
    }

    /**
     * Converts/writes a Mat into a BufferedImage.
     * 
     * @param matrix
     *            Mat of type CV_8UC3 or CV_8UC1
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
}
