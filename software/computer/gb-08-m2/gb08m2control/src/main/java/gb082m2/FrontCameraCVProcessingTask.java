package gb082m2;

import java.awt.image.BufferedImage;
import java.util.LinkedList;
import java.util.List;

import javafx.scene.shape.Rectangle;

import org.opencv.core.Core;
import org.opencv.core.Core.MinMaxLocResult;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.features2d.DMatch;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;

import utils.JPEGFrameGrabber;

public class FrontCameraCVProcessingTask implements Runnable
{
    Thread thread;
    boolean isStopped = false;
    boolean isPaused = true;
    Mat prevROIMat;

    FeatureDetector detector = FeatureDetector.create(FeatureDetector.SURF);
    DescriptorExtractor extractor = DescriptorExtractor.create(FeatureDetector.SURF);
    DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE);

    MatOfKeyPoint ROIDetectorKeypoints;
    Mat ROIDescriptorMat;

    public FrontCameraCVProcessingTask()
    {
        new Thread(this).start();
    }

    public void stop()
    {
        isStopped = true;
    }

    public void pause()
    {
        isPaused = true;
    }

    public boolean isPaused()
    {
        return isPaused;
    }

    public void resume()
    {
        isPaused = false;
    }

    @Override
    public void run()
    {
        while (!isStopped)
        {
            try
            {

                BufferedImage frontCameraFrame = GB08M2.getInstance().getFrontCameraFrameBufferedImage();
                if (frontCameraFrame != null)
                {
                    GB08M2.getInstance().setFrontCameraMat(JPEGFrameGrabber.bufferedImageToMat(frontCameraFrame));

                    Mat frontCameraFrameMat = GB08M2.getInstance().getFrontCameraMat();

                    if (!GB08M2AutomatedManager.patternTracking)
                    {
                        Rectangle ROI = GB08M2.getInstance().getFrontCameraROIRectangle();
                        if (ROI != null)
                        {
                            Rect ROIRect = new Rect((int) ROI.getX(), (int) ROI.getY(), (int) ROI.getWidth(), (int) ROI.getHeight());
                            Mat frontCameraROIMat = new Mat(frontCameraFrameMat, ROIRect);
                            GB08M2.getInstance().setFrontCameraROIMat(frontCameraROIMat);

                            Mat frontCameraROIGrayscaleMat = new Mat(frontCameraROIMat.width(), frontCameraROIMat.height(), CvType.CV_8UC1);
                            Imgproc.cvtColor(frontCameraROIMat, frontCameraROIGrayscaleMat, Imgproc.COLOR_RGB2GRAY);

                            ROIDetectorKeypoints = new MatOfKeyPoint();
                            detector.detect(frontCameraROIGrayscaleMat, ROIDetectorKeypoints);
                            ROIDescriptorMat = new Mat();
                            extractor.compute(frontCameraROIGrayscaleMat, ROIDetectorKeypoints, ROIDescriptorMat);

                            Mat frontCameraROISURFMat = new Mat();
                            Features2d.drawKeypoints(frontCameraROIGrayscaleMat, ROIDetectorKeypoints, frontCameraROISURFMat);

                            GB08M2.getInstance().setFrontCameraROIGrayscaleMat(frontCameraROISURFMat);
                        }
                    } else
                    //pattern tracking
                    {
                        Mat ROIMat = GB08M2.getInstance().getFrontCameraROIMat();
                        if (ROIMat != null)
                        {
                            //Template Match
                            int result_cols = frontCameraFrameMat.cols() - ROIMat.cols() + 1;
                            int result_rows = frontCameraFrameMat.rows() - ROIMat.rows() + 1;
                            Mat result = new Mat(result_rows, result_cols, CvType.CV_32FC1);

                            // Do the Matching and Normalize
                            Imgproc.matchTemplate(frontCameraFrameMat, ROIMat, result, Imgproc.TM_CCORR_NORMED);

                            Core.normalize(result, result, 0, 1, Core.NORM_MINMAX, -1, new Mat());

                            // Localizing the best match with minMaxLoc
                            MinMaxLocResult mmr = Core.minMaxLoc(result);

                            Point matchLoc;

                            if (Imgproc.TM_CCOEFF == Imgproc.TM_SQDIFF || Imgproc.TM_CCOEFF == Imgproc.TM_SQDIFF_NORMED)
                            {
                                matchLoc = mmr.minLoc;
                            } else
                            {
                                matchLoc = mmr.maxLoc;
                            }

                            Rect ROITemplateRect = new Rect((int) matchLoc.x, (int) matchLoc.y, ROIMat.cols(), ROIMat.rows());
                            Mat frontCameraROIDetectedMat = new Mat(frontCameraFrameMat, ROITemplateRect);
                            GB08M2.getInstance().setFrontCameraROIDetectedMat(frontCameraROIDetectedMat);

                            Mat frontCameraROIDetectedGrayscaleMat = new Mat(frontCameraROIDetectedMat.width(), frontCameraROIDetectedMat.height(), CvType.CV_8UC1);
                            Imgproc.cvtColor(frontCameraROIDetectedMat, frontCameraROIDetectedGrayscaleMat, Imgproc.COLOR_RGB2GRAY);

                            //SUFR 
                            Mat frontCameraGrayscaleMat = new Mat(frontCameraFrameMat.width(), frontCameraFrameMat.height(), CvType.CV_8UC1);
                            Imgproc.cvtColor(frontCameraROIDetectedMat, frontCameraROIDetectedGrayscaleMat, Imgproc.COLOR_RGB2GRAY);

                            MatOfKeyPoint frontCameraGrayscaleKeypoints = new MatOfKeyPoint();
                            detector.detect(frontCameraGrayscaleMat, frontCameraGrayscaleKeypoints);

                            Mat frontCameraGrayscaleDescriptorMat = new Mat();
                            extractor.compute(frontCameraGrayscaleMat, frontCameraGrayscaleKeypoints, frontCameraGrayscaleDescriptorMat);

                            MatOfDMatch ROImatches = new MatOfDMatch();
                            matcher.match(ROIDescriptorMat, frontCameraGrayscaleDescriptorMat, ROImatches);
                            List<DMatch> matchesList = ROImatches.toList();

                            Double max_dist = 0.0;
                            Double min_dist = 100.0;
                            for (int i = 0; i < ROIDescriptorMat.rows(); i++)
                            {
                                Double dist = (double) matchesList.get(i).distance;
                                if (dist < min_dist)
                                    min_dist = dist;
                                if (dist > max_dist)
                                    max_dist = dist;
                            }
                            System.out.println("-- Max dist : " + max_dist);
                            System.out.println("-- Min dist : " + min_dist);

                            LinkedList<DMatch> good_matches = new LinkedList<DMatch>();
                            MatOfDMatch gm = new MatOfDMatch();

                            for (int i = 0; i < ROIDescriptorMat.rows(); i++)
                            {
                                if (matchesList.get(i).distance < 3 * min_dist)
                                {
                                    good_matches.addLast(matchesList.get(i));
                                }
                            }

                            gm.fromList(good_matches);

                            Features2d.drawMatches(frontCameraROIDetectedGrayscaleMat, ROIDetectorKeypoints, frontCameraGrayscaleMat, frontCameraGrayscaleKeypoints, gm, frontCameraFrameMat, new Scalar(255, 0, 0), new Scalar(0, 0, 255), new MatOfByte(), 2);

                            GB08M2.getInstance().setFrontCameraROIDetectedGrayscaleMat(frontCameraROIDetectedGrayscaleMat);

                            Point ROIMatchLoc = new Point(matchLoc.x, matchLoc.y);
                            GB08M2.getInstance().setROIMatchLoc(ROIMatchLoc);
                            GB08M2.getInstance().setFrontCameraMat(frontCameraFrameMat);
                        }
                    }
                }

                Thread.sleep(500);

            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
    }
}
