package utilites;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.StringWriter;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.net.URLConnection;

import javafx.embed.swing.SwingFXUtils;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

import javax.imageio.ImageIO;

/**
 * Given an ImageView and URL read and create BufferedImages to be displayed
 * from a MJPEG stream
 * 
 * @author shrub34 Copyright 2012 Free for reuse, just please give me a credit
 *         if it is for a redistributed package
 */
public class MjpegRunner implements Runnable
{
    private static final String CONTENT_LENGTH = "Content-Length: ";
    private static final String CONTENT_TYPE = "Content-type: image/jpeg";
    private final ImageView viewer;
    private final InputStream urlStream;
    private StringWriter stringWriter;
    private boolean processing = true;

    public MjpegRunner(ImageView viewer, URL url) throws IOException
    {
        this.viewer = viewer;
        URLConnection urlConn = url.openConnection();
        // change the timeout to taste, I like 1 second
        urlConn.setReadTimeout(1000);
        urlConn.connect();
        urlStream = urlConn.getInputStream();
        stringWriter = new StringWriter(128);
    }

    /**
     * Stop the loop, and allow it to clean up
     */
    public synchronized void stop()
    {
        processing = false;
    }

    /**
     * Keeps running while process() returns true
     * 
     * Each loop asks for the next JPEG image and then sends it to our JPanel to
     * draw
     * 
     * @see java.lang.Runnable#run()
     */
    @Override
    public void run()
    {
        while (processing)
        {
            try
            {
                byte[] imageBytes = retrieveNextImage();
                ByteArrayInputStream bais = new ByteArrayInputStream(imageBytes);

                BufferedImage bufferedImage = ImageIO.read(bais);
                Image image = SwingFXUtils.toFXImage(bufferedImage, null);
                viewer.setImage(image);

            } catch (SocketTimeoutException ste)
            {
                System.err.println("failed stream read: " + ste);
                stop();
            } catch (IOException e)
            {
                System.err.println("failed stream read: " + e);
                stop();
            }
        }

        // close streams
        try
        {
            urlStream.close();
        } catch (IOException ioe)
        {
            System.err.println("Failed to close the stream: " + ioe);
        }
    }

    /**
     * Using the <i>urlStream</i> get the next JPEG image as a byte[]
     * 
     * @return byte[] of the JPEG
     * @throws IOException
     */
    private byte[] retrieveNextImage() throws IOException
    {
        boolean haveHeader = false;
        int currByte = -1;

        String header = null;
        // build headers
        int i = 0;
        while (i < 100)
        {
            haveHeader = false;
            currByte = -1;

            header = null;
            while ((currByte = urlStream.read()) > -1 && !haveHeader)
            {
                stringWriter.write(currByte);

                String tempString = stringWriter.toString();
                int indexOf = tempString.indexOf(CONTENT_TYPE);
                if (indexOf > 0)
                {
                    haveHeader = true;
                    header = tempString;
                }
            }
            i++;
        }
        // 255 indicates the start of the jpeg image
        while ((urlStream.read()) != 255)
        {
            // just skip extras
        }

        // rest is the buffer
        int contentLength = contentLength(header);
        byte[] imageBytes = new byte[contentLength + 1];
        // since we ate the original 255 , shove it back in
        imageBytes[0] = (byte) 255;
        int offset = 1;
        int numRead = 0;
        while (offset < imageBytes.length && (numRead = urlStream.read(imageBytes, offset, imageBytes.length - offset)) >= 0)
        {
            offset += numRead;
        }

        stringWriter = new StringWriter(128);

        return imageBytes;
    }

    // dirty but it works content-length parsing
    private static int contentLength(String header)
    {
        int indexOfContentLength = header.indexOf(CONTENT_LENGTH);
        int valueStartPos = indexOfContentLength + CONTENT_LENGTH.length();
        int indexOfEOL = header.indexOf('\n', indexOfContentLength);

        String lengthValStr = header.substring(valueStartPos, indexOfEOL).trim();

        int retValue = Integer.parseInt(lengthValStr);

        return retValue;
    }
}
