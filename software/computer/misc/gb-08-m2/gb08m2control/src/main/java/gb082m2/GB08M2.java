package gb082m2;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;

import javafx.scene.shape.Rectangle;

import org.opencv.core.Mat;
import org.opencv.core.Point;

import application.MainAppController;

public class GB08M2
{
    static final GB08M2 instance = new GB08M2();
    boolean isInitialized = false;

    GB08M2CommandManager gb08m2CommandManager;
    public GB08M2ManualControlManager gb08m2ManualControlManager;
    public GB08M2AutomatedManager gb08m2AutomatedManager;

    public FrontCameraFrameGrabberTask frontCameraFrameGrabberTask;
    public RearCameraFrameGrabberTask rearCameraFrameGrabberTask;

    public FrontCameraCVProcessingTask frontCameraCVProcessingTask;

    //Network connections
    static String hostname = "127.0.0.1";
    static int commandPort = 9001;
    static int frontCameraPort = 8080;
    static int rearCameraPort = 8081;

    //Hardware control
    //Motors
    public static final String frontLeftMotorDutyCommandPrefix = "FLD#";
    volatile int frontLeftMotorDuty = 1;
    volatile int frontLeftMotorCurrent = 0;
    public static final String frontLeftMotorCurrentCommandPrefix = "FLC";
    public static final String frontLeftMotorCurrentResultPrefix = "FLMC";

    public static final String frontRightMotorDutyCommandPrefix = "FRD#";
    volatile int frontRightMotorDuty = 1;
    volatile int frontRightMotorCurrent = 0;
    public static final String frontRightMotorCurrentCommandPrefix = "FRC";
    public static final String frontRightMotorCurrentResultPrefix = "FRMC";

    public static final String rearLeftMotorDutyCommandPrefix = "RLD#";
    volatile int rearLeftMotorDuty = 1;
    volatile int rearLeftMotorCurrent = 0;
    public static final String rearLeftMotorCurrentCommandPrefix = "RLC";
    public static final String rearLeftMotorCurrentResultPrefix = "RLMC";

    public static final String rearRightMotorDutyCommandPrefix = "RRD#";
    volatile int rearRightMotorDuty = 1;
    volatile int rearRightMotorCurrent = 0;
    public static final String rearRightMotorCurrentCommandPrefix = "RRC";
    public static final String rearRightMotorCurrentResultPrefix = "RRMC";

    public static final String leftMotorsDirectionForwardCommand = "LF";
    public static final String leftMotorsDirectionBackwardCommand = "LB";
    public static final String leftMotorsStopCommand = "LS";
    public static final String rightMotorsDirectionForwardCommand = "RF";
    public static final String rightMotorsDirectionBackwardCommand = "RB";
    public static final String rightMotorsStopCommand = "RS";

    volatile String leftMotorsDirection = "forward";
    volatile String rightMotorsDirection = "forward";

    //Hardware parameters
    //public static final int maxADCMotorCurrentValue = 403; //10A (0.13 V/A)
    public static final int maxADCMotorCurrentValue = 605; //15A (0.13 V/A)

    //Abstract parameters
    volatile int leftDuty = 1;
    volatile int rightDuty = 1;

    static final int accelerationStepDelay = 15;
    static final int decelerationStepDelay = 5;

    static final int currentReadingsStepDelay = 120;

    //Wheel encoders
    public static final String enableEncodersCommand = "ENEN";
    public static final String disableEncodersCommand = "DISEN";

    public static final String leftEncoderTicksCommand = "LEN";
    public static final String leftEncoderTicksResultPrefix = "LEN";
    public static final String leftEncoderTicksResetCommand = "LENRES";

    public static final String rightEncoderTicksCommand = "REN";
    public static final String rightEncoderTicksResultPrefix = "REN";
    public static final String rightEncoderTicksResetCommand = "RENRES";

    public static final int encoderReadingsDelay = 50;
    volatile int leftEncoderTicks = 0;
    volatile int rightEncoderTicks = 0;
    public boolean encodersEnabled = false;

    //Distance scanner
    public static final int distanceScannerCenterPosition = 1460;
    public static final String distanceScannerPositionCommand = "DSP#";
    public static final String distanceScannerDistanceCommand = "DSD";
    public static final String distanceScannerResultPrefix = "DSD";
    public static final String distanceScannerReleaseServoCommand = "DSRST";
    public static int distanceScannerPositioningDelay = 10;
    double distanceScannerPositionAngle = -1;
    int distanceScannerPosition = 1460;
    int distanceScannerDistance = 0;
    static int prevDistanceScannerPosition = 1460;
    public boolean distanceScannerDirection = true;

    //Battery
    public static final int batteryReadingsDelay = 10000;
    public static final String batteryVoltageCommand = "GBV";
    public static final String batteryVoltageResultPrefix = "BV";
    volatile int batteryVoltage = 0;

    //Lights
    public static final String lightsOnCommand = "LIGHTSON";
    public static final String lightsOffCommand = "LIGHTSOFF";
    public boolean lights = false;

    //Beep
    public static final String beepCommand = "ALARMBEEP#";
    int beepDuration = 0;

    //Cameras
    volatile BufferedImage frontCameraFrameBufferedImage;
    volatile BufferedImage rearCameraFrameBufferedImage;

    volatile Mat frontCameraMat;
    volatile Mat frontCameraROIMat;
    volatile Mat frontCameraROIGrayscaleMat;
    volatile Mat frontCameraROIGrayscaleWithSURFMat;
    volatile Mat frontCameraROIDetectedMat;
    volatile Mat frontCameraROIDetectedGrayscaleMat;
    volatile Mat frontCameraSURFMatchesMat;
    volatile Mat frontCameraDetectedROISURFMatchesWithSelectedTemplateMat;
    Rectangle frontCameraROIRectangle;
    Point ROIMatchLoc;

    public static int frontCameraTiltValue = 0;
    public static int frontCameraPanValue = 0;

    public GB08M2()
    {
        gb08m2ManualControlManager = new GB08M2ManualControlManager();
        gb08m2AutomatedManager = new GB08M2AutomatedManager();
    }

    public boolean initialize()
    {
        isInitialized = true;

        frontCameraFrameGrabberTask = new FrontCameraFrameGrabberTask();
        rearCameraFrameGrabberTask = new RearCameraFrameGrabberTask();
        frontCameraCVProcessingTask = new FrontCameraCVProcessingTask();

        gb08m2CommandManager = new GB08M2CommandManager();
        isInitialized &= gb08m2CommandManager.isConnected;

        return isInitialized;
    }

    public boolean deInitialize()
    {
        gb08m2ManualControlManager.deinitialize();
        gb08m2AutomatedManager.deinitialize();

        if (gb08m2CommandManager != null)
        {
            gb08m2CommandManager.disconnect();
        }

        if (frontCameraFrameGrabberTask != null)
        {
            frontCameraFrameGrabberTask.stop();
        }
        if (rearCameraFrameGrabberTask != null)
        {
            rearCameraFrameGrabberTask.stop();
        }
        if (frontCameraCVProcessingTask != null)
        {
            frontCameraCVProcessingTask.stop();
        }

        return isInitialized = false;
    }

    //Getters and Setters
    //General
    public static GB08M2 getInstance()
    {
        return instance;
    }

    public boolean isInitialized()
    {
        return isInitialized;
    }

    public String getHostname()
    {
        return hostname;
    }

    public static void setHostname(String hostname)
    {
        GB08M2.hostname = hostname;
    }

    public static int getCommandPort()
    {
        return commandPort;
    }

    public static void setCommandPort(int commandPort)
    {
        GB08M2.commandPort = commandPort;
    }

    //Cameras
    public int getFrontCameraPort()
    {
        return frontCameraPort;
    }

    public static void setFrontCameraPort(int frontCameraPort)
    {
        GB08M2.frontCameraPort = frontCameraPort;
    }

    public int getRearCameraPort()
    {
        return rearCameraPort;
    }

    public static void setRearCameraPort(int rearCameraPort)
    {
        GB08M2.rearCameraPort = rearCameraPort;
    }

    public synchronized Mat getFrontCameraMat()
    {
        return frontCameraMat;
    }

    public synchronized void setFrontCameraMat(Mat frontCameraMat)
    {
        this.frontCameraMat = frontCameraMat;
    }

    public synchronized Mat getFrontCameraROIMat()
    {
        return frontCameraROIMat;
    }

    public synchronized void setFrontCameraROIMat(Mat frontCameraROIMat)
    {
        this.frontCameraROIMat = frontCameraROIMat;
    }

    public synchronized Rectangle getFrontCameraROIRectangle()
    {
        return frontCameraROIRectangle;
    }

    public synchronized void setFrontCameraROIRectangle(Rectangle frontCameraROIRectangle)
    {
        this.frontCameraROIRectangle = frontCameraROIRectangle;
    }

    public synchronized Point getROIMatchLoc()
    {
        return ROIMatchLoc;
    }

    public synchronized void setROIMatchLoc(Point rOIMatchLoc)
    {
        ROIMatchLoc = rOIMatchLoc;
    }

    public Mat getFrontCameraROIGrayscaleMat()
    {
        return frontCameraROIGrayscaleMat;
    }

    public void setFrontCameraROIGrayscaleMat(Mat frontCameraROIGrayscaleMat)
    {
        this.frontCameraROIGrayscaleMat = frontCameraROIGrayscaleMat;
    }

    public Mat getFrontCameraROIDetectedMat()
    {
        return frontCameraROIDetectedMat;
    }

    public void setFrontCameraROIDetectedMat(Mat frontCameraROIDetectedMat)
    {
        this.frontCameraROIDetectedMat = frontCameraROIDetectedMat;
    }

    public Mat getFrontCameraROIDetectedGrayscaleMat()
    {
        return frontCameraROIDetectedGrayscaleMat;
    }

    public void setFrontCameraROIDetectedGrayscaleMat(Mat frontCameraROIDetectedGrayscaleMat)
    {
        this.frontCameraROIDetectedGrayscaleMat = frontCameraROIDetectedGrayscaleMat;
    }

    public synchronized Mat getFrontCameraSURFMatchesMat()
    {
        return frontCameraSURFMatchesMat;
    }

    public synchronized void setFrontCameraSURFMatchesMat(Mat frontCameraSURFMatchesMat)
    {
        this.frontCameraSURFMatchesMat = frontCameraSURFMatchesMat;
    }

    public synchronized Mat getFrontCameraROIGrayscaleWithSURFMat()
    {
        return frontCameraROIGrayscaleWithSURFMat;
    }

    public synchronized void setFrontCameraROIGrayscaleWithSURFMat(Mat frontCameraROIGrayscaleWithSURFMat)
    {
        this.frontCameraROIGrayscaleWithSURFMat = frontCameraROIGrayscaleWithSURFMat;
    }

    public synchronized Mat getFrontCameraDetectedROISURFMatchesWithSelectedTemplateMat()
    {
        return frontCameraDetectedROISURFMatchesWithSelectedTemplateMat;
    }

    public synchronized void setFrontCameraDetectedROISURFMatchesWithSelectedTemplateMat(Mat frontCameraDetectedROISURFMatchesWithSelectedTemplateMat)
    {
        this.frontCameraDetectedROISURFMatchesWithSelectedTemplateMat = frontCameraDetectedROISURFMatchesWithSelectedTemplateMat;
    }

    public static void setFronCameraTilt(int tilt)
    {
        final String USER_AGENT = "Mozilla/5.0";
        //String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command&command=tilt_set&value=" + String.valueOf(tilt);
        ///?action=command_ng&dest=0&plugin=0&id=10094853&group=1&value=-200
        String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command_ng&dest=0&plugin=0&id=10094853&group=1&value=";

        if (frontCameraTiltValue != tilt)
        {
            if (tilt > frontCameraTiltValue)
            {
                url += "-100";
            } else
            {
                url += "100";
            }

            frontCameraTiltValue = tilt;

            URL obj;
            try
            {
                obj = new URL(url);
                HttpURLConnection con;
                try
                {
                    con = (HttpURLConnection) obj.openConnection();

                    // optional default is GET
                    con.setRequestMethod("GET");

                    //add request header
                    con.setRequestProperty("User-Agent", USER_AGENT);

                    int responseCode = con.getResponseCode();
                    System.out.println("\nSending 'GET' request to URL : " + url);
                    System.out.println("Response Code : " + responseCode);

                    BufferedReader in = new BufferedReader(new InputStreamReader(con.getInputStream()));
                    String inputLine;
                    StringBuffer response = new StringBuffer();

                    while ((inputLine = in.readLine()) != null)
                    {
                        response.append(inputLine);
                    }
                    in.close();

                    //print result
                    System.out.println(response.toString());
                } catch (IOException e)
                {
                    // TODO Auto-generated catch block
                    //e.printStackTrace();
                    System.out.println("Network error");
                }
            } catch (MalformedURLException e)
            {
                // TODO Auto-generated catch block
                //e.printStackTrace();
            }
        }
    }

    public static void setFronCameraPan(int pan)
    {
        if (frontCameraPanValue != pan)
        {
            final String USER_AGENT = "Mozilla/5.0";

            //String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command&command=pan_set&value=" + String.valueOf(pan);
            ///?action=command_ng&dest=0&plugin=0&id=10094852&group=1&value=200

            String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command_ng&dest=0&plugin=0&id=10094852&group=1&value=";

            if (frontCameraPanValue > pan)
            {
                url += "100";
            } else
            {
                url += "-100";
            }

            frontCameraPanValue = pan;
            URL obj;
            try
            {
                obj = new URL(url);
                HttpURLConnection con;
                try
                {
                    con = (HttpURLConnection) obj.openConnection();

                    // optional default is GET
                    con.setRequestMethod("GET");

                    //add request header
                    con.setRequestProperty("User-Agent", USER_AGENT);

                    int responseCode = con.getResponseCode();
                    System.out.println("\nSending 'GET' request to URL : " + url);
                    System.out.println("Response Code : " + responseCode);

                    BufferedReader in = new BufferedReader(new InputStreamReader(con.getInputStream()));
                    String inputLine;
                    StringBuffer response = new StringBuffer();

                    while ((inputLine = in.readLine()) != null)
                    {
                        response.append(inputLine);
                    }
                    in.close();

                    //print result
                    System.out.println(response.toString());
                } catch (IOException e)
                {
                    // TODO Auto-generated catch block
                    //e.printStackTrace();
                    System.out.println("Network error");
                }
            } catch (MalformedURLException e)
            {
                // TODO Auto-generated catch block
                //e.printStackTrace();
            }

        }
    }

    public static void resetFrontCamTilt()
    {
        final String USER_AGENT = "Mozilla/5.0";

        //String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command&command=reset_pan_tilt";
        ///?action=command_ng&dest=0&plugin=0&id=10094855&group=1&value=1
        String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command_ng&dest=0&plugin=0&id=10094855&group=1&value=1";
        System.out.println(url);
        URL obj;
        try
        {
            obj = new URL(url);
            HttpURLConnection con;
            try
            {
                con = (HttpURLConnection) obj.openConnection();

                // optional default is GET
                con.setRequestMethod("GET");

                con.disconnect();

                //add request header
                con.setRequestProperty("User-Agent", USER_AGENT);

                int responseCode = con.getResponseCode();
                System.out.println("\nSending 'GET' request to URL : " + url);
                System.out.println("Response Code : " + responseCode);

                BufferedReader in = new BufferedReader(new InputStreamReader(con.getInputStream()));
                String inputLine;
                StringBuffer response = new StringBuffer();

                while ((inputLine = in.readLine()) != null)
                {
                    response.append(inputLine);
                }
                in.close();

                //print result
                System.out.println(response.toString());
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                //e.printStackTrace();
                System.out.println("Network error");
            }
        } catch (MalformedURLException e)
        {
            // TODO Auto-generated catch block
            //e.printStackTrace();
        }
        MainAppController.cameraPanSlider.setValue(0);
        MainAppController.cameraTiltSlider.setValue(0);
    }

    public static void resetFrontCamPan()
    {
        final String USER_AGENT = "Mozilla/5.0";

        //String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command&command=reset_pan_tilt";
        ///?action=command_ng&dest=0&plugin=0&id=10094854&group=1&value=1
        String url = "http://" + hostname + ":" + String.valueOf(frontCameraPort) + "/?action=command_ng&dest=0&plugin=0&id=10094854&group=1&value=1";
        System.out.println(url);
        URL obj;
        try
        {
            obj = new URL(url);
            HttpURLConnection con;
            try
            {
                con = (HttpURLConnection) obj.openConnection();

                // optional default is GET
                con.setRequestMethod("GET");

                con.disconnect();

                //add request header
                con.setRequestProperty("User-Agent", USER_AGENT);

                int responseCode = con.getResponseCode();
                System.out.println("\nSending 'GET' request to URL : " + url);
                System.out.println("Response Code : " + responseCode);

                BufferedReader in = new BufferedReader(new InputStreamReader(con.getInputStream()));
                String inputLine;
                StringBuffer response = new StringBuffer();

                while ((inputLine = in.readLine()) != null)
                {
                    response.append(inputLine);
                }
                in.close();

                //print result
                System.out.println(response.toString());
            } catch (IOException e)
            {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        } catch (MalformedURLException e)
        {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        MainAppController.cameraPanSlider.setValue(0);
        MainAppController.cameraTiltSlider.setValue(0);
    }

    //Hardware

    //Cameras
    public void activateFrontCamera()
    {
        gb08m2CommandManager.sendCommand("SHELL:/home/maska/startgb08m2ivideo1");
    }

    public void deActivateFrontCamera()
    {
        gb08m2CommandManager.sendCommand("SHELL:kill -9 $(pidof front_camera_mjpg_streamer)");
    }

    public void activateRearCamera()
    {
        gb08m2CommandManager.sendCommand("SHELL:/home/maska/startgb08m2ivideo0");
    }

    public void deActivateRearCamera()
    {
        gb08m2CommandManager.sendCommand("SHELL:kill -9 $(pidof rear_camera_mjpg_streamer)");
    }

    //Motors
    public synchronized int getFrontLeftMotorDuty()
    {
        return frontLeftMotorDuty;
    }

    public String getLeftMotorsDirection()
    {
        return leftMotorsDirection;
    }

    public void setLeftMotorsDirection(String leftMotorsDirection)
    {
        System.out.println("leftMotorsDirection:" + leftMotorsDirection);
        if (leftMotorsDirection.startsWith("forward"))
        {
            gb08m2CommandManager.sendCommand(leftMotorsDirectionForwardCommand);
        }
        if (leftMotorsDirection.startsWith("backward"))
        {
            gb08m2CommandManager.sendCommand(leftMotorsDirectionBackwardCommand);
        }
        this.leftMotorsDirection = leftMotorsDirection;
    }

    public void stopLeftMotors()
    {
        gb08m2CommandManager.sendCommand(leftMotorsStopCommand);
        setLeftMotorsDirection("stopped");
    }

    public String getRightMotorsDirection()
    {
        return rightMotorsDirection;
    }

    public void setRightMotorsDirection(String rightMotorsDirection)
    {
        if (rightMotorsDirection.startsWith("forward"))
        {
            gb08m2CommandManager.sendCommand(rightMotorsDirectionForwardCommand);
        }
        if (rightMotorsDirection.startsWith("backward"))
        {
            gb08m2CommandManager.sendCommand(rightMotorsDirectionBackwardCommand);
        }
        this.rightMotorsDirection = rightMotorsDirection;
    }

    public void stopRightMotors()
    {
        gb08m2CommandManager.sendCommand(rightMotorsStopCommand);
        setRightMotorsDirection("stopped");
    }

    public synchronized void setFrontLeftMotorDuty(int frontLeftMotorDuty)
    {
        if (this.frontLeftMotorDuty != frontLeftMotorDuty)
        {
            gb08m2CommandManager.sendCommand(frontLeftMotorDutyCommandPrefix + String.valueOf(frontLeftMotorDuty));
        }
        this.frontLeftMotorDuty = frontLeftMotorDuty;
    }

    public synchronized int getRearLeftMotorDuty()
    {
        return rearLeftMotorDuty;
    }

    public synchronized void setRearLeftMotorDuty(int rearLeftMotorDuty)
    {
        if (this.rearLeftMotorDuty != rearLeftMotorDuty)
        {
            gb08m2CommandManager.sendCommand(rearLeftMotorDutyCommandPrefix + String.valueOf(rearLeftMotorDuty));
        }
        this.rearLeftMotorDuty = rearLeftMotorDuty;
    }

    synchronized public int getFrontRightMotorDuty()
    {
        return frontRightMotorDuty;
    }

    public void setFrontRightMotorDuty(int frontRightMotorDuty)
    {
        if (this.frontRightMotorDuty != frontRightMotorDuty)
        {
            gb08m2CommandManager.sendCommand(frontRightMotorDutyCommandPrefix + String.valueOf(frontRightMotorDuty));
        }
        this.frontRightMotorDuty = frontRightMotorDuty;
    }

    synchronized public int getRearRightMotorDuty()
    {
        return rearRightMotorDuty;
    }

    public void setRearRightMotorDuty(int rearRightMotorDuty)
    {
        if (this.rearRightMotorDuty != rearRightMotorDuty)
        {
            gb08m2CommandManager.sendCommand(rearRightMotorDutyCommandPrefix + String.valueOf(rearRightMotorDuty));
        }
        this.rearRightMotorDuty = rearRightMotorDuty;
    }

    public synchronized void retrieveFrontLeftMotorCurrent()
    {
        gb08m2CommandManager.sendCommand(frontLeftMotorCurrentCommandPrefix);
    }

    public synchronized int getFrontLeftMotorCurrent()
    {
        return frontLeftMotorCurrent;
    }

    public synchronized void setFrontLeftMotorCurrent(int frontLeftMotorCurrent)
    {
        this.frontLeftMotorCurrent = frontLeftMotorCurrent;
    }

    public synchronized void retrieveFrontRightMotorCurrent()
    {
        gb08m2CommandManager.sendCommand(frontRightMotorCurrentCommandPrefix);
    }

    public synchronized int getFrontRightMotorCurrent()
    {
        return frontRightMotorCurrent;
    }

    public synchronized void setFrontRightMotorCurrent(int frontRightMotorCurrent)
    {
        this.frontRightMotorCurrent = frontRightMotorCurrent;
    }

    public synchronized void retrieveRearLeftMotorCurrent()
    {
        gb08m2CommandManager.sendCommand(rearLeftMotorCurrentCommandPrefix);
    }

    public synchronized int getRearLeftMotorCurrent()
    {
        return rearLeftMotorCurrent;
    }

    public synchronized void setRearLeftMotorCurrent(int rearLeftMotorCurrent)
    {
        this.rearLeftMotorCurrent = rearLeftMotorCurrent;
    }

    public synchronized void retrieveRearRightMotorCurrent()
    {
        gb08m2CommandManager.sendCommand(rearRightMotorCurrentCommandPrefix);
    }

    public synchronized int getRearRightMotorCurrent()
    {
        return rearRightMotorCurrent;
    }

    public synchronized void setRearRightMotorCurrent(int rearRightMotorCurrent)
    {
        this.rearRightMotorCurrent = rearRightMotorCurrent;
    }

    //Wheel encoders
    //100cm - 87 ticks
    public synchronized void enableEncoders()
    {
        gb08m2CommandManager.sendCommand(enableEncodersCommand);
        encodersEnabled = true;
    }

    public synchronized void disableEncoders()
    {
        gb08m2CommandManager.sendCommand(disableEncodersCommand);
        encodersEnabled = false;
    }

    //Left encoder
    public synchronized void retrieveLeftEncoderTicks()
    {
        gb08m2CommandManager.sendCommand(leftEncoderTicksCommand);
    }

    public synchronized void resetLeftEncoderTicks()
    {
        gb08m2CommandManager.sendCommand(leftEncoderTicksResetCommand);
    }

    public synchronized int getLeftEncoderTicks()
    {
        return leftEncoderTicks;
    }

    public synchronized void setLeftEncoderTicks(int leftEncoderTicks)
    {
        this.leftEncoderTicks = leftEncoderTicks;
    }

    //Right encoder
    public synchronized void retrieveRightEncoderTicks()
    {
        gb08m2CommandManager.sendCommand(rightEncoderTicksCommand);
    }

    public synchronized void resetRightEncoderTicks()
    {
        gb08m2CommandManager.sendCommand(rightEncoderTicksResetCommand);
    }

    public synchronized int getRightEncoderTicks()
    {
        return rightEncoderTicks;
    }

    public synchronized void setRightEncoderTicks(int rightEncoderTicks)
    {
        this.rightEncoderTicks = rightEncoderTicks;
    }

    //Distance scanner
    public synchronized int getDistanceScannerPosition()
    {
        return distanceScannerPosition;
    }

    public synchronized void setDistanceScannerPosition(int distanceScannerPosition, boolean updateAngleValue)
    {
        this.distanceScannerPosition = distanceScannerPosition;

        if (updateAngleValue)
        {
            //nearest 0.5
            double distanceScannerPositionAngle = Math.ceil(((2100 - distanceScannerPosition) / 9.85) * 2) / 2;
            setDistanceScannerPositionAngle(distanceScannerPositionAngle, false);
        }

        gb08m2CommandManager.sendCommand(distanceScannerPositionCommand + String.valueOf(this.distanceScannerPosition));
    }

    public synchronized double getDistanceScannerPositionAngle()
    {
        return distanceScannerPositionAngle;
    }

    public synchronized double getDistanceScannerPositionAngleRad()
    {
        return distanceScannerPositionAngle * Math.PI / 180;
    }

    public synchronized void setDistanceScannerPositionAngle(double distanceScannerPositionAngle, boolean execute)
    {
        this.distanceScannerPositionAngle = distanceScannerPositionAngle;
        int distanceScannerPosition = (int) Math.round(2100 - distanceScannerPositionAngle * 9.85);
        if (execute)
        {
            setDistanceScannerPosition(distanceScannerPosition, false);
        }
    }

    public void retrieveDistanceScannerDistance()
    {
        gb08m2CommandManager.sendCommand(distanceScannerDistanceCommand);
    }

    public synchronized int getDistanceScannerDistance()
    {
        return distanceScannerDistance;
    }

    public synchronized int getDistanceScannerDistance_cm()
    {
        double distanceScannerADCVoltage = distanceScannerDistance * 3.33 / 1023;
        //16.2537 * x^4 - 129.893 * x^3 + 382.268 * x^2 - 512.611 * x + 306.439
        int distanceScannerDistance_cm = (int) Math.round(16.2537 * Math.pow(distanceScannerADCVoltage, 4) - 129.893 * Math.pow(distanceScannerADCVoltage, 3) + 382.268 * Math.pow(distanceScannerADCVoltage, 2) - 512.611 * distanceScannerADCVoltage + 306.439);
        return distanceScannerDistance_cm;
    }

    public synchronized void setDistanceScannerDistance(int distanceScannerDistance)
    {
        this.distanceScannerDistance = distanceScannerDistance;
    }

    public synchronized void setDistanceScannerPositionSetDelay(int distanceScannerPositionSetDelay)
    {
        GB08M2.distanceScannerPositioningDelay = distanceScannerPositionSetDelay;
    }

    public void releaseDistanceScannerServo()
    {
        gb08m2CommandManager.sendCommand(distanceScannerReleaseServoCommand);
    }

    //Battery
    public void retrieveBatteryVoltage()
    {
        gb08m2CommandManager.sendCommand(batteryVoltageCommand);
    }

    public synchronized int getBatteryVoltage()
    {
        return batteryVoltage;
    }

    public synchronized void setBatteryVoltage(int batteryVoltage)
    {
        this.batteryVoltage = batteryVoltage;
    }

    public boolean isLights()
    {
        return lights;
    }

    public void setLights(boolean lights)
    {
        if (lights)
        {
            gb08m2CommandManager.sendCommand(lightsOnCommand);
        } else
        {
            gb08m2CommandManager.sendCommand(lightsOffCommand);
        }
        this.lights = lights;
    }

    //Beep
    public void setBeep(int beepDuration)
    {
        this.beepDuration = beepDuration;
        gb08m2CommandManager.sendCommand(beepCommand + String.valueOf(this.beepDuration));
    }

    public int getBeepDuration()
    {
        return this.beepDuration;
    }

    public synchronized BufferedImage getFrontCameraFrameBufferedImage()
    {
        return frontCameraFrameBufferedImage;
    }

    public synchronized void setFrontCameraFrameBufferedImage(BufferedImage frontCameraFrameBufferedImage)
    {
        this.frontCameraFrameBufferedImage = frontCameraFrameBufferedImage;
    }

    public synchronized BufferedImage getRearCameraFrameBufferedImage()
    {
        return rearCameraFrameBufferedImage;
    }

    public synchronized void setRearCameraFrameBufferedImage(BufferedImage rearCameraFrameBufferedImage)
    {
        this.rearCameraFrameBufferedImage = rearCameraFrameBufferedImage;
    }

    //Abstract
    public synchronized int getLeftDuty()
    {
        return leftDuty;
    }

    public synchronized void setLeftDuty(int leftDuty)
    {
        this.leftDuty = leftDuty;
    }

    public synchronized int getRightDuty()
    {
        return rightDuty;
    }

    public synchronized void setRightDuty(int rightDuty)
    {
        this.rightDuty = rightDuty;
    }

    public static class Orientation
    {
        public double posX;
        public double posY;
    }
}