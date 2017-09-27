package ua.in.wercool.gb08m2client;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Socket;

/**
 * Created by maska on 9/26/17.
 */

public class GB08M2 {

    private static GB08M2 instance;
    AppCompatActivity callerActivity;

    Socket cmdSocket;
    CommunicationThread commThread;
    String result;

    public static final String startFrontCamCommand = "SHELL:/home/maska/startgb08m2ivideo1";
    public static final String stopFrontCamCommand = "SHELL:kill -9 $(pidof front_camera_mjpg_streamer)";

    public static final String startRearCamCommand = "SHELL:/home/maska/startgb08m2ivideo0";
    public static final String stopRearCamCommand = "SHELL:kill -9 $(pidof rear_camera_mjpg_streamer)";

    public static final String lightsOnCommand = "LIGHTSON";
    public static final String lightsOffCommand = "LIGHTSOFF";

    public static final String batteryVoltageCommand = "GBV";
    public static final String batteryVoltageResultPrefix = "BV";

    public static final String frontLeftMotorDutyCommandPrefix = "FLD#";
    public static final String frontLeftMotorCurrentCommandPrefix = "FLC";
    public static final String frontLeftMotorCurrentResultPrefix = "FLMC";

    public static final String frontRightMotorDutyCommandPrefix = "FRD#";
    public static final String frontRightMotorCurrentCommandPrefix = "FRC";
    public static final String frontRightMotorCurrentResultPrefix = "FRMC";

    public static final String rearLeftMotorDutyCommandPrefix = "RLD#";
    public static final String rearLeftMotorCurrentCommandPrefix = "RLC";
    public static final String rearLeftMotorCurrentResultPrefix = "RLMC";

    public static final String rearRightMotorDutyCommandPrefix = "RRD#";
    public static final String rearRightMotorCurrentCommandPrefix = "RRC";
    public static final String rearRightMotorCurrentResultPrefix = "RRMC";

    public static final String leftMotorsDirectionForwardCommand = "LF";
    public static final String leftMotorsDirectionBackwardCommand = "LB";
    public static final String leftMotorsStopCommand = "LS";
    public static final String rightMotorsDirectionForwardCommand = "RF";
    public static final String rightMotorsDirectionBackwardCommand = "RB";
    public static final String rightMotorsStopCommand = "RS";

    private int leftMotorDuty = 15;
    private int rightMotorDuty = 15;

    private int curLeftMotorDuty = 1;
    private int curRightMotorDuty = 1;

    public static GB08M2 getInstance()
    {
        if (instance == null)
        {
            instance = new GB08M2();
        }
        return instance;
    }

    public void setCallerActivity(AppCompatActivity callerActivity)
    {
        this.callerActivity = callerActivity;
    }

    public void connect()
    {
        if (cmdSocket == null)
        {
            SharedPreferences settings = callerActivity.getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("GB08M2Host", "109.87.34.156");

            try {
                cmdSocket = new Socket();
                cmdSocket.connect(new InetSocketAddress(savedHost, 9001), 5000);
                Log.i("GB08M2", "CONNECTING...");

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, "CONNECTED", Toast.LENGTH_LONG).show();
                    }
                });

                commThread = new CommunicationThread();
                new Thread(commThread).start();

                Log.i("GB08M2", "CONNECTED");
            }
            catch (final IOException e)
            {
                e.printStackTrace();

                try {
                    cmdSocket.close();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
                cmdSocket = null;

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, e.getMessage(), Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }
        else
        {
            if (!cmdSocket.isConnected())
            {
                try {
                    cmdSocket.close();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
                cmdSocket = null;
                connect();
            }
            else
            {
                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, "Already connected", Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }
    }

    public void disconnect()
    {
        if (commThread!= null)
        {
            commThread.stopped = true;
        }
        if (cmdSocket != null)
        {
            try {
                cmdSocket.close();
            } catch (IOException e1) {
                e1.printStackTrace();
            }
            cmdSocket = null;

            callerActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(callerActivity, "DISCONNECTED", Toast.LENGTH_LONG).show();
                }
            });
        }
        else
        {
            callerActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(callerActivity, "Already disconnected", Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    public void sendCMD(String cmd)
    {
        Log.i("GB08M2 CMD", cmd);
        if (cmdSocket != null) {
            try {
                PrintWriter out = new PrintWriter(new BufferedWriter(
                        new OutputStreamWriter(cmdSocket.getOutputStream())),
                        true);
                out.println(cmd);

            } catch (final IOException e) {
                e.printStackTrace();

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, e.getMessage(), Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }
        else
        {
            callerActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(callerActivity, "NO CONNECTION", Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    class CommunicationThread implements Runnable {

        public boolean stopped = false;
        private BufferedReader input;

        public CommunicationThread() {
            try {

                this.input = new BufferedReader(new InputStreamReader(cmdSocket.getInputStream()));

            } catch (final IOException e) {
                e.printStackTrace();

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, e.getMessage(), Toast.LENGTH_SHORT).show();
                    }
                });
            }
        }

        public void run()
        {
            while (!Thread.currentThread().isInterrupted() && !stopped)
            {
                try {

                    String response = input.readLine();
                    Log.i("GB08M2 Socket > ", response);

                    if (callerActivity.getClass().equals(CommandsActivity.class)) {
                        if (response.indexOf(batteryVoltageResultPrefix) > -1) {
                            result = response.substring(batteryVoltageResultPrefix.length() + 1);

                            callerActivity.runOnUiThread(new Runnable() {
                                @Override
                                public void run() {
                                    ((TextView) (callerActivity.findViewById(R.id.batVoltageTextView))).setText(result);
                                }
                            });
                        }
                    }

                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            if (stopped)
            {
                Thread.currentThread().interrupt();
            }
        }
    }

    public int getLeftMotorDuty()
    {
        return this.leftMotorDuty;
    }
    public void setLeftMotorDuty(int leftMotorDuty)
    {
        if (leftMotorDuty >= 1 && leftMotorDuty <= 100)
        {
            this.leftMotorDuty = leftMotorDuty;
        }
        else
        {
            this.leftMotorDuty = (leftMotorDuty < 1) ? 1 : 100;
        }
        Log.i("GB08M2", String.format("leftMorotDuty=%d", this.leftMotorDuty));
    }
    public int getRightMotorDuty()
    {
        return this.rightMotorDuty;
    }
    public void setRightMotorDuty(int rightMotorDuty)
    {
        if (rightMotorDuty >= 1 && rightMotorDuty <= 100)
        {
            this.rightMotorDuty = rightMotorDuty;
        }
        else
        {
            this.rightMotorDuty = (rightMotorDuty < 1) ? 1 : 100;
        }
        Log.i("GB08M2", String.format("rightMotorDuty=%d", this.rightMotorDuty));
    }

    public int getCurLeftMotorDuty()
    {
        return this.curLeftMotorDuty;
    }
    public void setCurtLeftMotorDuty(int curLeftMotorDuty)
    {
        if (curLeftMotorDuty >= 1 && curLeftMotorDuty <= getLeftMotorDuty())
        {
            this.curLeftMotorDuty = curLeftMotorDuty;
        }
        else
        {
            this.curLeftMotorDuty = (curLeftMotorDuty < 1) ? 1 : getLeftMotorDuty();
        }
        sendCMD(String.format("%s%d", frontLeftMotorDutyCommandPrefix, getCurLeftMotorDuty()));
        sendCMD(String.format("%s%d", rearLeftMotorDutyCommandPrefix, getCurLeftMotorDuty()));
        Log.i("GB08M2", String.format("curLeftMorotDuty=%d", this.curLeftMotorDuty));
    }

    public int getCurRightMotorDuty()
    {
        return this.curRightMotorDuty;
    }
    public void setCurRightMotorDuty(int curRightMotorDuty)
    {
        if (curRightMotorDuty >= 1 && curRightMotorDuty <= getRightMotorDuty())
        {
            this.curRightMotorDuty = curRightMotorDuty;
        }
        else
        {
            this.curRightMotorDuty = (curRightMotorDuty < 1) ? 1 : getRightMotorDuty();
        }
        sendCMD(String.format("%s%d", frontRightMotorDutyCommandPrefix, getCurRightMotorDuty()));
        sendCMD(String.format("%s%d", rearRightMotorDutyCommandPrefix, getCurRightMotorDuty()));
        Log.i("GB08M2", String.format("curRightMotorDuty=%d", this.curRightMotorDuty));
    }
}
