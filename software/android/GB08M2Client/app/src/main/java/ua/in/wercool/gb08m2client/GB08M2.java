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
}
