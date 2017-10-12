package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;

/**
 * Created by maska on 10/12/17.
 */

public class ChargerStationClient {

    AppCompatActivity callerActivity;

    String savedHost;
    public static final int CHARGER_SERVER_PORT = 9090;
    // message to send to the server
    private String mServerMessage;
    // while this is true, the server will continue running
    private boolean mRun = false;
    // used to send messages
    private PrintWriter mBufferOut;
    // used to read messages from the server
    private BufferedReader mBufferIn;

    Thread workerThread;

    public static class SingletonHolder {
        public static final ChargerStationClient HOLDER_INSTANCE = new ChargerStationClient();
    }

    public static ChargerStationClient getInstance() {
        return SingletonHolder.HOLDER_INSTANCE;
    }

    ChargerStationClient()
    {
        SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        savedHost = settings.getString("ChargerHost", "109.87.34.156");
    }

    public void setCallerActivity(AppCompatActivity callerActivity)
    {
        this.callerActivity = callerActivity;
    }

    public void startClient()
    {
        if (workerThread == null)
        {
            workerThread = new Thread(new Worker());
            workerThread.start();
        }
        else
        {
            callerActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(callerActivity, "Charger Client already connected", Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    /**
     * Close the connection and release the members
     */
    public void stopClient()
    {

        if (workerThread == null)
        {
            callerActivity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    Toast.makeText(callerActivity, "Charger Client already disconnected", Toast.LENGTH_SHORT).show();
                }
            });
        }
        else
        {

            mRun = false;

            try {
                workerThread.join(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            if (mBufferOut != null) {
                mBufferOut.flush();
                mBufferOut.close();
            }

            mBufferIn = null;
            mBufferOut = null;
            mServerMessage = null;

            workerThread.interrupt();
            workerThread = null;
        }
    }

    /**
     * Sends the message entered by client to the server
     *
     * @param message text entered by client
     */
    public void sendMessage(String message)
    {
        if (mBufferOut != null && !mBufferOut.checkError())
        {
            mBufferOut.println(message);
            mBufferOut.flush();
        }
    }

    private class Worker implements Runnable
    {
        @Override
        public void run()
        {
            mRun = true;

            //here you must put your computer's IP address.
            try
            {
                InetAddress serverAddr = InetAddress.getByName(savedHost);

                Log.i("TCP Charger Client", "Connecting...");

                try
                {
                    //create a socket to make the connection with the server
                    Socket socket = new Socket(serverAddr, CHARGER_SERVER_PORT);

                    //sends the message to the server
                    mBufferOut = new PrintWriter(new BufferedWriter(new OutputStreamWriter(socket.getOutputStream())), true);

                    //receives the message which the server sends back
                    mBufferIn = new BufferedReader(new InputStreamReader(socket.getInputStream()));

                    Log.i("TCP Charger Client", "Connected");

                    sendMessage("CLIENT READY");

                    //in this while the client listens for the messages sent by the server
                    while (mRun)
                    {
                        mServerMessage = mBufferIn.readLine();

                        if (mServerMessage != null)
                        {
                            Log.i("TCP Charger Client", mServerMessage);
                        }

                        try {
                            Thread.sleep(25);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    socket.close();

                } catch (IOException e) {
                    Log.e("TCP Charger Client", "Error", e);
                }


            } catch (UnknownHostException e) {
                Log.e("TCP Charger Client", "Error", e);
            }
        }
    }
}
