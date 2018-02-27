package ua.in.wercool.valterclient;

import android.content.Context;
import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.widget.Toast;

import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Calendar;

/**
 * Created by maska on 10/10/17.
 */

public class ValterWebSocketClient {

    AppCompatActivity callerActivity;

    URI uri;
    WebSocketClient mWebSocketClient;

    WatchDogRunnable watchDogRunnable;
    Thread watchDogThread;

    int watchDogSleep = 1000;

    public static class SingletonHolder {
        public static final ValterWebSocketClient HOLDER_INSTANCE = new ValterWebSocketClient();
    }

    public static ValterWebSocketClient getInstance() {
        return SingletonHolder.HOLDER_INSTANCE;
    }

    ValterWebSocketClient()
    {
        SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        try {
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            uri = new URI("ws://" + savedHost + ":8888");
        } catch (URISyntaxException e) {
            e.printStackTrace();
            return;
        }
    }

    public void setCallerActivity(AppCompatActivity callerActivity)
    {
        this.callerActivity = callerActivity;
    }

    public void updateURI()
    {
        SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        try {
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            uri = new URI("ws://" + savedHost + ":8888");
        } catch (URISyntaxException e) {
            e.printStackTrace();
            return;
        }
    }

    public void sendMessage(String message)
    {
        Log.i("sendMessage: ", message);
        if (mWebSocketClient != null)
        {
            if (mWebSocketClient.isOpen())
            {
                mWebSocketClient.send(message);
            }
        }
    }

    public void connect()
    {
        mWebSocketClient = new WebSocketClient(uri) {
            @Override
            public void onOpen(ServerHandshake serverHandshake) {
                Log.i("ValterWebSocket", "Opened");
                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, "Valter WebSocket CONNECTED", Toast.LENGTH_LONG).show();
                    }
                });
                mWebSocketClient.send("SRV#CLIENT READY");

                if (watchDogThread == null) {
                    watchDogRunnable = new WatchDogRunnable(mWebSocketClient);
                    watchDogThread = new Thread(watchDogRunnable);
                    watchDogThread.start();
                } else {
                    watchDogRunnable.watchDogRun = false;
                    watchDogThread.interrupt();
                    watchDogRunnable = new WatchDogRunnable(mWebSocketClient);
                    watchDogThread = new Thread(watchDogRunnable);
                    watchDogThread.start();
                }
            }

            @Override
            public void onMessage(final String message)
            {
                Log.i("ValterWebSocket IN", message);
            }

            @Override
            public void onClose(int i, final String s, boolean b) {
                Log.i("ValterWebSocket onClose", "Closed " + s);

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, "Valter WebSocket Closed", Toast.LENGTH_LONG).show();
                    }
                });
            }

            @Override
            public void onError(final Exception e) {
                Log.i("ValterWebSocket onError", "Error " + e.getMessage());

                callerActivity.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Toast.makeText(callerActivity, e.getMessage(), Toast.LENGTH_LONG).show();
                    }
                });
            }
        };
        mWebSocketClient.connect();
    }

    public void disconnect()
    {
        if (mWebSocketClient != null)
        {
            mWebSocketClient.close();
        }
    }

    public int getWatchDogSleep() {
        return watchDogSleep;
    }

    public void setWatchDogSleep(int watchDogSleep) {
        this.watchDogSleep = watchDogSleep;
    }

    private class WatchDogRunnable implements Runnable {

        public boolean watchDogRun = true;

        private WebSocketClient mWebSocketClient;

        public WatchDogRunnable(WebSocketClient mWebSocketClient) {
            this.mWebSocketClient = mWebSocketClient;
        }

        @Override
        public void run() {
            while (watchDogRun) {
                try {

                    if (mWebSocketClient != null)
                    {
                        if (mWebSocketClient.isOpen())
                        {
                            mWebSocketClient.send(String.format("SRV#WDIN:%d", Calendar.getInstance().get(Calendar.SECOND)));
                            Log.i("WatchDogRunnable", String.format("SRV#WDIN:%d", Calendar.getInstance().get(Calendar.SECOND)));
                        }
                    }

                    Thread.sleep(watchDogSleep);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
    }

}
