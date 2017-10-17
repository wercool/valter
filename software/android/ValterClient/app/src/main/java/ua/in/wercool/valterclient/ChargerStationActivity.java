package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;

public class ChargerStationActivity extends AppCompatActivity {

    private MjpegView chargerCameraView;
    public ProgressBar progressBar;

    Button up;
    Button down;
    Button left;
    Button right;
    Button center;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_charger_station);

        progressBar = (ProgressBar) findViewById(R.id.progressBar);

        chargerCameraView = (MjpegView) findViewById(R.id.chargerCameraView);
        chargerCameraView.setAdjustHeight(true);
        chargerCameraView.setMode(MjpegView.MODE_BEST_FIT);
        chargerCameraView.setCallerActivity(this);
        chargerCameraView.setProgressBar(progressBar);

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ChargerHost", "109.87.34.156");

        chargerCameraView.setUrl("http://" + savedHost + ":9595/?action=stream");
//        chargerCameraView.setUrl("http://200.36.58.250/mjpg/video.mjpg?resolution=640x480");
        chargerCameraView.setRecycleBitmap(true);

        up = (Button) findViewById(R.id.up);
        up.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVTILTUP");
                        break;
                }
                return true;
            }
        });

        down = (Button) findViewById(R.id.down);
        down.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVTILTDOWN");
                        break;
                }
                return true;
            }
        });

        left = (Button) findViewById(R.id.left);
        left.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVYAWLEFT");
                        break;
                }
                return true;
            }
        });

        right = (Button) findViewById(R.id.right);
        right.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVYAWRIGHT");
                        break;
                }
                return true;
            }
        });

        center = (Button) findViewById(R.id.center);
        center.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("CAMCENTER");
                        break;
                }
                return true;
            }
        });

    }

    @Override
    protected void onResume() {
        chargerCameraView.startStream();
        super.onResume();
    }

    @Override
    protected void onPause() {
        chargerCameraView.stopStream();
        super.onPause();
    }

    @Override
    protected void onStop() {
        chargerCameraView.stopStream();
        super.onStop();
    }
}
