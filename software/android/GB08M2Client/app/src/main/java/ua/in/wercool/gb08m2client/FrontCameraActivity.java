package ua.in.wercool.gb08m2client;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ProgressBar;

public class FrontCameraActivity extends AppCompatActivity {

    private MjpegView fronCameraView;
    public ProgressBar progressBar;

    Button leftForward;
    Button forward;
    Button rightForward;
    Button left;
    Button stop;
    Button right;
    Button leftBackward;
    Button backward;
    Button rightBackward;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_front_camera);

        progressBar = (ProgressBar) findViewById(R.id.progressBar);

        fronCameraView = (MjpegView) findViewById(R.id.frontCameraView);
        fronCameraView.setAdjustHeight(true);
        fronCameraView.setMode(MjpegView.MODE_BEST_FIT);
        fronCameraView.setCallerActivity(this);
        fronCameraView.setProgressBar(progressBar);

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("GB08M2Host", "109.87.34.156");

        fronCameraView.setUrl("http://" + savedHost + ":8080/?action=stream");
//        fronCameraView.setUrl("http://200.36.58.250/mjpg/video.mjpg?resolution=640x480");
        fronCameraView.setRecycleBitmap(true);


        leftForward = (Button) findViewById(R.id.leftForward);
        leftForward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                if (event.getAction() == MotionEvent.ACTION_DOWN)
                {
                    Log.i("GB08M2", "LEFT FWD - ACCELERATE");
                }
                else if (event.getAction() == MotionEvent.ACTION_UP)
                {
                    Log.i("GB08M2", "LEFT FWD - DECELERATE");
                }
                return true;
            }
        });

        forward = (Button) findViewById(R.id.forward);
        rightForward = (Button) findViewById(R.id.rightForward);
        left = (Button) findViewById(R.id.left);
        stop = (Button) findViewById(R.id.stop);
        right = (Button) findViewById(R.id.right);
        leftBackward = (Button) findViewById(R.id.leftBackward);
        backward = (Button) findViewById(R.id.backward);
        rightBackward = (Button) findViewById(R.id.rightBackward);
    }

    @Override
    protected void onResume() {
        fronCameraView.startStream();
        super.onResume();
    }

    @Override
    protected void onPause() {
        fronCameraView.stopStream();
        super.onPause();
    }

    @Override
    protected void onStop() {
        fronCameraView.stopStream();
        super.onStop();
    }
}
