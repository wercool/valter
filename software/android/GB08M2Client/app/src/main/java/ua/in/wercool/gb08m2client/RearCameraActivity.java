package ua.in.wercool.gb08m2client;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.ProgressBar;

public class RearCameraActivity extends AppCompatActivity {

    private MjpegView rearCameraView;
    public ProgressBar progressBar;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_rear_camera);
        progressBar = (ProgressBar) findViewById(R.id.progressBar);

        rearCameraView = (MjpegView) findViewById(R.id.rearCameraView);
        rearCameraView.setAdjustHeight(true);
        rearCameraView.setMode(MjpegView.MODE_BEST_FIT);
        rearCameraView.setCallerActivity(this);
        rearCameraView.setProgressBar(progressBar);

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("GB08M2Host", "109.87.34.156");

        rearCameraView.setUrl("http://" + savedHost + ":8081/?action=stream");
//        fronCameraView.setUrl("http://200.36.58.250/mjpg/video.mjpg?resolution=640x480");
        rearCameraView.setRecycleBitmap(true);
    }

    @Override
    protected void onResume() {
        rearCameraView.startStream();
        super.onResume();
    }

    @Override
    protected void onPause() {
        rearCameraView.stopStream();
        super.onPause();
    }

    @Override
    protected void onStop() {
        rearCameraView.stopStream();
        super.onStop();
    }
}
