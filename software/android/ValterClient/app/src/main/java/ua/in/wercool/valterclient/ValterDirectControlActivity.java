package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.ProgressBar;

public class ValterDirectControlActivity extends AppCompatActivity {

    private MjpegView currentCameraView;
    public ProgressBar progressBar;

    ValterManualNavigation valterManualNavigationFragment;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_valter_direct_control);

        progressBar = (ProgressBar) findViewById(R.id.progressBar);

        currentCameraView = (MjpegView) findViewById(R.id.currentCameraView);
        currentCameraView.setAdjustHeight(true);
        currentCameraView.setMode(MjpegView.MODE_BEST_FIT);
        currentCameraView.setCallerActivity(this);
        currentCameraView.setProgressBar(progressBar);

        valterManualNavigationFragment = new ValterManualNavigation();
    }

    @Override
    protected void onResume() {
        currentCameraView.startStream();
        super.onResume();
    }

    @Override
    protected void onPause() {
        currentCameraView.stopStream();
        super.onPause();
    }

    @Override
    protected void onStop() {
        currentCameraView.stopStream();
        super.onStop();
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.select_current_camera_menu, menu);
        return true;
    }

    public boolean onFrontalCameraItemClick(MenuItem item)
    {
        currentCameraView.stopStream();

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ValterHost", "109.87.34.156");

        System.out.println("FRONTAL CAMERA");

        if (savedHost.indexOf("192.168") > -1) {
            System.out.println("FRONTAL CAMERA STREAM IS ON INTRANET");
            savedHost = "192.168.101.101";
        }

        currentCameraView.setUrl("http://" + savedHost + ":10100/?action=stream");
        currentCameraView.setRecycleBitmap(true);
        currentCameraView.startStream();

        return true;
    }

    public boolean onRearCameraItemClick(MenuItem item)
    {
        currentCameraView.stopStream();

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ValterHost", "109.87.34.156");

        System.out.println("REAR CAMERA");

        if (savedHost.indexOf("192.168") > -1) {
            System.out.println("REAR CAMERA STREAM IS ON INTRANET");
            savedHost = "192.168.101.101";
        }
        currentCameraView.setUrl("http://" + savedHost + ":10101/?action=stream");
        currentCameraView.setRecycleBitmap(true);
        currentCameraView.startStream();

        return true;
    }

    public boolean onHeadLCameraItemClick(MenuItem item)
    {
        currentCameraView.stopStream();

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ValterHost", "109.87.34.156");

        System.out.println("HEAD LEFT CAMERA");

        if (savedHost.indexOf("192.168") > -1) {
            System.out.println("FRONTAL CAMERA STREAM IS ON INTRANET");
            savedHost = "192.168.101.102";
        }
        currentCameraView.setUrl("http://" + savedHost + ":10200/?action=stream");
        currentCameraView.setRecycleBitmap(true);
        currentCameraView.startStream();

        return true;
    }

    public boolean onHeadRCameraItemClick(MenuItem item)
    {
        currentCameraView.stopStream();

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ValterHost", "109.87.34.156");

        System.out.println("HEAD RIGHT CAMERA");

        if (savedHost.indexOf("192.168") > -1) {
            System.out.println("FRONTAL CAMERA STREAM IS ON INTRANET");
            savedHost = "192.168.101.102";
        }
        currentCameraView.setUrl("http://" + savedHost + ":10201/?action=stream");
        currentCameraView.setRecycleBitmap(true);
        currentCameraView.startStream();

        return true;
    }
}
