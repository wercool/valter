package ua.in.wercool.valterclient;

import android.app.Dialog;
import android.content.Intent;
import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.widget.ProgressBar;

public class ValterDirectControlActivity extends AppCompatActivity {

    private MjpegView currentCameraView;
    public ProgressBar progressBar;

    ValterManualNavigation valterManualNavigationFragment;
    HeadYawPitchControlFragment headYawPitchControlFragment;

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
        getSupportFragmentManager().beginTransaction().add(R.id.fragmentContainer, valterManualNavigationFragment).commit();
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
        inflater.inflate(R.menu.menu_valter_direct_controls, menu);
        menu.getItem(4).setIcon(R.drawable.joystick);
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

    public boolean valterManualSteeringItemClick(MenuItem item)
    {
        valterManualNavigationFragment = new ValterManualNavigation();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, valterManualNavigationFragment).commit();
        return true;
    }

    public boolean valterHeadYawPitchItemClick(MenuItem item)
    {
        headYawPitchControlFragment = new HeadYawPitchControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, headYawPitchControlFragment).commit();
        return true;
    }


    public boolean valterCommandsItemClick(MenuItem item)
    {
        ValterCommandsActivity.dialogMode = true;
        Intent myIntent = new Intent(getApplicationContext(), ValterCommandsActivity.class);
        startActivityForResult(myIntent, 0);
        return true;
    }

}
