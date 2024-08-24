package ua.in.wercool.valterclient;

import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.view.menu.MenuBuilder;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.View;
import android.view.inputmethod.BaseInputConnection;
import android.widget.ImageButton;
import android.widget.ProgressBar;

public class ValterDirectControlActivity extends AppCompatActivity {

    private MjpegView currentCameraView;
    public ProgressBar progressBar;
    ImageButton optionsMenuButton;

    ValterManualNavigation valterManualNavigationFragment;
    HeadYawPitchControlFragment headYawPitchControlFragment;
    PlatformManipulatorAndIRBumperControlFragment platformManipulatorAndIRBumperControlFragment;
    ArmControlRightControlFragment armControlRightControlFragment;
    ArmControlLeftControlFragment armControlLeftControlFragment;

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


        optionsMenuButton = (ImageButton) findViewById(R.id.optionsMenuButton);
        optionsMenuButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                BaseInputConnection  mInputConnection = new BaseInputConnection(optionsMenuButton, true);
                KeyEvent kd = new KeyEvent(KeyEvent.ACTION_DOWN, KeyEvent.KEYCODE_MENU);
                KeyEvent ku = new KeyEvent(KeyEvent.ACTION_UP, KeyEvent.KEYCODE_MENU);
                mInputConnection.sendKeyEvent(kd);
                mInputConnection.sendKeyEvent(ku);
            }
        });
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

    @SuppressLint("RestrictedApi")
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_valter_direct_controls, menu);
        menu.getItem(4).setIcon(R.drawable.joystick);
        if(menu instanceof MenuBuilder){
            MenuBuilder m = (MenuBuilder) menu;
            m.setOptionalIconsVisible(true);
        }
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
            savedHost = "192.168.0.101";
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
            savedHost = "192.168.0.101";
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
            savedHost = "192.168.0.102";
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
            savedHost = "192.168.0.102";
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

    public boolean valterACRItemClick(MenuItem item) {
        armControlRightControlFragment = new ArmControlRightControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, armControlRightControlFragment).commit();
        return true;
    }

    public boolean valterACLItemClick(MenuItem item) {
        armControlLeftControlFragment = new ArmControlLeftControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, armControlLeftControlFragment).commit();
        return true;
    }

    public boolean valterPMIBItemClick(MenuItem item) {
        platformManipulatorAndIRBumperControlFragment = new PlatformManipulatorAndIRBumperControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, platformManipulatorAndIRBumperControlFragment).commit();
        return true;
    }

    public boolean valterCommandsItemClick(MenuItem item)
    {
        ValterCommandsActivity.dialogMode = true;
        Intent myIntent = new Intent(getApplicationContext(), ValterCommandsActivity.class);
        startActivityForResult(myIntent, 0);
        return true;
    }

    public boolean valterTasksItemClick(MenuItem item)
    {
        ValterCommandsActivity.dialogMode = true;
        Intent myIntent = new Intent(getApplicationContext(), ValterTasksActivity.class);
        startActivityForResult(myIntent, 0);
        return true;
    }

}
