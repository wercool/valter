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
    PlatformJoystickControlFragment platformJoystickControlFragment;
    HeadYawPitchControlFragment headYawPitchControlFragment;
    PlatformManipulatorAndIRBumperControlFragment platformManipulatorAndIRBumperControlFragment;
    ArmControlRightControlFragment armControlRightControlFragment;
    ArmControlLeftControlFragment armControlLeftControlFragment;

    TorsoControlFragment torsoControlFragment;

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
        if(menu instanceof MenuBuilder){
            MenuBuilder m = (MenuBuilder) menu;
            m.setOptionalIconsVisible(true);
        }
        return true;
    }

    public void onFrontalCameraItemClick(MenuItem item)
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
    }

    public void onRearCameraItemClick(MenuItem item)
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
    }

    public void onHeadLCameraItemClick(MenuItem item)
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
    }

    public void onHeadRCameraItemClick(MenuItem item)
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
    }

    public void valterManualSteeringItemClick(MenuItem item)
    {
        valterManualNavigationFragment = new ValterManualNavigation();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, valterManualNavigationFragment).commit();
    }

    public void valterJoystickControlItemClick(MenuItem item)
    {
        platformJoystickControlFragment = new PlatformJoystickControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, platformJoystickControlFragment).commit();
    }

    public void valterHeadYawPitchItemClick(MenuItem item)
    {
        headYawPitchControlFragment = new HeadYawPitchControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, headYawPitchControlFragment).commit();
    }

    public void valterACRItemClick(MenuItem item) {
        armControlRightControlFragment = new ArmControlRightControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, armControlRightControlFragment).commit();
    }

    public void valterACLItemClick(MenuItem item) {
        armControlLeftControlFragment = new ArmControlLeftControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, armControlLeftControlFragment).commit();
    }

    public void valterTorsoItemClick(MenuItem item) {
        torsoControlFragment = new TorsoControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, torsoControlFragment).commit();
    }

    public void valterPMIBItemClick(MenuItem item) {
        platformManipulatorAndIRBumperControlFragment = new PlatformManipulatorAndIRBumperControlFragment();
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, platformManipulatorAndIRBumperControlFragment).commit();
    }

    public void valterCommandsItemClick(MenuItem item)
    {
        ValterCommandsActivity.dialogMode = true;
        Intent myIntent = new Intent(getApplicationContext(), ValterCommandsActivity.class);
        startActivityForResult(myIntent, 0);
    }

    public void valterScriptsItemClick(MenuItem item)
    {
        Intent myIntent = new Intent(getApplicationContext(), ValterScriptsActivity.class);
        startActivityForResult(myIntent, 0);
    }

    public void valterTasksItemClick(MenuItem item)
    {
        ValterCommandsActivity.dialogMode = true;
        Intent myIntent = new Intent(getApplicationContext(), ValterTasksActivity.class);
        startActivityForResult(myIntent, 0);
    }
}
