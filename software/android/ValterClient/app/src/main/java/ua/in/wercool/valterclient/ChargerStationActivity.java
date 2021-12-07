package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.ProgressBar;

public class ChargerStationActivity extends AppCompatActivity {

    private MjpegView chargerCameraView;
    public ProgressBar progressBar;

    ChargerStationCamControl chargerStationCamControlFragment;
    ValterManualNavigation valterManualNavigationFragment;


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
        chargerCameraView.setRecycleBitmap(true);

        chargerStationCamControlFragment = new ChargerStationCamControl();
        valterManualNavigationFragment = new ValterManualNavigation();

        getSupportFragmentManager().beginTransaction().add(R.id.fragmentContainer, chargerStationCamControlFragment).commit();
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

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.charger_station_menu, menu);
        return true;
    }

    public boolean onChargerStationCamControlClick(MenuItem item)
    {
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, chargerStationCamControlFragment).commit();
        return true;
    }

    public boolean onChargerStationValterControlClick(MenuItem item)
    {
        getSupportFragmentManager().beginTransaction().replace(R.id.fragmentContainer, valterManualNavigationFragment).commit();
        return true;
    }
}
