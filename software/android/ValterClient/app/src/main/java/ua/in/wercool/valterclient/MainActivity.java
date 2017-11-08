package ua.in.wercool.valterclient;

import android.content.Context;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

public class MainActivity extends AppCompatActivity {

    private static Context appContext;
    public static Context getContext()
    {
        return appContext;
    }

    Button valterTasksButton;
    Button chargerStationControlButton;
    Button settingButton;
    Button chargerSettingsButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        appContext = this;

        valterTasksButton = (Button) findViewById(R.id.valterTasksButton);
        valterTasksButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), ValterTasksActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        chargerStationControlButton = (Button) findViewById(R.id.chargerStationControlButton);
        chargerStationControlButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), ChargerStationActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        settingButton = (Button) findViewById(R.id.settingsButton);
        settingButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), SettingsActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        chargerSettingsButton = (Button) findViewById(R.id.chargerSettingsButton);
        chargerSettingsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), ChargerSettingsActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });
    }
}
