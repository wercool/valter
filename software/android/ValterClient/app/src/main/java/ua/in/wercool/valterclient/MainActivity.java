package ua.in.wercool.valterclient;

import android.content.ComponentCallbacks2;
import android.content.Context;
import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import org.java_websocket.client.WebSocketClient;

import valter.Valter;

public class MainActivity extends AppCompatActivity {

    private static Context appContext;
    public static Context getContext()
    {
        return appContext;
    }

    Button valterDirectControlsButton;
    Button valterTasksButton;
    Button chargerStationControlButton;
    Button settingButton;
    Button chargerSettingsButton;
    Button valterCommandsButton;
    Button voiceControlButton;
    Button valter3DButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        appContext = this;

        valterDirectControlsButton = (Button) findViewById(R.id.valterDirectControlsButton);
        valterDirectControlsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), ValterDirectControlActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        valterTasksButton = (Button) findViewById(R.id.valterTasksButton);
        valterTasksButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), ValterTasksActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        valterCommandsButton = (Button) findViewById(R.id.valterCommandsButton);
        valterCommandsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ValterCommandsActivity.dialogMode = false;
                Intent myIntent = new Intent(getApplicationContext(), ValterCommandsActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        voiceControlButton = (Button) findViewById(R.id.voiceControlButton);
        voiceControlButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), VoiceControlActivity.class);
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

        valter3DButton = (Button) findViewById(R.id.valter3DButton);
        valter3DButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), Valter3DActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });
    }

    @Override
    public void onTrimMemory(int level) {
        super.onTrimMemory(level);
        if(level == ComponentCallbacks2.TRIM_MEMORY_UI_HIDDEN) {
            Log.i("ValterClient", "WENT BACKGROUND");
            ValterWebSocketClient.getInstance().disconnect();
        }
    }
}
