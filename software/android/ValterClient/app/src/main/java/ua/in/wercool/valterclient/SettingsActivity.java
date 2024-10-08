package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.os.StrictMode;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;

import valter.Valter;

public class SettingsActivity extends AppCompatActivity {

    public static final String PREFS_NAME = "ValterClientPreferences";

    Spinner hostsSpinner;
    Button connectButton;
    Button disconnectButton;

    Button startFrontalCamButton;
    Button stopFrontalCamButton;
    Button startRearCamButton;
    Button stopRearCamButton;
    Button startHeadLEyeCamButton;
    Button stopHeadLEyeCamButton;
    Button startHeadREyeCamButton;
    Button stopHeadREyeCamButton;
    Button startFrontMicStreamButton;
    Button stopFrontMicStreamButton;
    Button startFrontMicStreamPlaybackButton;
    Button stopFrontMicStreamPlaybackButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        hostsSpinner = (Spinner) findViewById(R.id.hostsSpinner);
        String[] hosts = getResources().getStringArray(R.array.hosts);
        ArrayAdapter<String> hostsSpinnerAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_dropdown_item_1line, hosts);
        hostsSpinner.setAdapter(hostsSpinnerAdapter);

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("ValterHost", "109.87.34.156");

        int selectedIndex = 1;
        for (int i = 0; i < hosts.length; i++)
        {
            if (savedHost.equals(hosts[i]))
            {
                selectedIndex = i;
            }
        }

        hostsSpinner.setSelection(selectedIndex);

        hostsSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                System.out.println("Selected host: " + hostsSpinner.getSelectedItem().toString());
                SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
                SharedPreferences.Editor editor = settings.edit();
                editor.putString("ValterHost", hostsSpinner.getSelectedItem().toString());
                editor.commit();
                ValterWebSocketClient.getInstance().updateURI();
            }
            @Override
            public void onNothingSelected(AdapterView<?> parent) {
//                System.out.println(hostsSpinner.getSelectedItem().toString());
            }
        });

        ValterWebSocketClient.getInstance().setCallerActivity(this);

        connectButton = (Button) findViewById(R.id.connectButton);
        connectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ValterWebSocketClient.getInstance().disconnect();
                ValterWebSocketClient.getInstance().connect();
            }
        });

        disconnectButton = (Button) findViewById(R.id.disconnectButton);
        disconnectButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setHeadCameraRState(false);
                Valter.getInstance().setHeadCameraLState(false);
                Valter.getInstance().setFrontalCameraState(false);
                Valter.getInstance().setFrontMicStreamState(false);
                ValterWebSocketClient.getInstance().disconnect();
            }
        });

        startFrontalCamButton = (Button) findViewById(R.id.startFrontalCamButton);
        startFrontalCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
//                ValterWebSocketClient.getInstance().sendMessage("CTRL#FCAMON");
                Valter.getInstance().setFrontalCameraState(true);
            }
        });

        stopFrontalCamButton = (Button) findViewById(R.id.stopFrontalCamButton);
        stopFrontalCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
//                ValterWebSocketClient.getInstance().sendMessage("CTRL#FCAMOFF");
                Valter.getInstance().setFrontalCameraState(false);
            }
        });

        startRearCamButton = (Button) findViewById(R.id.startRearCamButton);
        startRearCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
//                ValterWebSocketClient.getInstance().sendMessage("CTRL#RCAMON");
            }
        });

        stopRearCamButton = (Button) findViewById(R.id.stopRearCamButton);
        stopRearCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
//                ValterWebSocketClient.getInstance().sendMessage("CTRL#RCAMOFF");
            }
        });

        startHeadREyeCamButton = (Button) findViewById(R.id.startHeadREyeCamButton);
        startHeadREyeCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setHeadCameraRState(true);
            }
        });

        stopHeadREyeCamButton = (Button) findViewById(R.id.stopHeadREyeCamButton);
        stopHeadREyeCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setHeadCameraRState(false);
            }
        });

        startHeadLEyeCamButton = (Button) findViewById(R.id.startHeadLEyeCamButton);
        startHeadLEyeCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setHeadCameraLState(true);
            }
        });

        stopHeadLEyeCamButton = (Button) findViewById(R.id.stopHeadLEyeCamButton);
        stopHeadLEyeCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setHeadCameraLState(false);
            }
        });


        startFrontMicStreamButton = (Button) findViewById(R.id.startFrontMicStreamButton);
        startFrontMicStreamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setFrontMicStreamState(true);
            }
        });

        stopFrontMicStreamButton = (Button) findViewById(R.id.stopFrontMicStreamButton);
        stopFrontMicStreamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().setFrontMicStreamState(false);
            }
        });


//        startFrontMicStreamPlaybackButton = (Button) findViewById(R.id.startFrontMicStreamPlaybackButton);
//        startFrontMicStreamPlaybackButton.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                Valter.getInstance().startFrontMicStreamPlayback();
//            }
//        });
//
//        stopFrontMicStreamPlaybackButton = (Button) findViewById(R.id.stopFrontMicStreamPlaybackButton);
//        stopFrontMicStreamPlaybackButton.setOnClickListener(new View.OnClickListener() {
//            @Override
//            public void onClick(View v) {
//                Valter.getInstance().stopFrontMicStreamPlayback();
//            }
//        });

    }
}
