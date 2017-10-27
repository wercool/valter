package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.Spinner;

public class SettingsActivity extends AppCompatActivity {

    public static final String PREFS_NAME = "ValterClientPreferences";

    Spinner hostsSpinner;
    Button connectButton;
    Button disconnectButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

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
                ValterWebSocketClient.getInstance().disconnect();
            }
        });
    }
}