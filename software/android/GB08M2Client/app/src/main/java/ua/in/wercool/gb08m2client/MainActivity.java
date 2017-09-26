package ua.in.wercool.gb08m2client;

import android.content.Intent;
import android.os.StrictMode;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

public class MainActivity extends AppCompatActivity {

    Button frontCamButton;
    Button rearCamButton;
    Button commandsButton;
    Button settingButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        StrictMode.ThreadPolicy policy = new StrictMode.ThreadPolicy.Builder().permitAll().build();
        StrictMode.setThreadPolicy(policy);

        GB08M2.getInstance().setCallerActivity(this);


        frontCamButton = (Button) findViewById(R.id.frontCamButton);
        frontCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), FrontCameraActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        rearCamButton = (Button) findViewById(R.id.rearCamButton);
        rearCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), RearCameraActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        commandsButton = (Button) findViewById(R.id.commandsButton);
        commandsButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), CommandsActivity.class);
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
    }
}
