package ua.in.wercool.gb08m2client;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;

public class MainActivity extends AppCompatActivity {

    Button frontCamButton;
    Button settingButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        settingButton = (Button) findViewById(R.id.settingsButton);
        settingButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), SettingsActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });

        frontCamButton = (Button) findViewById(R.id.frontCamButton);
        frontCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent myIntent = new Intent(getApplicationContext(), FrontCameraActivity.class);
                startActivityForResult(myIntent, 0);
            }
        });
    }
}
