package ua.in.wercool.gb08m2client;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class CommandsActivity extends AppCompatActivity {

    Button startFrontCamButton;
    Button stopFrontCamButton;
    Button startRearCamButton;
    Button stopRearCamButton;
    Button lightsOnButton;
    Button lightsOffButton;
    Button batVoltageButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_commands);

        GB08M2.getInstance().setCallerActivity(this);

        batVoltageButton = (Button) findViewById(R.id.batVoltageButton);
        batVoltageButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.batteryVoltageCommand);
            }
        });

        startFrontCamButton = (Button) findViewById(R.id.startFrontCamButton);
        startFrontCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.startFrontCamCommand);
            }
        });

        stopFrontCamButton = (Button) findViewById(R.id.stopFrontCamButton);
        stopFrontCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.stopFrontCamCommand);
            }
        });

        startRearCamButton = (Button) findViewById(R.id.startRearCamButton);
        startRearCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.startRearCamCommand);
            }
        });

        stopRearCamButton = (Button) findViewById(R.id.stopRearCamButton);
        stopRearCamButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.stopRearCamCommand);
            }
        });

        lightsOnButton = (Button) findViewById(R.id.lightsOnButton);
        lightsOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.lightsOnCommand);
            }
        });

        lightsOffButton = (Button) findViewById(R.id.lightsOffButton);
        lightsOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                GB08M2.getInstance().sendCMD(GB08M2.lightsOffCommand);
            }
        });

    }

    @Override
    protected void onResume() {
        super.onResume();
        GB08M2.getInstance().setCallerActivity(this);
    }

}
