package ua.in.wercool.valterclient;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import valter.Valter;

public class ArmControlRightCommands extends Fragment {

    Button stopAllButton;
    Button stopAllWatchersButton;
    Button startAllWatchersButton;
    Button rollMotorOnButton;
    Button rollMotorOffButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_right_commands, container, false);

        stopAllButton = (Button) rootView.findViewById(R.id.stopAllButton);
        stopAllButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACR", "ARM RIGHT STOP ALL");
                Valter.getInstance().ACRStopAll();
            }
        });

        stopAllWatchersButton = (Button) rootView.findViewById(R.id.stopAllWatchersButton);
        stopAllWatchersButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACR", "STOP ALL RIGHT ARM WATCHERS");
                Valter.getInstance().ACRSetWatcherState(false);
            }
        });

        startAllWatchersButton = (Button) rootView.findViewById(R.id.startAllWatchersButton);
        startAllWatchersButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACR", "START ALL RIGHT ARM WATCHERS");
                Valter.getInstance().ACRSetWatcherState(true);
            }
        });

        rollMotorOnButton = (Button) rootView.findViewById(R.id.rollMotorOnButton);
        rollMotorOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACR", "ARM RIGHT ROLL MOTOR ON");
                Valter.getInstance().setRightArmRollMotorState(true);
            }
        });

        rollMotorOffButton = (Button) rootView.findViewById(R.id.rollMotorOffButton);
        rollMotorOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACR", "ARM RIGHT ROLL MOTOR OFF");
                Valter.getInstance().setRightArmRollMotorState(false);
            }
        });

        return rootView;
    }
}
