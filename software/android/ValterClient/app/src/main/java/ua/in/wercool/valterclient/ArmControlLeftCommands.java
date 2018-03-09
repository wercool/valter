package ua.in.wercool.valterclient;


import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import valter.Valter;

public class ArmControlLeftCommands extends Fragment {

    Button stopAllButton;
    Button stopAllWatchersButton;
    Button startAllWatchersButton;
    Button rollMotorOnButton;
    Button rollMotorOffButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_left_commands, container, false);

        stopAllButton = (Button) rootView.findViewById(R.id.stopAllButton);
        stopAllButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACL", "ARM LEFT STOP ALL");
                Valter.getInstance().ACLStopAll();
            }
        });

        stopAllWatchersButton = (Button) rootView.findViewById(R.id.stopAllWatchersButton);
        stopAllWatchersButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACL", "STOP ALL LEFT ARM WATCHERS");
                Valter.getInstance().ACLSetWatcherState(false);
            }
        });

        startAllWatchersButton = (Button) rootView.findViewById(R.id.startAllWatchersButton);
        startAllWatchersButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACL", "START ALL LEFT ARM WATCHERS");
                Valter.getInstance().ACLSetWatcherState(true);
            }
        });

        rollMotorOnButton = (Button) rootView.findViewById(R.id.rollMotorOnButton);
        rollMotorOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACL", "ARM LEFT ROLL MOTOR ON");
                Valter.getInstance().setLeftArmRollMotorState(true);
            }
        });

        rollMotorOffButton = (Button) rootView.findViewById(R.id.rollMotorOffButton);
        rollMotorOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("ACL", "ARM LEFT ROLL MOTOR OFF");
                Valter.getInstance().setLeftArmRollMotorState(false);
            }
        });

        return rootView;
    }

}
