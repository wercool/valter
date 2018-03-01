package ua.in.wercool.valterclient;


import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import valter.Valter;


public class BodyControlP1Commands extends Fragment {

    Button stopAllButton;
    Button headLedOnButton;
    Button headLedOffButton;
    Button leftAccumulatorOnButton;
    Button leftAccumulatorOffButton;
    Button rightAccumulatorOnButton;
    Button rightAccumulatorOffButton;
    Button headMotorsActivateButton;
    Button headMotorsDeactivateButton;
    Button head24VOnButton;
    Button head24VOffButton;
    Button headYawOnButton;
    Button headYawOffButton;
    Button headPitchOnButton;
    Button headPitchOffButton;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_body_control_p1_commands, container, false);

        stopAllButton = (Button) rootView.findViewById(R.id.stopAllButton);
        stopAllButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "STOP ALL");
            }
        });

        headLedOnButton = (Button) rootView.findViewById(R.id.headLedOnButton);
        headLedOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD LED ON");
                Valter.getInstance().setHeadLedState(true);
            }
        });

        headLedOffButton = (Button) rootView.findViewById(R.id.headLedOffButton);
        headLedOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD LED OFF");
                Valter.getInstance().setHeadLedState(false);
            }
        });

        leftAccumulatorOnButton = (Button) rootView.findViewById(R.id.leftAccumulatorOnButton);
        leftAccumulatorOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "LEFT ACCUMULATOR ON");
                Valter.getInstance().setLeftAccumulatorState(true);
            }
        });

        leftAccumulatorOffButton = (Button) rootView.findViewById(R.id.leftAccumulatorOffButton);
        leftAccumulatorOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "LEFT ACCUMULATOR OFF");
                Valter.getInstance().setLeftAccumulatorState(false);
            }
        });

        rightAccumulatorOnButton = (Button) rootView.findViewById(R.id.rightAccumulatorOnButton);
        rightAccumulatorOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "RIGHT ACCUMULATOR ON");
                Valter.getInstance().setRightAccumulatorState(true);
            }
        });

        rightAccumulatorOffButton = (Button) rootView.findViewById(R.id.rightAccumulatorOffButton);
        rightAccumulatorOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "RIGHT ACCUMULATOR OFF");
                Valter.getInstance().setRightAccumulatorState(false);
            }
        });

        headMotorsActivateButton = (Button) rootView.findViewById(R.id.headMotorsActivateButton);
        headMotorsActivateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD MOTORS ACTIVATED");
                Valter.getInstance().setHeadMotorsActivatedState(true);
            }
        });

        headMotorsDeactivateButton = (Button) rootView.findViewById(R.id.headMotorsDeactivateButton);
        headMotorsDeactivateButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD MOTORS DEACTIVATED");
                Valter.getInstance().setHeadMotorsActivatedState(false);
            }
        });

        head24VOnButton = (Button) rootView.findViewById(R.id.head24VOnButton);
        head24VOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD 24V ON");
                Valter.getInstance().setHead24VState(true);
            }
        });

        head24VOffButton = (Button) rootView.findViewById(R.id.head24VOffButton);
        head24VOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD 24V OFF");
                Valter.getInstance().setHead24VState(false);
            }
        });

        headYawOnButton = (Button) rootView.findViewById(R.id.headYawOnButton);
        headYawOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD YAW ON");
                Valter.getInstance().setHeadYawMotorState(true);
            }
        });

        headYawOffButton = (Button) rootView.findViewById(R.id.headYawOffButton);
        headYawOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD YAW OFF");
                Valter.getInstance().setHeadYawMotorState(false);
            }
        });

        headPitchOnButton = (Button) rootView.findViewById(R.id.headPitchOnButton);
        headPitchOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD PITCH ON");
                Valter.getInstance().setHeadPitchMotorState(true);
            }
        });

        headPitchOffButton = (Button) rootView.findViewById(R.id.headPitchOffButton);
        headPitchOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("BCP1", "HEAD PITCH OFF");
                Valter.getInstance().setHeadPitchMotorState(false);
            }
        });

        return rootView;
    }

}
