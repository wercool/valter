package ua.in.wercool.valterclient;


import android.graphics.Color;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;

import valter.Valter;

public class ArmControlRightControlFragment extends Fragment {

    ImageButton forearmUpButton;
    ImageButton forearmDownButton;
    ImageButton closeHandButton;
    ImageButton openHandButton;
    ImageButton forearmRollCCWButton;
    ImageButton forearmRollCWButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_right_control, container, false);


        forearmUpButton = (ImageButton) rootView.findViewById(R.id.forearmUpButton);
        forearmUpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        forearmUpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().rightForearmDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        forearmUpButton.getBackground().clearColorFilter();
                        Valter.getInstance().rightForearmDONE();
                        break;
                }
                return true;
            }
        });

        forearmDownButton = (ImageButton) rootView.findViewById(R.id.forearmDownButton);
        forearmDownButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        forearmDownButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().rightForearmDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        forearmDownButton.getBackground().clearColorFilter();
                        Valter.getInstance().rightForearmDONE();
                        break;
                }
                return true;
            }
        });

        closeHandButton = (ImageButton) rootView.findViewById(R.id.closeHandButton);
        closeHandButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().rightHandClose();
            }
        });

        openHandButton = (ImageButton) rootView.findViewById(R.id.openHandButton);
        openHandButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Valter.getInstance().rightHandOpen();
            }
        });

        forearmRollCCWButton = (ImageButton) rootView.findViewById(R.id.forearmRollCCWButton);
        forearmRollCCWButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        forearmRollCCWButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().rightForearmRollDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        forearmRollCCWButton.getBackground().clearColorFilter();
                        Valter.getInstance().rightForearmRollDONE();
                        break;
                }
                return true;
            }
        });

        forearmRollCWButton = (ImageButton) rootView.findViewById(R.id.forearmRollCWButton);
        forearmRollCWButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        forearmRollCWButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().rightForearmRollDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        forearmRollCWButton.getBackground().clearColorFilter();
                        Valter.getInstance().rightForearmRollDONE();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
