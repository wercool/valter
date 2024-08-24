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

        return rootView;
    }

}
