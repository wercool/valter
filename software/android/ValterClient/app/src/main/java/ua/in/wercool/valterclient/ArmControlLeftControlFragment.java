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

public class ArmControlLeftControlFragment extends Fragment {

    ImageButton forearmUpButton;
    ImageButton forearmDownButton;
    ImageButton armUpButton;
    ImageButton armDownButton;
    ImageButton limbUpButton;
    ImageButton limbDownButton;
    ImageButton forearmRollCCWButton;
    ImageButton forearmRollCWButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_left_control, container, false);

        forearmUpButton = (ImageButton) rootView.findViewById(R.id.forearmUpButton);
        forearmUpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        forearmUpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().leftForearmDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        forearmUpButton.getBackground().clearColorFilter();
                        Valter.getInstance().leftForearmDONE();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
