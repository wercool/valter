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

    ImageButton armUpButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_right_control, container, false);

        armUpButton = (ImageButton) rootView.findViewById(R.id.armUpButton);
        armUpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        armUpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().headYawDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        armUpButton.getBackground().clearColorFilter();
                        Valter.getInstance().headYawDONE();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
