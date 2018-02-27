package ua.in.wercool.valterclient;


import android.graphics.Color;
import android.graphics.PorterDuff;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;

import ua.in.wercool.valterclient.R;
import valter.Valter;


public class HeadYawPitchControlFragment extends Fragment {

    ImageButton headYawLeftButton;
    ImageButton headYawRightButton;
    ImageButton headPitchDownButton;
    ImageButton headPitchUpButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_head_yaw_pitch_control, container, false);

        headYawLeftButton = (ImageButton) rootView.findViewById(R.id.headYawLeftButton);
        headYawLeftButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        headYawLeftButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().headYawDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        headYawLeftButton.getBackground().clearColorFilter();
                        Valter.getInstance().headYawDONE();
                        break;
                }
                return true;
            }
        });

        headYawRightButton = (ImageButton) rootView.findViewById(R.id.headYawRightButton);
        headYawRightButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        headYawRightButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().headYawDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        headYawRightButton.getBackground().clearColorFilter();
                        Valter.getInstance().headYawDONE();
                        break;
                }
                return true;
            }
        });

        headPitchDownButton = (ImageButton) rootView.findViewById(R.id.headPitchDownButton);
        headPitchDownButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        headPitchDownButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().headPitchDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        headPitchDownButton.getBackground().clearColorFilter();
                        Valter.getInstance().headPitchDONE();
                        break;
                }
                return true;
            }
        });

        headPitchUpButton = (ImageButton) rootView.findViewById(R.id.headPitchUpButton);
        headPitchUpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        headPitchUpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().headPitchDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        headPitchUpButton.getBackground().clearColorFilter();
                        Valter.getInstance().headPitchDONE();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
