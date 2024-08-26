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

public class TorsoControlFragment extends Fragment {
    ImageButton torsoYawLeftButton;
    ImageButton torsoYawRightButton;
    ImageButton bodyPitchDownButton;
    ImageButton bodyPitchUpButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_torso_control, container, false);

        torsoYawLeftButton = (ImageButton) rootView.findViewById(R.id.torsoYawLeftButton);
        torsoYawLeftButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        torsoYawLeftButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().torsoYawDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        torsoYawLeftButton.getBackground().clearColorFilter();
                        Valter.getInstance().torsoYawDONE();
                        break;
                }
                return true;
            }
        });

        torsoYawRightButton = (ImageButton) rootView.findViewById(R.id.torsoYawRightButton);
        torsoYawRightButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        torsoYawRightButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().torsoYawDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        torsoYawRightButton.getBackground().clearColorFilter();
                        Valter.getInstance().torsoYawDONE();
                        break;
                }
                return true;
            }
        });

        bodyPitchDownButton = (ImageButton) rootView.findViewById(R.id.bodyPitchDownButton);
        bodyPitchDownButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        bodyPitchDownButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().bodyPitchDO(true);
                        break;
                    case MotionEvent.ACTION_UP:
                        bodyPitchDownButton.getBackground().clearColorFilter();
                        Valter.getInstance().bodyPitchDONE();
                        break;
                }
                return true;
            }
        });

        bodyPitchUpButton = (ImageButton) rootView.findViewById(R.id.bodyPitchUpButton);
        bodyPitchUpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        bodyPitchUpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().bodyPitchDO(false);
                        break;
                    case MotionEvent.ACTION_UP:
                        bodyPitchUpButton.getBackground().clearColorFilter();
                        Valter.getInstance().bodyPitchDONE();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
