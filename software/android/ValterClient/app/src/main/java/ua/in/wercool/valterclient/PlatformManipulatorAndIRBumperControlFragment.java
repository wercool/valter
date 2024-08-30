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

public class PlatformManipulatorAndIRBumperControlFragment extends Fragment {

    ImageButton link2UpButton;
    ImageButton link2DownButton;

    ImageButton link1UpButton;
    ImageButton link1DownButton;
    ImageButton gripperCCWButton;
    ImageButton gripperCWButton;


    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {

        View rootView = inflater.inflate(R.layout.fragment_platform_manipulator_and_irbumper_control, container, false);

        link2DownButton = (ImageButton) rootView.findViewById(R.id.link2DownButton);
        link2DownButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        link2DownButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibMoveLink2Down();
                        break;
                    case MotionEvent.ACTION_UP:
                        link2DownButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibLink2Stop();
                        break;
                }
                return true;
            }
        });

        link2UpButton = (ImageButton) rootView.findViewById(R.id.link2UpButton);
        link2UpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        link2UpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibMoveLink2Up();
                        break;
                    case MotionEvent.ACTION_UP:
                        link2UpButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibLink2Stop();
                        break;
                }
                return true;
            }
        });

        link1DownButton = (ImageButton) rootView.findViewById(R.id.link2DownButton);
        link1DownButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        link1DownButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibMoveLink1Down();
                        break;
                    case MotionEvent.ACTION_UP:
                        link1DownButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibLink1Stop();
                        break;
                }
                return true;
            }
        });

        link1UpButton = (ImageButton) rootView.findViewById(R.id.link2UpButton);
        link1UpButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        link1UpButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibMoveLink1Up();
                        break;
                    case MotionEvent.ACTION_UP:
                        link1UpButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibLink1Stop();
                        break;
                }
                return true;
            }
        });

        gripperCCWButton = (ImageButton) rootView.findViewById(R.id.gripperCCWButton);
        gripperCCWButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        gripperCCWButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibGripperCCW();
                        break;
                    case MotionEvent.ACTION_UP:
                        gripperCCWButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibGripperStop();
                        break;
                }
                return true;
            }
        });

        gripperCWButton = (ImageButton) rootView.findViewById(R.id.gripperCCWButton);
        gripperCWButton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        gripperCWButton.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        Valter.getInstance().pmibGripperCW();
                        break;
                    case MotionEvent.ACTION_UP:
                        gripperCWButton.getBackground().clearColorFilter();
                        Valter.getInstance().pmibGripperStop();
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

}
