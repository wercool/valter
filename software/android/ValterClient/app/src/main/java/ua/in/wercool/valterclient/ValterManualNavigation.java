package ua.in.wercool.valterclient;

import android.content.Context;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.net.Uri;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ProgressBar;

import valter.Valter;


public class ValterManualNavigation extends Fragment {

    Button leftForward;
    Button forward;
    Button rightForward;
    Button left;
    Button stop;
    Button right;
    Button leftBackward;
    Button backward;
    Button rightBackward;

    ImageButton leftDutyIncreasebutton;
    ImageButton leftDutyDecreasebutton;
    ProgressBar leftDutyIndicator;

    ImageButton rightDutyIncreasebutton;
    ImageButton rightDutyDecreasebutton;
    ProgressBar rightDutyIndicator;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        /** Inflating the layout for this fragment **/
        View rootView = inflater.inflate(R.layout.valter_manual_navigation_fragment, null);

        leftForward = (Button) rootView.findViewById(R.id.leftForward);
        leftForward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#LF");
                        leftForward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LD");
                        leftForward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        forward = (Button) rootView.findViewById(R.id.forward);
        forward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#LRF");
                        forward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LRD");
                        forward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        rightForward = (Button) rootView.findViewById(R.id.rightForward);
        rightForward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#RF");
                        rightForward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#RD");
                        rightForward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        left = (Button) rootView.findViewById(R.id.left);
        left.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#RL");
                        left.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LRD");
                        left.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        stop = (Button) rootView.findViewById(R.id.stop);
        stop.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#STOP");
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#STOP");
                        break;
                }
                return false;
            }
        });

        right = (Button) rootView.findViewById(R.id.right);
        right.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#RR");
                        right.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LRD");
                        right.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        leftBackward = (Button) rootView.findViewById(R.id.leftBackward);
        leftBackward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#LB");
                        leftBackward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LD");
                        leftBackward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        backward = (Button) rootView.findViewById(R.id.backward);
        backward.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#LRB");
                        backward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#LRD");
                        backward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        rightBackward = (Button) rootView.findViewById(R.id.rightBackward);
        rightBackward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        Valter.getInstance().sendMessage("PCP1#RB");
                        rightBackward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        Valter.getInstance().sendMessage("PCP1#RD");
                        rightBackward.getBackground().clearColorFilter();
                        break;
                }
                return true;
            }
        });

        leftDutyIndicator = (ProgressBar) rootView.findViewById(R.id.leftDutyIndicator);
//        leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());

        leftDutyIncreasebutton = (ImageButton) rootView.findViewById(R.id.leftDutyIncreasebutton);
        leftDutyIncreasebutton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        break;
                    case MotionEvent.ACTION_UP:
                        break;
                }
                return false;
            }
        });

        leftDutyDecreasebutton = (ImageButton) rootView.findViewById(R.id.leftDutyDecreasebutton);
        leftDutyDecreasebutton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        break;
                    case MotionEvent.ACTION_UP:
                        break;
                }
                return false;
            }
        });

        rightDutyIndicator = (ProgressBar) rootView.findViewById(R.id.rightDutyIndicator);
//        rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());

        rightDutyIncreasebutton = (ImageButton) rootView.findViewById(R.id.rightDutyIncreasebutton);
        rightDutyIncreasebutton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        break;
                    case MotionEvent.ACTION_UP:
                        break;
                }
                return false;
            }
        });

        rightDutyDecreasebutton = (ImageButton) rootView.findViewById(R.id.rightDutyDecreasebutton);
        rightDutyDecreasebutton.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        break;
                    case MotionEvent.ACTION_UP:
                        break;
                }
                return false;
            }
        });

        return rootView;
    }
}
