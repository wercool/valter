package ua.in.wercool.valterclient;

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

    ImageButton leftDutyIncreaseButton;
    ImageButton leftDutyDecreaseButton;
    ProgressBar leftDutyIndicator;

    ImageButton rightDutyIncreaseButton;
    ImageButton rightDutyDecreaseButton;
    ProgressBar rightDutyIndicator;

    boolean setDutyLock = true;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        /** Inflating the layout for this fragment **/
        View rootView = inflater.inflate(R.layout.valter_manual_navigation_fragment, null);

        leftDutyIndicator = (ProgressBar) rootView.findViewById(R.id.leftDutyIndicator);
        rightDutyIndicator = (ProgressBar) rootView.findViewById(R.id.rightDutyIndicator);

        leftDutyIndicator.setAlpha((float)0.5);
        rightDutyIndicator.setAlpha((float)0.5);

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

        leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());
        leftDutyIndicator.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        setDutyLock = !setDutyLock;
                        if (setDutyLock) {
                            leftDutyIndicator.setAlpha((float)0.5);
                            rightDutyIndicator.setAlpha((float)0.5);
                        }
                        else {
                            leftDutyIndicator.setAlpha((float)1.0);
                            rightDutyIndicator.setAlpha((float)1.0);
                        }
                    break;
                }
                return true;
            }
        });


        leftDutyIncreaseButton = (ImageButton) rootView.findViewById(R.id.leftDutyIncreaseButton);
        leftDutyIncreaseButton.setOnTouchListener(new View.OnTouchListener() {
            private Handler mHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (mHandler != null) return true;
                        mHandler = new Handler();
                        mHandler.postDelayed(mAction, 5);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (mHandler == null) return true;
                        mHandler.removeCallbacks(mAction);
                        mHandler = null;
                        Valter.getInstance().sendMessage(String.format("PCP1#LMD=%d", Valter.getInstance().getLeftMotorDuty()));
                        if (setDutyLock) {
                            Valter.getInstance().sendMessage(String.format("PCP1#RMD=%d", Valter.getInstance().getRightMotorDuty()));
                        }
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    Valter.getInstance().changeLeftMotorDuty(1);
                    leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());
                    if (setDutyLock) {
                        Valter.getInstance().changeRightMotorDuty(1);
                        rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());
                    }
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        leftDutyDecreaseButton = (ImageButton) rootView.findViewById(R.id.leftDutyDecreaseButton);
        leftDutyDecreaseButton.setOnTouchListener(new View.OnTouchListener() {
            private Handler mHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (mHandler != null) return true;
                        mHandler = new Handler();
                        mHandler.postDelayed(mAction, 5);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (mHandler == null) return true;
                        mHandler.removeCallbacks(mAction);
                        mHandler = null;
                        Valter.getInstance().sendMessage(String.format("PCP1#LMD=%d", Valter.getInstance().getLeftMotorDuty()));
                        if (setDutyLock) {
                            Valter.getInstance().sendMessage(String.format("PCP1#RMD=%d", Valter.getInstance().getRightMotorDuty()));
                        }
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    Valter.getInstance().changeLeftMotorDuty(-1);
                    leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());
                    if (setDutyLock) {
                        Valter.getInstance().changeRightMotorDuty(-1);
                        rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());
                    }
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());
        rightDutyIndicator.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        setDutyLock = !setDutyLock;
                        if (setDutyLock) {
                            leftDutyIndicator.setAlpha((float)0.5);
                            rightDutyIndicator.setAlpha((float)0.5);
                        }
                        else {
                            leftDutyIndicator.setAlpha((float)1.0);
                            rightDutyIndicator.setAlpha((float)1.0);
                        }
                        break;
                }
                return true;
            }
        });

        rightDutyIncreaseButton = (ImageButton) rootView.findViewById(R.id.rightDutyIncreaseButton);
        rightDutyIncreaseButton.setOnTouchListener(new View.OnTouchListener() {
            private Handler mHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (mHandler != null) return true;
                        mHandler = new Handler();
                        mHandler.postDelayed(mAction, 5);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (mHandler == null) return true;
                        mHandler.removeCallbacks(mAction);
                        mHandler = null;
                        Valter.getInstance().sendMessage(String.format("PCP1#RMD=%d", Valter.getInstance().getRightMotorDuty()));
                        if (setDutyLock) {
                            Valter.getInstance().sendMessage(String.format("PCP1#LMD=%d", Valter.getInstance().getLeftMotorDuty()));
                        }
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    Valter.getInstance().changeRightMotorDuty(1);
                    rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());
                    if (setDutyLock) {
                        Valter.getInstance().changeLeftMotorDuty(1);
                        leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());
                    }
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        rightDutyDecreaseButton = (ImageButton) rootView.findViewById(R.id.rightDutyDecreaseButton);
        rightDutyDecreaseButton.setOnTouchListener(new View.OnTouchListener() {
            private Handler mHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (mHandler != null) return true;
                        mHandler = new Handler();
                        mHandler.postDelayed(mAction, 5);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (mHandler == null) return true;
                        mHandler.removeCallbacks(mAction);
                        mHandler = null;
                        Valter.getInstance().sendMessage(String.format("PCP1#RMD=%d", Valter.getInstance().getRightMotorDuty()));
                        if (setDutyLock) {
                            Valter.getInstance().sendMessage(String.format("PCP1#LMD=%d", Valter.getInstance().getLeftMotorDuty()));
                        }
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    Valter.getInstance().changeRightMotorDuty(-1);
                    rightDutyIndicator.setProgress(Valter.getInstance().getRightMotorDuty());
                    if (setDutyLock) {
                        Valter.getInstance().changeLeftMotorDuty(-1);
                        leftDutyIndicator.setProgress(Valter.getInstance().getLeftMotorDuty());
                    }
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        return rootView;
    }
}
