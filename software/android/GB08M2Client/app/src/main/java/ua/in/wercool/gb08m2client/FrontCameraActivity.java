package ua.in.wercool.gb08m2client;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.PorterDuff;
import android.os.Handler;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.support.v7.widget.Toolbar;
import android.util.Log;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ProgressBar;

public class FrontCameraActivity extends AppCompatActivity {

    private MjpegView fronCameraView;
    public ProgressBar progressBar;

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
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_front_camera);

//        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
//        setSupportActionBar(toolbar);

        progressBar = (ProgressBar) findViewById(R.id.progressBar);

        fronCameraView = (MjpegView) findViewById(R.id.frontCameraView);
        fronCameraView.setAdjustHeight(true);
        fronCameraView.setMode(MjpegView.MODE_BEST_FIT);
        fronCameraView.setCallerActivity(this);
        fronCameraView.setProgressBar(progressBar);

        SharedPreferences settings = getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        String savedHost = settings.getString("GB08M2Host", "109.87.34.156");

        fronCameraView.setUrl("http://" + savedHost + ":8080/?action=stream");
//        fronCameraView.setUrl("http://200.36.58.250/mjpg/video.mjpg?resolution=640x480");
        fronCameraView.setRecycleBitmap(true);


        //Motors Controls

        leftForward = (Button) findViewById(R.id.leftForward);
        leftForward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "LEFT FWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionForwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        leftForward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "LEFT FWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1)
                    {
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        leftForward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        forward = (Button) findViewById(R.id.forward);
        forward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "L and R FWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionForwardCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionForwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        forward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "L and R FWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {

                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);

                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1 || GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().setCurtLeftMotorDuty(1);
                        GB08M2.getInstance().setCurRightMotorDuty(1);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        forward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        rightForward = (Button) findViewById(R.id.rightForward);
        rightForward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "RIGHT FWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionForwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        rightForward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "RIGHT FWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);
                    if (GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        rightForward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        left = (Button) findViewById(R.id.left);
        left.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "Turn LEFT - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionBackwardCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionForwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        left.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "Turn LEFT - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {

                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);

                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1 || GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().setCurtLeftMotorDuty(1);
                        GB08M2.getInstance().setCurRightMotorDuty(1);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        left.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        stop = (Button) findViewById(R.id.stop);
        stop.setOnTouchListener(new View.OnTouchListener() {
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
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                    GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        right = (Button) findViewById(R.id.right);
        right.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "Turn RIGHT - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionForwardCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionBackwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        right.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "Turn RIGHT - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {

                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);

                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1 || GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().setCurtLeftMotorDuty(1);
                        GB08M2.getInstance().setCurRightMotorDuty(1);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        right.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        leftBackward = (Button) findViewById(R.id.leftBackward);
        leftBackward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "LEFT BWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionBackwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        leftBackward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "LEFT BWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1)
                    {
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        leftBackward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        backward = (Button) findViewById(R.id.backward);
        backward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "L and R BWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsDirectionBackwardCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionBackwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        backward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "L and R BWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() + 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {

                    GB08M2.getInstance().setCurtLeftMotorDuty(GB08M2.getInstance().getCurLeftMotorDuty() - 1);
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);

                    if (GB08M2.getInstance().getCurLeftMotorDuty() == 1 || GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().setCurtLeftMotorDuty(1);
                        GB08M2.getInstance().setCurRightMotorDuty(1);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().leftMotorsStopCommand);
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        backward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });

        rightBackward = (Button) findViewById(R.id.rightBackward);
        rightBackward.setOnTouchListener(new View.OnTouchListener() {
            private Handler increaseHandler;
            private Handler decreaseHandler;
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        if (increaseHandler != null) return true;
                        Log.i("GB08M2", "RIGHT BWD - ACCELERATE");

                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsDirectionBackwardCommand);

                        if (decreaseHandler != null)
                        {
                            decreaseHandler.removeCallbacks(decreaseAction);
                            decreaseHandler = null;
                        }

                        increaseHandler = new Handler();
                        increaseHandler.postDelayed(increaseAction, 25);

                        rightBackward.getBackground().setColorFilter(Color.GREEN, PorterDuff.Mode.MULTIPLY);
                        break;
                    case MotionEvent.ACTION_UP:
                        if (increaseHandler == null) return true;
                        Log.i("GB08M2", "RIGHT BWD - DECELERATE");

                        increaseHandler.removeCallbacks(increaseAction);
                        increaseHandler = null;

                        decreaseHandler = new Handler();
                        decreaseHandler.postDelayed(decreaseAction, 1);
                        break;
                }
                return true;
            }

            Runnable increaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() + 1);
                    increaseHandler.postDelayed(this, 25);
                }
            };
            Runnable decreaseAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setCurRightMotorDuty(GB08M2.getInstance().getCurRightMotorDuty() - 1);
                    if (GB08M2.getInstance().getCurRightMotorDuty() == 1)
                    {
                        GB08M2.getInstance().sendCMD(GB08M2.getInstance().rightMotorsStopCommand);
                        rightBackward.getBackground().clearColorFilter();
                    }
                    else
                    {
                        decreaseHandler.postDelayed(this, 1);
                    }
                }
            };
        });



        leftDutyIndicator = (ProgressBar) findViewById(R.id.leftDutyIndicator);
        leftDutyIndicator.setProgress(GB08M2.getInstance().getLeftMotorDuty());
        leftDutyIncreasebutton = (ImageButton) findViewById(R.id.leftDutyIncreasebutton);
        leftDutyIncreasebutton.setOnTouchListener(new View.OnTouchListener() {
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
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setLeftMotorDuty(GB08M2.getInstance().getLeftMotorDuty() + 1);
                    leftDutyIndicator.setProgress(GB08M2.getInstance().getLeftMotorDuty());
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        leftDutyDecreasebutton = (ImageButton) findViewById(R.id.leftDutyDecreasebutton);
        leftDutyDecreasebutton.setOnTouchListener(new View.OnTouchListener() {
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
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setLeftMotorDuty(GB08M2.getInstance().getLeftMotorDuty() - 1);
                    leftDutyIndicator.setProgress(GB08M2.getInstance().getLeftMotorDuty());
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        rightDutyIndicator = (ProgressBar) findViewById(R.id.rightDutyIndicator);
        rightDutyIndicator.setProgress(GB08M2.getInstance().getRightMotorDuty());
        rightDutyIncreasebutton = (ImageButton) findViewById(R.id.rightDutyIncreasebutton);
        rightDutyIncreasebutton.setOnTouchListener(new View.OnTouchListener() {
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
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setRightMotorDuty(GB08M2.getInstance().getRightMotorDuty() + 1);
                    rightDutyIndicator.setProgress(GB08M2.getInstance().getRightMotorDuty());
                    mHandler.postDelayed(this, 5);
                }
            };
        });

        rightDutyDecreasebutton = (ImageButton) findViewById(R.id.rightDutyDecreasebutton);
        rightDutyDecreasebutton.setOnTouchListener(new View.OnTouchListener() {
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
                        break;
                }
                return false;
            }

            Runnable mAction = new Runnable() {
                @Override public void run() {
                    GB08M2.getInstance().setRightMotorDuty(GB08M2.getInstance().getRightMotorDuty() - 1);
                    rightDutyIndicator.setProgress(GB08M2.getInstance().getRightMotorDuty());
                    mHandler.postDelayed(this, 5);
                }
            };
        });
    }

//    @Override
//    public boolean onCreateOptionsMenu(Menu menu) {
//        // Inflate the menu; this adds items to the action bar if it is present.
//        getMenuInflater().inflate(R.menu.main_menu, menu);
//        return true;
//    }

    @Override
    protected void onResume() {
        fronCameraView.startStream();
        super.onResume();
    }

    @Override
    protected void onPause() {
        fronCameraView.stopStream();
        super.onPause();
    }

    @Override
    protected void onStop() {
        fronCameraView.stopStream();
        super.onStop();
    }
}
