package ua.in.wercool.valterclient;


import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageView;

import valter.Valter;

public class PlatformJoystickControlFragment extends Fragment {
    private ImageView joystickHandle;
    private ImageView joystickBase;
    private float centerX, centerY;
    private int joystickRadius;
    private Handler handler = new Handler();
    private Runnable sendCmdVelRunnable;
    private long lastUpdateTime = 0;
    private static final long UPDATE_INTERVAL = 100; // 100 ms

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.platform_joystick_platform_control, container, false);

        joystickHandle = (ImageView) rootView.findViewById(R.id.joystickHandle);
        joystickBase = (ImageView) rootView.findViewById(R.id.joystickBase);

        joystickBase.post(new Runnable() {
            @Override
            public void run() {
                // Get the center position of the joystick base
                centerX = joystickBase.getWidth() / 2f;
                centerY = joystickBase.getHeight() / 2f;
                joystickRadius = (int) Math.round(joystickBase.getWidth() / 2.25);

                // Center the joystick handle at the start
                joystickHandle.setX(centerX - joystickHandle.getWidth() / 2f);
                joystickHandle.setY(centerY - joystickHandle.getHeight() / 2f);
            }
        });

        joystickHandle.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                float rawX = event.getRawX();
                float rawY = event.getRawY();

                // Get the relative position inside the joystick base
                int[] location = new int[2];
                joystickBase.getLocationOnScreen(location);

                float x = rawX - location[0];
                float y = rawY - location[1];

                switch (event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                    case MotionEvent.ACTION_MOVE:
                        updateJoystickPosition(x, y);
                        break;
                    case MotionEvent.ACTION_UP:
                        // Reset the joystick handle to the center when released
                        joystickHandle.setX(centerX - joystickHandle.getWidth() / 2f);
                        joystickHandle.setY(centerY - joystickHandle.getHeight() / 2f);
                        Valter.getInstance().sendCmdVel(
                                0.0,
                                0.0
                        );
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

    private void updateJoystickPosition(float x, float y) {
        float dx = x - centerX;
        float dy = y - centerY;
        double distance = Math.sqrt(dx * dx + dy * dy);

        if (distance > joystickRadius) {
            dx = (float) ((dx / distance) * joystickRadius);
            dy = (float) ((dy / distance) * joystickRadius);
        }

        joystickHandle.setX(centerX + dx - joystickHandle.getWidth() / 2f);
        joystickHandle.setY(centerY + dy - joystickHandle.getHeight() / 2f);

        // Normalize the values to get velocity and angle in the required ranges
        double velocity = -dy / joystickRadius;  // Forward is positive, backward is negative
        double angle = -dx / joystickRadius;      // Right is positive, left is negative

        // Clamp values to the range [-1, 1]
        velocity = Math.max(-1, Math.min(1, velocity));
        angle = Math.max(-1, Math.min(1, angle));

        long currentTime = System.currentTimeMillis();
        if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
            lastUpdateTime = currentTime;

            // Log the velocity and angle values
            Log.i("Joystick", "Velocity: " + velocity + ", Angle: " + angle);

            // Send the command
            Valter.getInstance().sendCmdVel(
                    0.12 * velocity,
                    0.5 * angle
            );
        }
    }
}





