package pro.valter.valterclient;

import android.provider.Settings;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;

import java.util.Locale;

public class ManualNavigationActivity extends AppCompatActivity {

    ValterClient valterClient;
    float linVel = 0; // * 0.12
    float angVel = 0; // * 0.587
    float prevLinVel;
    float prevAngVel;
    long prev_time;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_manual_navigation);

        valterClient = (ValterClient) getApplication();
        prev_time = System.currentTimeMillis();

        JoystickView joystick = (JoystickView) findViewById(R.id.joystickView);
        joystick.setOnMoveListener(new JoystickView.OnMoveListener() {
            @Override
            public void onMove(int angle, int strength) {
                long cur_time = System.currentTimeMillis();
                if (cur_time - prev_time > 150 || angle == 0 || strength == 0)
                {
                    Log.d("ValterClient", "Angle: " + angle + " Strength: " + (double) strength / 100);

                    linVel = (float) (((double) strength / 100) * (angle > 180 ? -1 : 1) * 0.12);
                    angVel = (float) (Math.cos((double) angle * Math.PI / 180) * (angle > 0 ? -1 : 0) * 0.587);

                    ((TextView) findViewById(R.id.linearVelocity)).setText(String.format(Locale.US, "%.3f", linVel));
                    ((TextView) findViewById(R.id.angularVelocity)).setText(String.format(Locale.US, "%.3f", angVel));

                    if (Math.abs(prevLinVel - linVel) > 0.005 || Math.abs(prevAngVel - angVel) > 0.005) {
                        String msg = String.format(Locale.US, "%.3f,%.3f", linVel, angVel);

                        Log.d("ValterClient", msg);

                        if (valterClient.mConnectedThread != null) {
                            valterClient.mConnectedThread.write(msg.getBytes());
                        }

                        prevLinVel = linVel;
                        prevAngVel = angVel;
                    }
                    prev_time = System.currentTimeMillis();
                }
            }
        });
    }
}
