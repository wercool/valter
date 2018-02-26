package ua.in.wercool.valterclient;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import valter.Valter;

public class PlatformLocationp1Commands extends Fragment {

    Button sonarLedsOnButton;
    Button sonarLedsOffButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_platform_locationp1_commands, container, false);

        sonarLedsOnButton = (Button) rootView.findViewById(R.id.sonarLedsOnButton);
        sonarLedsOnButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("PCP1", "SONAR LEDS ON");
                Valter.getInstance().setSonarLedsState(true);
            }
        });


        sonarLedsOffButton = (Button) rootView.findViewById(R.id.sonarLedsOffButton);
        sonarLedsOffButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("PCP1", "SONAR LEDS OFF");
                Valter.getInstance().setSonarLedsState(false);
            }
        });

        return rootView;
    }
}
