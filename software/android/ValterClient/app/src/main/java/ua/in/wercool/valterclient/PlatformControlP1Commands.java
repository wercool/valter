package ua.in.wercool.valterclient;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import valter.Valter;


public class PlatformControlP1Commands extends Fragment {

    Button stopAllButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_platform_control_p1_commands, container, false);

        stopAllButton = (Button) rootView.findViewById(R.id.stopAllButton);
        stopAllButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Log.i("PCP1", "STOP ALL");
            }
        });

        return rootView;
    }
}
