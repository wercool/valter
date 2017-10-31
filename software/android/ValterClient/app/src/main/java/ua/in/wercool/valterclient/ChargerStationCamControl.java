package ua.in.wercool.valterclient;

import android.content.Context;
import android.net.Uri;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;


public class ChargerStationCamControl extends Fragment {

    Button up;
    Button down;
    Button left;
    Button right;
    Button center;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        /** Inflating the layout for this fragment **/
        View rootView = inflater.inflate(R.layout.charger_station_cam_control_fragment, null);


        up = (Button) rootView.findViewById(R.id.up);
        up.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVTILTUP");
                        break;
                }
                return true;
            }
        });

        down = (Button) rootView.findViewById(R.id.down);
        down.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVTILTDOWN");
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
                        ChargerStationClient.getInstance().sendMessage("SRVYAWLEFT");
                        break;
                }
                return true;
            }
        });

        right = (Button) rootView.findViewById(R.id.right);
        right.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("SRVYAWRIGHT");
                        break;
                }
                return true;
            }
        });

        center = (Button) rootView.findViewById(R.id.center);
        center.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        ChargerStationClient.getInstance().sendMessage("CAMCENTER");
                        break;
                }
                return true;
            }
        });

        return rootView;
    }
}
