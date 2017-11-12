package ua.in.wercool.valterclient;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.MotionEvent;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;


public class SphereCamControl extends Fragment {

    Button up;
    Button down;
    Button left;
    Button right;
    Button center;

    String savedHost;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        /** Inflating the layout for this fragment **/
        View rootView = inflater.inflate(R.layout.sphere_cam_control_fragment, null);

        SharedPreferences settings = this.getActivity().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
        savedHost = settings.getString("GB08M2Host", "109.87.34.156");

        up = (Button) rootView.findViewById(R.id.up);
        up.setOnTouchListener(new View.OnTouchListener() {
            @Override
            public boolean onTouch(View v, MotionEvent event) {
                switch(event.getAction()) {
                    case MotionEvent.ACTION_DOWN:
                        doGETHTTPRequest("http://" + savedHost + ":8080/?action=command_ng&dest=0&plugin=0&id=10094853&group=1&value=-200");
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
                        doGETHTTPRequest("http://" + savedHost + ":8080/?action=command_ng&dest=0&plugin=0&id=10094853&group=1&value=200");
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
                        doGETHTTPRequest("http://" + savedHost + ":8080/?action=command_ng&dest=0&plugin=0&id=10094852&group=1&value=200");
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
                        doGETHTTPRequest("http://" + savedHost +":8080/?action=command_ng&dest=0&plugin=0&id=10094852&group=1&value=-200");
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
                        doGETHTTPRequest("http://" + savedHost + ":8080/?action=command_ng&dest=0&plugin=0&id=168062211&group=1&value=1");
                        break;
                }
                return true;
            }
        });

        return rootView;
    }

    private void doGETHTTPRequest(String url_str)
    {
        URL url = null;
        HttpURLConnection urlConnection = null;
        try {
            url = new URL(url_str);
            try {
                urlConnection = (HttpURLConnection) url.openConnection();
                int responseCode = urlConnection.getResponseCode();
                System.out.println("HTTP Response:" + responseCode);
            } catch (IOException e) {
                e.printStackTrace();
            }

        } catch (MalformedURLException e) {
            e.printStackTrace();
        } finally {
            if (urlConnection != null)
            {
                urlConnection.disconnect();
            }
        }
    }
}
