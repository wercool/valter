package ua.in.wercool.valterclient;


import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ImageButton;

public class ArmControlLeftControlFragment extends Fragment {

    ImageButton forearmUpButton;
    ImageButton forearmDownButton;

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,  Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_arm_control_left_control, container, false);

        return rootView;
    }

}
