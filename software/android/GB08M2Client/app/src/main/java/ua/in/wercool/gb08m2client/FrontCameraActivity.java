package ua.in.wercool.gb08m2client;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;

public class FrontCameraActivity extends AppCompatActivity {

    private MjpegView fronCameraView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_front_camera);

        fronCameraView = (MjpegView) findViewById(R.id.frontCameraView);
        fronCameraView.setAdjustHeight(true);
        fronCameraView.setMode(MjpegView.MODE_FIT_WIDTH);
        fronCameraView.setUrl("http://109.87.34.156:8080/?action=stream");
//        fronCameraView.setUrl("http://bma-itic1.iticfoundation.org/mjpeg2.php?camid=61.91.182.114:1111");
//        fronCameraView.setUrl("http://200.36.58.250/mjpg/video.mjpg?resolution=640x480");
        fronCameraView.setRecycleBitmap(true);
    }

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
