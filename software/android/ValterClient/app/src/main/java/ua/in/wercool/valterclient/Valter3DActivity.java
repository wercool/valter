package ua.in.wercool.valterclient;

import android.app.Activity;
import android.app.ActivityManager;
import android.content.Context;
import android.content.pm.ConfigurationInfo;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Toast;

import javax.microedition.khronos.opengles.GL10;

/**
 * Created by maska on 1/6/18.
 */

public class Valter3DActivity extends Activity {

    private Valter3DGLSurfaceView valter3DGLSurfaceView;
    private Valter3DGLRenderer valter3DGLRenderer;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        ActivityManager am = (ActivityManager ) getSystemService(Context.ACTIVITY_SERVICE);
        ConfigurationInfo info = am.getDeviceConfigurationInfo();
        boolean supportES2 = (info.reqGlEsVersion >= 0x00020000);
        if (supportES2 ) {
            valter3DGLRenderer = new Valter3DGLRenderer();

            valter3DGLSurfaceView = new Valter3DGLSurfaceView(this);
            valter3DGLSurfaceView.setEGLContextClientVersion(2);
            valter3DGLSurfaceView.setRenderer(valter3DGLRenderer);
            this.setContentView(valter3DGLSurfaceView);
        } else {
            Log.e("OpenGLES 2", "This device does not support ES2. (" + info.reqGlEsVersion + ")");
            Toast.makeText(this, "This device does not support ES2. (" + info.reqGlEsVersion + ")", Toast.LENGTH_LONG).show();
        }
    }
}
